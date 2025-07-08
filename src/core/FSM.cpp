#include "core/FSM.h"

/*-----------------------------------------------------------------------------
 FSM data constructor
-----------------------------------------------------------------------------*/
systemData::systemData() :
    APPS1(PIN_APPS_ONE, false),
    APPS2(PIN_APPS_TWO, false),
    BSE(PIN_BSE, true),

    // Pump control for battery cooling - Unused when EV1 is active
    pump(PIN_PUMP, 0.0, 0.0, 0.0),

    pinRTDButton(PIN_RTD_BUTTON, BUTTON_DEBOUNCE_TIME, INPUT),
    pinSDCTap(PIN_SHUTDOWN_TAP, INPUT, ANALOG_PIN_BUFFER_SIZE),

    pinRUN(PIN_RUN, OUTPUT),
    pinRFE(PIN_RFE, OUTPUT),
    pinRTDBuzzer(PIN_RTD_SOUND, OUTPUT),
    pinBrakeLight(PIN_BRAKE_LIGHT, OUTPUT),
    pinReset(PIN_RESET, OUTPUT),

    // EV1 additional Teensy pins - Unused when EV1.5 is active
    pinAIRPlus(PIN_AIR_PLUS, OUTPUT),

    // EV1.5 additional Teensy pins - Unused when EV1 is active
    pinPump(PIN_PUMP, OUTPUT),
    pinPumpSwitch(PIN_PUMP_SWITCH, OUTPUT),
    pinFaultLED(PIN_LED_FAULT, OUTPUT)
{
    // Reset timers
    timers.b100msPassed = 0;
    timers.buzzerTimer = 0;
    timers.chargeTimer = 0;
    timers.resetTimer = 0;

    // Reset timer flags
    timers.bChargeTimerStarted = false;
    timers.bBuzzerActive = false;
    timers.bResetTimerStarted = false;
}

/*-----------------------------------------------------------------------------
 FSM system constructor
-----------------------------------------------------------------------------*/
systemFSM::systemFSM() {
    // Assign current state to the reset state (load pedal configuration)
    state = &systemFSM::PEDALS;
}

/*-----------------------------------------------------------------------------
 FSM system constructor
-----------------------------------------------------------------------------*/
void systemFSM::ProcessState() {
    // Get the latest reading on the pedals and SDC tap
    system.UpdatePedalStructures();
    system.UpdateSDCTapBuffer();

    // Update the brake light
    system.ActivateBrakeLight();

    // Determine to run pump based on motor temperature
    system.RunPump();

    // Check for pedal sensor errors
    system.Set100msFlag( system.CheckPedalImplausibility() );

    // Update the fault error bits
    system.SetFaultBuffer( IRQHandler::GetErrorBuffer() );

    // Execute the current state the member function pointer points to
    (this->*state)();
}

/*-----------------------------------------------------------------------------
 PEDALS State - Load pedal configuration
-----------------------------------------------------------------------------*/
void systemFSM::PEDALS() {
    // Check for a successful load of pedal bounds
    if ( !system.SetPedalBounds() ) {
        DebugErrorPrint("ERROR: PEDAL BOUNDS NOT SET");

        // Enable fault LED to indicate error
        IRQHandler::EnableFaultLEDTimer();

        // Wait for 5 seconds to show error state
        delay(5000);
        
        // Trigger immediate system reset
        IRQHandler::ResetWDT();
    }

    // Assign member function pointer to next state
    state = &systemFSM::INIT;
    DebugPrintln("STATE: INIT");
}

/*-----------------------------------------------------------------------------
 INIT State - Activate system reset
-----------------------------------------------------------------------------*/
void systemFSM::INIT() {
    // Start the reset timer on first iteration
    if ( !system.GetResetTimerFlag() ) {
        system.SetResetTimerFlag(true);
        system.SetResetTimer(0);
        return;
    }

    // Wait until the reset time elapses
    if ( system.GetResetTimer() < RESET_TIME ) {
        return;
    }

    // Disable reset (active low)
    system.GetResetPin().WriteOutput(HIGH);

    DebugPrintln("SHUTDOWN CIRCUIT RESET");

    // Assign member function pointer to next state
    state = &systemFSM::PRECHARGE;
    DebugPrintln("STATE: PRECHARGE");
}

/*-----------------------------------------------------------------------------
 PRECHARGE State - Wait for the tractive system to be energized
-----------------------------------------------------------------------------*/
void systemFSM::PRECHARGE() {
    analogPin pinSDCTap = system.GetSDCTapPin();

    system.SetStateBuffer(systemState::PRECHARGE);

    // Begin pedal calibration when startup detected
    if ( IRQHandler::GetButtonHeld() ) {
        // Reset charge flag
        system.SetChargeTimerFlag(false); 

        // Use fault LED on ECU to indicate calibration mode
		IRQHandler::EnableCalibrationTimer();

        state = &systemFSM::CALIBRATE_PEDALS;
        DebugPrintln("STATE: CALIBRATE PEDALS");
        return;
    }

    // Wait until shutdown tap is high
    if ( pinSDCTap.GetBuffer().GetAverage() < SDC_TAP_HIGH ) {
        return;
    }

    // Start precharge timer on first iteration
    if ( !system.GetChargeTimerFlag() ) {
        system.SetChargeTimer(0);
        system.SetChargeTimerFlag(true);
        return;
    }

    // Wait until precharge time has elapsed
    if ( system.GetChargeTimer() < CHARGE_TIME ) {
        return;
    }

    // Set AIR plus pin high
    EV1_AIR_PLUS_HIGH();
    
    // Reset charge flag
    system.SetChargeTimerFlag(false);

    DebugPrintln("SYSTEM CHARGED");

    // Transition to RTD when vehicle is powered
    state = &systemFSM::RTD;
    DebugPrintln("STATE: RTD");
}

/*-----------------------------------------------------------------------------
 RTD State - Wait for the driver to activate the vehicle
-----------------------------------------------------------------------------*/
void systemFSM::RTD() {
    digitalPin pinRTDBuzzer = system.GetRTDBuzzerPin();

    system.SetStateBuffer(systemState::RTD);

    // Transition to FAULT if any possible error occurs
    if ( system.CheckAllErrors() ) {
		// Use fault LED on ECU to indicate calibration mode
		IRQHandler::EnableFaultLEDTimer();

        // Output ECU errors
        DebugPrintVehicleErrors(system);

        state = &systemFSM::FAULT;
        DebugPrintln("STATE: FAULT");
        return;
    }

    // Begin motor calibration when startup detected
    if ( IRQHandler::GetButtonHeld() ) {
        // Use fault LED on ECU to indicate calibration mode
		IRQHandler::EnableCalibrationTimer();

        state = &systemFSM::CALIBRATE_MOTOR;
        DebugPrintln("STATE: CALIBRATE MOTOR");
        return;
    }
    
    // Await driver input to activate vehicle
    if ( system.ReadyToDrive() && !system.GetBuzzerTimerFlag() ) {
        // Set motor controller enable signals high
        system.ActivateBamocar();

        // Activate buzzer pin
        pinRTDBuzzer.WriteOutput(HIGH);

        // Begin timer for buzzer
        system.SetBuzzerTimer(0);
        system.SetBuzzerTimerFlag(true);

        // Remove calibration controls
        detachInterrupt( system.GetRTDButtonPin().GetPin() );
    }

    // Play buzzer for 1 second and transition to IDLE
    if ( system.GetBuzzerTimerFlag() && system.GetBuzzerTimer() >= BUZZER_TIME ) {
        // Disable buzzer and flag
        pinRTDBuzzer.WriteOutput(LOW);
        system.SetBuzzerTimerFlag(false);

        state = &systemFSM::IDLE;
        DebugPrintln("STATE: IDLE");
    }
}

/*-----------------------------------------------------------------------------
 IDLE State - Wait for the driver pedal input to drive or brake
-----------------------------------------------------------------------------*/
void systemFSM::IDLE() {
    CAN_message_t msgTorque;
    uint8_t torqueBuf[PAR_RX_DLC] = {0, 0, 0};

    system.SetStateBuffer(systemState::IDLE);

    // Transition to FAULT if any possible error occurs
    if ( system.CheckAllErrors() ) {
		// Begin toggling fault LED on ECU PCB
		IRQHandler::EnableFaultLEDTimer();

        // Output ECU errors
        DebugPrintVehicleErrors(system);

        state = &systemFSM::FAULT;
        DebugPrintln("STATE: FAULT");
        return;
    }

    // Switch to DRIVE state after surpassing throttle threshold
    if ( system.GetLowerPercentAPPS() * 100 > PERCENT_THRESHOLD ) {
        state = &systemFSM::DRIVE;
        DebugPrintln("STATE: DRIVE");
        return;
    } 

    // Switch to BRAKE state after surpassing brake threshold
    if ( system.GetBSE().GetPercentRequest() * 100 > PERCENT_BRAKE ) {
        state = &systemFSM::BRAKE;
        DebugPrintln("STATE: BRAKE");
        return;
    }

	// SKIPPING DURING TEST BENCHING
    // Send torque command of zero to Bamocar
    PopulateCANMessage(&msgTorque, ID_CAN_MESSAGE_RX, PAR_RX_DLC, torqueBuf, REG_DIG_TORQUE_SET);
    SendCANMessage(msgTorque);
}

/*-----------------------------------------------------------------------------
 DRIVE State - Wait for the driver pedal input to drive or brake
-----------------------------------------------------------------------------*/
void systemFSM::DRIVE() {
    CAN_message_t msgTorque;
    uint8_t torqueBuf[PAR_RX_DLC] = {0, 0, 0};

    system.SetStateBuffer(systemState::DRIVE);

    // Transition to FAULT if any possible error occurs
    if ( system.CheckAllErrors() ) {
		// Begin toggling fault LED on ECU PCB
		IRQHandler::EnableFaultLEDTimer();

        // Output ECU errors
        DebugPrintVehicleErrors(system);

        state = &systemFSM::FAULT;
        DebugPrintln("STATE: FAULT");
        return;
    }

    // Transition to IDLE state if APPS receive minimal force (braking)
    if ( system.GetLowerPercentAPPS() * 100 < PERCENT_THRESHOLD ) {
        state = &systemFSM::IDLE;
        DebugPrintln("STATE: IDLE");
        return;
    }

    // Update CAN message buffer with updated pedal readings
    system.ProcessAPPS(torqueBuf);

	// SKIPPING DURING TEST BENCHING
    // Send torque command to Bamocar
    PopulateCANMessage(&msgTorque, ID_CAN_MESSAGE_RX, PAR_RX_DLC, torqueBuf, REG_DIG_TORQUE_SET);
    SendCANMessage(msgTorque);
}

/*-----------------------------------------------------------------------------
 BRAKE State - Apply zero torque to motor and slow down vehicle
-----------------------------------------------------------------------------*/
void systemFSM::BRAKE() {
    CAN_message_t msgTorque;
    uint8_t torqueBuf[PAR_RX_DLC] = {0, 0, 0};

    system.SetStateBuffer(systemState::BRAKE);

    // Transition to FAULT if any possible error occurs
    if ( system.CheckAllErrors() ) {
		// Begin toggling fault LED on ECU PCB
		IRQHandler::EnableFaultLEDTimer();

        // Output ECU errors
        DebugPrintVehicleErrors(system);

        state = &systemFSM::FAULT;
        DebugPrintln("STATE: FAULT");
        return;
    }

    // Transition to IDLE state if BSE receives minimal force
    if ( system.GetBSE().GetPercentRequest() * 100 < PERCENT_BRAKE ) {
        state = &systemFSM::IDLE;
        DebugPrintln("STATE: IDLE");
        return;
    }

	// SKIPPING DURING TEST BENCHING
    // Send torque command of zero to Bamocar
    PopulateCANMessage(&msgTorque, ID_CAN_MESSAGE_RX, PAR_RX_DLC, torqueBuf, REG_DIG_TORQUE_SET);
    SendCANMessage(msgTorque);
}

/*-----------------------------------------------------------------------------
 FAULT State - Shut off power to the motor when an error occurs
-----------------------------------------------------------------------------*/
void systemFSM::FAULT() {
    analogPin pinSDCTap = system.GetSDCTapPin();
    uint8_t faultBuf = system.GetFaultBuffer();

    system.SetStateBuffer(systemState::FAULT);

    // Disable power to motor controller 
    system.DeactivateBamocar();

    // System cannot be re-activated if the pedal sensors disagree or are out of range
    if ( PedalsDisagree() || PedalsOOR() ) {
        return;
    }

    // APPS / Brake Pedal Plausability Check resolved
    if ( BothPedalsPressed() && system.GetLowerPercentAPPS() * 100 < PLAUSIBILITY_CHECK ) {
        // Re-activate motor controller enable signals
        system.ActivateBamocar();
        
        // Clear both pedals pressed error bit
        IRQHandler::SetErrorBuffer( faultBuf & ~(1 << ERROR_CODE_APPS_BSE) );

        // Disable fault LED on ECU PCB when leaving FAULT state
        IRQHandler::DisableFaultLEDTimer();

        // Revert to IDLE state
        state = &systemFSM::IDLE;
        DebugPrintln("STATE: IDLE");
        return;
    }

    // Check shutdown tap closes again
    if ( ShutdownCircuitOpen() && pinSDCTap.GetBuffer().GetAverage() >= SDC_TAP_HIGH ) {
        // Clear shutdown open error bit
        IRQHandler::SetErrorBuffer( faultBuf & ~(1 << ERROR_CODE_SHUTDOWN) );
        IRQHandler::SetShutdownState(false);

        // Disable fault LED on ECU PCB when leaving FAULT state
        IRQHandler::DisableFaultLEDTimer();

        // Revert to RTD state
        state = &systemFSM::RTD;
        DebugPrintln("STATE: RTD");
        return;
    }
}

/*-----------------------------------------------------------------------------
 CALIBRATE PEDALS State - Re-configure pedal sensors
-----------------------------------------------------------------------------*/
void systemFSM::CALIBRATE_PEDALS() {
	DebugPrintln("BEGINNING PEDAL CALIBARTION...");

    digitalPin pinRTDButton = system.GetRTDButtonPin();

    // Disable RTD button interrupt during pedal calibration
    detachInterrupt( pinRTDButton.GetPin() );

    // Begin calibration
    system.CalibratePedals();

    // Set calibration mode flag low
    IRQHandler::SetButtonHeld(false);

    // Disable fault LED serving as calibration mode indicator
    IRQHandler::DisableCalibrationTimer();

    // Set RTD button pin to be interrupt controlled for pedal calibration
    attachInterrupt(digitalPinToInterrupt( pinRTDButton.GetPin() ), RTDButtonISR, CHANGE);

    DebugPrint("APPS1 Lower Bound: "); DebugPrintln( system.GetAPPS1().GetPercentRequestLowerBound() );
    DebugPrint("APPS1 Upper Bound: "); DebugPrintln( system.GetAPPS1().GetPercentRequestUpperBound() );

    DebugPrint("APPS2 Lower Bound: "); DebugPrintln( system.GetAPPS2().GetPercentRequestLowerBound() );
    DebugPrint("APPS2 Upper Bound: "); DebugPrintln( system.GetAPPS2().GetPercentRequestUpperBound() );

    DebugPrint("BSE Lower Bound: "); DebugPrintln( system.GetBSE().GetPercentRequestLowerBound() );
    DebugPrint("BSE Upper Bound: "); DebugPrintln( system.GetBSE().GetPercentRequestUpperBound() );

    // Transition back to PEDALS to read in new encoded values
    state = &systemFSM::PEDALS;
    DebugPrintln("STATE: PEDALS");
}

/*-----------------------------------------------------------------------------
 CALIBRATE MOTOR State - Calibrate motor with Bamocar
-----------------------------------------------------------------------------*/
void systemFSM::CALIBRATE_MOTOR() {
    DebugPrintln("BEGINNING MOTOR CALIBARTION...");

    digitalPin pinRTDButton = system.GetRTDButtonPin();

    // Disable RTD button interrupt during pedal calibration
    detachInterrupt( pinRTDButton.GetPin() );

    // Begin calibration
    system.CalibrateMotor();

    // Set calibration mode flag low
    IRQHandler::SetButtonHeld(false);

    // Disable fault LED serving as calibration mode indicator
    IRQHandler::DisableCalibrationTimer();

    // Set RTD button pin to be interrupt controlled for pedal calibration
    attachInterrupt(digitalPinToInterrupt( pinRTDButton.GetPin() ), RTDButtonISR, CHANGE);

     // Transition back to RTD for driving
    state = &systemFSM::RTD;
    DebugPrintln("STATE: RTD");
}
