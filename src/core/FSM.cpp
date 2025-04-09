#include "core/FSM.h"

/*-----------------------------------------------------------------------------
 FSM data constructor
-----------------------------------------------------------------------------*/
systemData::systemData() :
    APPS1(PIN_APPS_ONE),
    APPS2(PIN_APPS_TWO),
    BSE(PIN_BSE),

    // Pump control for battery cooling - Unused when EV1 is active
    pump(PIN_PUMP, 0.0, 0.0, 0.0),

    pinRTDButton(PIN_RTD_BUTTON, BUTTON_DEBOUNCE_TIME, INPUT),
    pinSDCTap(PIN_SHUTDOWN_TAP, SHUTDOWN_STABLE_TIME, INPUT),

    pinRUN(PIN_RUN, OUTPUT),
    pinGO(PIN_GO, OUTPUT),
    pinRTDBuzzer(PIN_RTD_SOUND, OUTPUT),
    pinBrakeLight(PIN_BRAKE_LIGHT, OUTPUT),
    pinReset(PIN_RESET, OUTPUT),

    // EV1 additional Teensy pins - Unused when EV1.5 is active
    pinAIRPlus(PIN_AIR_PLUS, OUTPUT),

    // EV1.5 additional Teensy pins - Unused when EV1 is active
    pinPump(PIN_PUMP, OUTPUT),
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
    // state = &systemFSM::PEDALS;

    state = &systemFSM::INIT;
}

/*-----------------------------------------------------------------------------
 FSM system constructor
-----------------------------------------------------------------------------*/
void systemFSM::ProcessState() {
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
        EXIT;
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

    // TODO - Temporary for testing
    system.GetAPPS1().SetPercentRequestLowerBound(0);
    system.GetAPPS2().SetPercentRequestLowerBound(0);
    system.GetBSE().SetPercentRequestLowerBound(0);
    system.GetAPPS1().SetPercentRequestUpperBound(65535);
    system.GetAPPS2().SetPercentRequestUpperBound(65535);
    system.GetBSE().SetPercentRequestUpperBound(65535);

    // Activate reset (active low)
    system.GetResetPin().WriteOutput(LOW);

    DebugPrintln("SHUTDOWN CIRCUIT RESET");

    // Assign member function pointer to next state
    state = &systemFSM::PRECHARGE;
    DebugPrintln("STATE: PRECHARGE");
}

/*-----------------------------------------------------------------------------
 PRECHARGE State - Wait for the tractive system to be energized
-----------------------------------------------------------------------------*/
void systemFSM::PRECHARGE() {
    digitalPin & pinSDCTap = system.GetSDCTapPin();

    // Set precharge state value to be sent to dashboard
    system.SetStateBuffer(0);

    // Begin pedal calibration when startup detected
    if ( IRQHandler::GetCalibrationMode() ) {
        // Reset charge flag
        system.SetChargeTimerFlag(false); 

        state = &systemFSM::CALIBRATE;
        DebugPrintln("STATE: CALIBRATE");
        return;
    }

    // Wait until shutdown tap is high
    if ( !pinSDCTap.ReadDebouncedPin() ) {
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

    // Set SDC error to be interrupt controlled
    attachInterrupt(digitalPinToInterrupt( system.GetSDCTapPin().GetPin() ), 
        ShutdownCircuitISR, FALLING);

    // Remove pedal calibration mode control
    detachInterrupt( system.GetRTDButtonPin().GetPin() );
    
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

    // Set the RTD state value to be sent to dashboard
    system.SetStateBuffer(1);

    // Transition to FAULT if any possible error occurs
    if ( system.CheckAllErrors() ) {
		// Begin toggling fault LED on ECU PCB
		IRQHandler::EnableFaultLEDTimer();

        state = &systemFSM::FAULT;
        DebugPrintln("STATE: FAULT");
        return;
    }

    // Await driver input to activate vehicle
    if ( system.ReadyToDrive() ) {
        // Set motor controller enable signals high
        system.ActivateBamocar();

        // Activate buzzer pin
        pinRTDBuzzer.WriteOutput(HIGH);

        // Begin timer for buzzer
        system.SetBuzzerTimer(0);
        system.SetBuzzerTimerFlag(true);
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
    // CAN_message_t msgTorque;
    // uint8_t torqueBuf[PAR_RX_DLC] = {0, 0, 0};

    // Set the IDLE state value to be sent to dashboard
    system.SetStateBuffer(2);

    // Transition to FAULT if any possible error occurs
    if ( system.CheckAllErrors() ) {
		// Begin toggling fault LED on ECU PCB
		IRQHandler::EnableFaultLEDTimer();

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
    // PopulateCANMessage(&msgTorque, ID_CAN_MESSAGE_RX, PAR_RX_DLC, torqueBuf, REG_DIG_TORQUE_SET);
    // SendCANMessage(msgTorque);
}

/*-----------------------------------------------------------------------------
 DRIVE State - Wait for the driver pedal input to drive or brake
-----------------------------------------------------------------------------*/
void systemFSM::DRIVE() {
    CAN_message_t msgTorque;
    uint8_t torqueBuf[PAR_RX_DLC] = {0, 0, 0};

    // Set the DRIVE state value to be sent to dashboard
    system.SetStateBuffer(3);

    // Transition to FAULT if any possible error occurs
    if ( system.CheckAllErrors() ) {
		// Begin toggling fault LED on ECU PCB
		IRQHandler::EnableFaultLEDTimer();

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
    // PopulateCANMessage(&msgTorque, ID_CAN_MESSAGE_RX, PAR_RX_DLC, torqueBuf, REG_DIG_TORQUE_SET);
    // SendCANMessage(msgTorque);
}

/*-----------------------------------------------------------------------------
 BRAKE State - Apply zero torque to motor and slow down vehicle
-----------------------------------------------------------------------------*/
void systemFSM::BRAKE() {
    // CAN_message_t msgTorque;
    // uint8_t torqueBuf[PAR_RX_DLC] = {0, 0, 0};

    // Set the BRAKE state value to be sent to dashboard
    system.SetStateBuffer(4);

    // Transition to FAULT if any possible error occurs
    if ( system.CheckAllErrors() ) {
		// Begin toggling fault LED on ECU PCB
		IRQHandler::EnableFaultLEDTimer();

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
    // PopulateCANMessage(&msgTorque, ID_CAN_MESSAGE_RX, PAR_RX_DLC, torqueBuf, REG_DIG_TORQUE_SET);
    // SendCANMessage(msgTorque);
}

/*-----------------------------------------------------------------------------
 FAULT State - Shut off power to the motor when an error occurs
-----------------------------------------------------------------------------*/
void systemFSM::FAULT() {
    // static bool bErrorsWritten = false;

    digitalPin & pinSDCTap = system.GetSDCTapPin();

    // Update the fault error bits
    system.SetFaultBuffer( IRQHandler::GetErrorBuffer() );
    uint8_t faultBuf = system.GetFaultBuffer();

    // Set the FAULT state value to be sent to dashboard
    system.SetStateBuffer(5);
    
    // Print which errors occurred (binary format) (OOR, BPP, DIS, SDC)
    DebugPrint("VEHICLE ERRORS: ");
    DebugPrint( bitRead(faultBuf, ERROR_CODE_OOR) );
    DebugPrint( bitRead(faultBuf, ERROR_CODE_APPS_BSE) );
    DebugPrint( bitRead(faultBuf, ERROR_CODE_DISAGREE) );
    DebugPrintln( bitRead(faultBuf, ERROR_CODE_SHUTDOWN) );

    // Disable power to motor controller 
    system.DeactivateBamocar();

	// SKIPPING DURING TEST BENCHING
    // Write ECU errors to a data file on the SD card
    // if (!bErrorsWritten) {
    //     ErrorToSD();
        
    //     // Set flag high to only write once
    //     bErrorsWritten = true;
    // }

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

        // bErrorsWritten = false;

        // Revert to IDLE state
        state = &systemFSM::IDLE;
        DebugPrintln("STATE: IDLE");
        return;
    }

    // Check shutdown tap closes again
    if ( ShutdownCircuitOpen() && pinSDCTap.ReadDebouncedPin() ) {
        // Clear shutdown open error bit
        IRQHandler::SetErrorBuffer( faultBuf & ~(1 << ERROR_CODE_SHUTDOWN) );
        IRQHandler::SetShutdownState(false);

        // Disable fault LED on ECU PCB when leaving FAULT state
        IRQHandler::DisableFaultLEDTimer();

        // bErrorsWritten = false;

        // Revert to RTD state
        state = &systemFSM::RTD;
        DebugPrintln("STATE: RTD");
        return;
    }
}

/*-----------------------------------------------------------------------------
 CALIBRATE State - Re-configure pedal sensors
-----------------------------------------------------------------------------*/
void systemFSM::CALIBRATE() {
	DebugPrintln("BEGINNING CALIBARTION...");

    // Get the RTD button pin
    digitalPin & pinRTDButton = system.GetRTDButtonPin();

    // Disable RTD button interrupt during pedal calibration
    detachInterrupt( pinRTDButton.GetPin() );

    // Begin calibration
    system.CalibratePedals();

    // Set calibration mode flag low
    IRQHandler::SetCalibrationMode(false);

    // Set RTD button pin to be interrupt controlled for pedal calibration
    attachInterrupt(digitalPinToInterrupt( pinRTDButton.GetPin() ), PedalCalibrationISR, FALLING);

    // Transition back to PEDALS to read in new encoded values
    state = &systemFSM::PEDALS;
    DebugPrintln("STATE: PEDALS");
}
