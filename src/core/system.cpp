#include "core/FSM.h"

/*-----------------------------------------------------------------------------
 Activate brake light when BSE is pressed
-----------------------------------------------------------------------------*/
void systemData::ActivateBrakeLight(void) {
    // Check if brake is significantly pressed to activate brake light
    if ( BSE.GetPercentRequest() * 100 > PERCENT_BRAKE ) {
        pinBrakeLight.WriteOutput(HIGH);
    } else {
        pinBrakeLight.WriteOutput(LOW);
    }
}

/*----------------------------------------------------------------------------- 
 Update pedal data for all hall sensors
-----------------------------------------------------------------------------*/
void systemData::UpdatePedalStructures(void) {
    // Update pedal data for each hall sensor
    APPS1.UpdatePedalData();
    APPS2.UpdatePedalData();
    BSE.UpdatePedalData();
}

/*----------------------------------------------------------------------------- 
 Sample the SDC tap
-----------------------------------------------------------------------------*/
void systemData::UpdateSDCTapBuffer(void) {
    // Get the signal buffer
    circularBuffer buffer = pinSDCTap.GetBuffer();

    // Get the latest pin reading and add to buffer
    buffer.PushBuffer( pinSDCTap.ReadRawPinAnalog() );

    // Set the SDC tap buffer
    pinSDCTap.SetBuffer(buffer);
}

/*-----------------------------------------------------------------------------
 Check OOR & signal agreement - Returns true if error lasts for over 100 ms
-----------------------------------------------------------------------------*/
bool systemData::CheckPedalImplausibility(void) {
    static elapsedMillis pedalErrorTimer = 0;
    static bool bPedalError = false;
    bool bResult = false;

    // Check if APPS signals disagree or any signals are out of range
    if ( !CheckAPPS() || CheckPedalsOOR() ) {
        // Check the error duration
        if (!bPedalError) {
            // Start millisecond timer
            pedalErrorTimer = 0;
            bPedalError = true;
        } else if (pedalErrorTimer > IMPLAUSIBILITY_TIME) {
            // Return true when error(s) occurs for 100 ms 
            bResult = true;
            
            // Reset error check flag
            bPedalError = false;
        }
    } else {
        // Return false if either/both error(s) resolve in less than 100 ms
        bPedalError = false;
    }

    return bResult;
}

/*-----------------------------------------------------------------------------
 Check for driver RTD input
-----------------------------------------------------------------------------*/
bool systemData::ReadyToDrive(void) {
    // Get the brake percent request and RTD button reading
    float brakeRequest = BSE.GetPercentRequest() * 100;
    bool bRTDButtonPressed = pinRTDButton.ReadPulsedPin( pinRTDButton.ReadDebouncedPin() );

    // Check brake and RTD button are pressed
    return brakeRequest >= PERCENT_BRAKE && bRTDButtonPressed;
}

/*-----------------------------------------------------------------------------
 Activate motor controller
-----------------------------------------------------------------------------*/
void systemData::ActivateBamocar(void) {
	// Digital parameters required by the motor controller to drive
	pinRUN.WriteOutput(HIGH);
	pinRFE.WriteOutput(HIGH);
}

/*-----------------------------------------------------------------------------
 Deactivate motor controller
-----------------------------------------------------------------------------*/
void systemData::DeactivateBamocar(void) {
	// Digital parameters required by the motor controller to drive
	pinRUN.WriteOutput(LOW);
	pinRFE.WriteOutput(LOW);
}

/*----------------------------------------------------------------------------- 
 Use the previous functions to process incoming APPS data
-----------------------------------------------------------------------------*/
void systemData::ProcessAPPS(uint8_t * pTorqueBuf) {
    // Fetch and average analog data
    uint16_t torqueAPPS1 = APPS1.GetTorqueRequest();
    uint16_t torqueAPPS2 = APPS2.GetTorqueRequest();

    // Obtain the lower signal
    uint16_t signal = (torqueAPPS1 < torqueAPPS2) ? torqueAPPS1 : torqueAPPS2;

    // Set the data buffer equal to the processed signal
    if ( CheckAPPS() ) {
        pTorqueBuf[2] = signal & BYTE_ONE;
        pTorqueBuf[1] = (signal & BYTE_TWO) >> 8;
    }
}

/*----------------------------------------------------------------------------- 
 Get the lower percent request
-----------------------------------------------------------------------------*/
float systemData::GetLowerPercentAPPS(void) {
    // Get the two APPS percent requests
    float requestAPPS1 = APPS1.GetPercentRequest();
    float requestAPPS2 = APPS2.GetPercentRequest();

    // Return the lower percent request
    return (requestAPPS1 < requestAPPS2) ? requestAPPS1 : requestAPPS2;
}

/*-----------------------------------------------------------------------------
 Compare accelerator pedal positions - Returns true if signals agree
-----------------------------------------------------------------------------*/
bool systemData::CheckAPPS(void) {
    // Get the two APPS percent requests
    float requestAPPS1 = APPS1.GetPercentRequest();
    float requestAPPS2 = APPS2.GetPercentRequest();

    // Return the lower percent request
    return abs(requestAPPS1 - requestAPPS2) <= APPS_AGREEMENT;
}

/*-----------------------------------------------------------------------------
 Check for pedals OOR - Returns true if any signals are OOR
-----------------------------------------------------------------------------*/
bool systemData::CheckPedalsOOR(void) {
    // Check if pedals are out of range
	return APPS1.CheckPedalOOR() || APPS2.CheckPedalOOR() || BSE.CheckPedalOOR();
}

/*-----------------------------------------------------------------------------
 APPS BSE error check - Returns true if APPS & BSE are pressed
-----------------------------------------------------------------------------*/
bool systemData::CheckPedalPlausibility(void) {
    // Get the APPS and BSE percent requests
    float requestAPPS = GetLowerPercentAPPS() * 100;
    float requestBSE = BSE.GetPercentRequest() * 100;
    
    // Check if both pedals are pressed
    return requestAPPS > PERCENT_ACCEL && requestBSE > PERCENT_BRAKE;
}

/*-----------------------------------------------------------------------------
 Check All Errors - Returns true if any errors occur
-----------------------------------------------------------------------------*/
bool systemData::CheckAllErrors(void) {
    bool bResult = false;
    uint8_t errors = IRQHandler::GetErrorBuffer();

    // Check if the shutdown circuit opened
    if ( pinSDCTap.GetBuffer().GetAverage() < SDC_TAP_HIGH ) {
        // Set the SDC error bit high
        IRQHandler::SetErrorBuffer( errors | (1 << ERROR_CODE_SHUTDOWN) );
        bResult = true;

        DebugPrintln("ERROR: SHUTDOWN CIRCUIT OPENED");
    }

    // Check if pedal sensors disagree
    if ( timers.b100msPassed && !CheckAPPS() ) {
        // Set the APPS disagrement error bit high
        IRQHandler::SetErrorBuffer( errors | (1 << ERROR_CODE_DISAGREE) );
        bResult = true;

        DebugPrintln("ERROR: APPS DISAGREE");
    }

    // Check if accelerator and brake are both pressed
    if ( CheckPedalPlausibility() ) {
        // Set the APPS & BSE disagrement error bit high
        IRQHandler::SetErrorBuffer( errors | (1 << ERROR_CODE_APPS_BSE) );
        bResult = true;

        DebugPrintln("ERROR: APPS & BSE PRESSED");
    }

    // Check if any of the three sensors are out of range
    if ( timers.b100msPassed && CheckPedalsOOR() ) {
        // Set the pedals out of range disagrement error bit high
        IRQHandler::SetErrorBuffer( errors | (1 << ERROR_CODE_OOR) );
        bResult = true;

        DebugPrintln("ERROR: SENSOR(S) OUT OF RANGE");
    }

    return bResult;
}

/*-----------------------------------------------------------------------------
 Extract encoded values from SD file to set bounds for percent requests
-----------------------------------------------------------------------------*/
bool systemData::SetPedalBounds(void) {
    bool bSuccessfulLoad = false;
    size_t bufferLength;

    // Obtain the pedal sensors
    hall * sensors[NUM_SENSORS] = {&APPS1, &APPS2, &BSE};

    // Open file in SD card
    File fPedalBounds = SD.open(FILE_PEDAL_BOUNDS, FILE_READ);

    // Check file has opened
    if (fPedalBounds) {
        // Read sensor data and convert to array of integers
        String pedalBoundString = fPedalBounds.readString();
        const char * strPedalBounds = pedalBoundString.c_str();
        uint16_t * pPedalBounds = SplitIntegerString(strPedalBounds, DELIMITER, bufferLength);

        // Check number of values read matches number of bounds for sensors
        if (pPedalBounds && bufferLength == 2 * NUM_SENSORS) {
            // Iterate through each set of bounds per sensor
            for (uint8_t index = 0; index < NUM_SENSORS; ++index) {
                // Determine if the sensor voltage increases with actuation
                bool bVoltageInverted = sensors[index]->GetVoltageInverted();

                // Use direct or swapped bounds based on inversion
                uint16_t upper = bVoltageInverted ? pPedalBounds[index + NUM_SENSORS] : pPedalBounds[index];
                uint16_t lower = bVoltageInverted ? pPedalBounds[index] : pPedalBounds[index + NUM_SENSORS];

                // Apply tolerance to lower bound
                lower = static_cast<uint16_t>(lower * 0.97);

                // Set the bounds
                sensors[index]->SetPercentRequestUpperBound(upper);
                sensors[index]->SetPercentRequestLowerBound(lower);
            }

            bSuccessfulLoad = true;

            DebugPrintln("PEDAL BOUNDS SET");
        }

        // Free buffer from memory
        if (pPedalBounds) {
            free(pPedalBounds);
            pPedalBounds = NULL;
        }

        // Close the file
        fPedalBounds.close();
    }

    return bSuccessfulLoad;
}

/*-----------------------------------------------------------------------------
 Slowly ramp up or ramp down the duty cycle for the pump when in use
-----------------------------------------------------------------------------*/
void systemData::RampPump(bool direction) {
    // Get the time the function first executes
    static uint32_t rampTime = millis();

    // Check if 100 milliseconds has passed
    if ( millis() - rampTime >= 100 ) {
        // Reset timer
        rampTime = millis();

        // Increase or decrease duty cycle based on direction
        if (direction) {
            // Increment duty cycle by 5%
            pump.SetPWMDutyCycle( pump.GetPWMDutyCycle() + 5 );
        } else if ( pump.GetPWMDutyCycle() ) {
            // Decrement duty cycle by 5%
            pump.SetPWMDutyCycle( pump.GetPWMDutyCycle() - 5 );
        }
    }

    // Cap the duty cycle at 100%
    if ( pump.GetPWMDutyCycle() > 100 ) {
        pump.SetPWMDutyCycle(100);
    }
}

/*-----------------------------------------------------------------------------
 Turn pump on using precharge circuitry
-----------------------------------------------------------------------------*/
void systemData::RunPump(void) {
    static uint32_t prechargeTimer = millis();
    static bool bPumpActivated = false;

    // Check if pump has not been activated and motor temperature is high
    if ( !bPumpActivated && IRQHandler::GetMotorTemperature() > 40 ) {
        // Initially turn on pin driving pump through precharge resistor
        pinPump.WriteOutput(HIGH);

        // Check if 500 ms has passed
        if ( millis() - prechargeTimer >= 500 ) {
            // Switch the relay on to allow full current to pump
            pinPumpSwitch.WriteOutput(HIGH);

            // Turn off initial precharge pathway
            pinPump.WriteOutput(LOW);

            // Mark pump as activated
            bPumpActivated = true;
        }
    } else if ( bPumpActivated && IRQHandler::GetMotorTemperature() < 20 ) {
        // Turn off the pump
        pinPump.WriteOutput(LOW);
        pinPumpSwitch.WriteOutput(LOW);

        // Mark pump as decativated
        bPumpActivated = false;
    }
}

/*-----------------------------------------------------------------------------
 Output what errors occurred during a fault condition
-----------------------------------------------------------------------------*/
void systemData::DebugPrintErrors(void) {
    // Set the fault buffer from interrupt error buffer
    faultBuf = IRQHandler::GetErrorBuffer();

    // Print which errors occurred (binary format) (OOR, BPP, DIS, SDC)
    DebugPrint("VEHICLE ERRORS: ");
    DebugPrint( bitRead(faultBuf, ERROR_CODE_OOR) );
    DebugPrint( bitRead(faultBuf, ERROR_CODE_APPS_BSE) );
    DebugPrint( bitRead(faultBuf, ERROR_CODE_DISAGREE) );
    DebugPrintln( bitRead(faultBuf, ERROR_CODE_SHUTDOWN) );
}
