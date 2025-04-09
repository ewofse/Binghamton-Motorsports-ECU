#include "core/FSM.h"

/*-----------------------------------------------------------------------------
 Activate brake light when BSE is pressed
-----------------------------------------------------------------------------*/
void systemData::ActivateBrakeLight() {
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
void systemData::UpdatePedalStructures() {
    // Update pedal data for each hall sensor
    APPS1.UpdatePedalData();
    APPS2.UpdatePedalData();
    BSE.UpdatePedalData();
}

/*-----------------------------------------------------------------------------
 Check OOR & signal agreement - Returns true if error lasts for over 100 ms
-----------------------------------------------------------------------------*/
bool systemData::CheckPedalImplausibility() {
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
bool systemData::ReadyToDrive() {
    // Get the brake percent request and RTD button reading
    float brakeRequest = BSE.GetPercentRequest() * 100;
    bool bRTDButtonPressed = pinRTDButton.ReadPulsedPin( pinRTDButton.ReadDebouncedPin() );

    // Check brake and RTD button are pressed
    return brakeRequest >= PERCENT_BRAKE && bRTDButtonPressed;
}

/*-----------------------------------------------------------------------------
 Activate motor controller
-----------------------------------------------------------------------------*/
void systemData::ActivateBamocar() {
	// Digital parameters required by the motor controller to drive
	pinRUN.WriteOutput(HIGH);
	pinGO.WriteOutput(HIGH);
}

/*-----------------------------------------------------------------------------
 Deactivate motor controller
-----------------------------------------------------------------------------*/
void systemData::DeactivateBamocar() {
	// Digital parameters required by the motor controller to drive
	pinRUN.WriteOutput(LOW);
	pinGO.WriteOutput(LOW);
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
float systemData::GetLowerPercentAPPS() {
    // Get the two APPS percent requests
    float requestAPPS1 = APPS1.GetPercentRequest();
    float requestAPPS2 = APPS2.GetPercentRequest();

    // Return the lower percent request
    return (requestAPPS1 < requestAPPS2) ? requestAPPS1 : requestAPPS2;
}

/*-----------------------------------------------------------------------------
 Compare accelerator pedal positions - Returns true if signals agree
-----------------------------------------------------------------------------*/
bool systemData::CheckAPPS() {
    // Get the two APPS percent requests
    float requestAPPS1 = APPS1.GetPercentRequest();
    float requestAPPS2 = APPS2.GetPercentRequest();

    // Return the lower percent request
    return abs(requestAPPS1 - requestAPPS2) <= APPS_AGREEMENT;
}

/*-----------------------------------------------------------------------------
 Check for pedals OOR - Returns true if any signals are OOR
-----------------------------------------------------------------------------*/
bool systemData::CheckPedalsOOR() {
    // Check if pedals are out of range
	return APPS1.CheckPedalOOR() || APPS2.CheckPedalOOR() || BSE.CheckPedalOOR();
}

/*-----------------------------------------------------------------------------
 APPS BSE error check - Returns true if APPS & BSE are pressed
-----------------------------------------------------------------------------*/
bool systemData::CheckPedalPlausibility() {
    // Get the APPS and BSE percent requests
    float requestAPPS = GetLowerPercentAPPS() * 100;
    float requestBSE = BSE.GetPercentRequest() * 100;
    
    // Check if both pedals are pressed
    return requestAPPS > PERCENT_ACCEL && requestBSE > PERCENT_BRAKE;
}

/*-----------------------------------------------------------------------------
 Check All Errors - Returns true if any errors occur
-----------------------------------------------------------------------------*/
bool systemData::CheckAllErrors() {
    bool bResult = false;
    uint8_t errors = IRQHandler::GetErrorBuffer();

    // Check if the shutdown circuit opened
    if ( IRQHandler::GetShutdownState() ) {
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
bool systemData::SetPedalBounds() {
    bool bSuccessfulLoad = false;
    size_t bufferLength;

    // Open file in SD card
    File fPedalBounds = SD.open(FILE_PEDAL_BOUNDS, FILE_READ);

    // Check file has opened
    if (fPedalBounds) {
        // Read sensor data and convert to array of integers
        const char * strPedalBounds = fPedalBounds.readString().c_str();
        uint16_t * pPedalBounds = SplitIntegerString(strPedalBounds, DELIMITER, bufferLength);

        // Check number of values read matches number of bounds for sensors
        if (pPedalBounds && bufferLength == 2 * NUM_SENSORS) {
            // Set upper bounds for percent request
            APPS1.SetPercentRequestUpperBound( pPedalBounds[0] );
            APPS2.SetPercentRequestUpperBound( pPedalBounds[1] );
            BSE.SetPercentRequestUpperBound( pPedalBounds[2] );

            // Set lower bounds for percent request (add a tolerance)
            APPS1.SetPercentRequestLowerBound( (uint16_t) pPedalBounds[3] * 0.97 );
            APPS1.SetPercentRequestLowerBound( (uint16_t) pPedalBounds[4] * 0.97 );
            BSE.SetPercentRequestLowerBound( (uint16_t) pPedalBounds[5] * 0.97 );

            bSuccessfulLoad = true;

            DebugPrintln("PEDAL BOUNDS SET");
        }

        // Free buffer from memory
        free(pPedalBounds);
        pPedalBounds = NULL;

        // Close the file
        fPedalBounds.close();
    }

    return bSuccessfulLoad;
}
