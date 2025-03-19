#include "sensors/pedals.h"

/*-----------------------------------------------------------------------------
 Check for driver RTD input
-----------------------------------------------------------------------------*/
bool ReadyToDrive(hall * pBSE, digitalPin pinBSE) {
	float BSE = pBSE->GetPercentRequest() * 100;

	// Check brake and RTD Button are pressed
	return BSE >= PERCENT_BRAKE && pinBSE.ReadPulsedPin( pinBSE.ReadDebouncedPin() );
}

/*-----------------------------------------------------------------------------
 Activate brake light when BSE is pressed (20%)
-----------------------------------------------------------------------------*/
void ActivateBrakeLight(hall * pBSE, digitalPin pinBrakeLight) {
	// Check if brake is significantly pressed to activate brake light
	if ( pBSE->GetPercentRequest() * 100 > PERCENT_BRAKE ) {
		pinBrakeLight.WriteOutput(HIGH);
	} else {
		pinBrakeLight.WriteOutput(LOW);
	}
}

/*-----------------------------------------------------------------------------
 Average the signal
-----------------------------------------------------------------------------*/
void AverageSignal(hall * pAPPS) {
    // Modify signal buffer by adding newest reading
    pAPPS->buffer.PushBuffer( pAPPS->GetRawOutput() );

    // Calculate new average and up scale
    uint16_t average = pAPPS->buffer.GetTotal() / pAPPS->buffer.GetCount();
    uint16_t cookedSignal = (TWO_BYTES * average) / ADC_RESOLUTION;

    // Set hall sensor cooked output to new averaged signal
    pAPPS->SetCookedOutput(cookedSignal);
}

/*----------------------------------------------------------------------------- 
 Use the previous functions to process incoming APPS data                                                        
-----------------------------------------------------------------------------*/
void ProcessAPPS(hall * pAPPS1, hall * pAPPS2, uint8_t * pTorqueBuf) {
    // Fetch and average analog data
    uint16_t torqueAPPS1 = pAPPS1->GetTorqueRequest();
    uint16_t torqueAPPS2 = pAPPS2->GetTorqueRequest();

    // Obtain the lower signal
    uint16_t signal = (torqueAPPS1 < torqueAPPS2) ? torqueAPPS1: torqueAPPS2;

    // Set the data buffer equal to the processed signal
    if ( CheckAPPS(pAPPS1, pAPPS2) ) {
        pTorqueBuf[2] = signal & BYTE_ONE;
        pTorqueBuf[1] = (signal & BYTE_TWO) >> 8;
    }
}

/*----------------------------------------------------------------------------- 
 Update a hall sensor's data
-----------------------------------------------------------------------------*/
void UpdatePedalData(hall * pSensor) {
    // Read the raw signal from input analog pin
    pSensor->SetRawOutput( pSensor->ReadPedal() );

    // Average normalized signal
    AverageSignal(pSensor);

    // Calculate the pedal percent request
    uint16_t torque = pSensor->GetPercentRequest() * FIFTEEN_BITS;

	// Cap the torque request to maximum if over 100% pedal percent request
	if (torque > FIFTEEN_BITS) {
		torque = FIFTEEN_BITS;
	}

    // Set the torque percent request
    pSensor->SetTorqueRequest(torque);
}

/*----------------------------------------------------------------------------- 
 Update pedal data for all hall sensors
-----------------------------------------------------------------------------*/
void UpdatePedalStructures(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
    // Update pedal data for each hall sensor
    UpdatePedalData(pAPPS1);
    UpdatePedalData(pAPPS2);
    UpdatePedalData(pBSE);
}

/*----------------------------------------------------------------------------- 
 Get the lower percent request
-----------------------------------------------------------------------------*/
float GetLowerPercentAPPS(hall * pAPPS1, hall * pAPPS2) {
    float APPS1 = pAPPS1->GetPercentRequest();
    float APPS2 = pAPPS2->GetPercentRequest();

    // Return the lower percent request
    return (APPS1 < APPS2) ? APPS1 : APPS2;
}

/*-----------------------------------------------------------------------------
 Compare accelerator pedal positions - Returns true if signals agree
-----------------------------------------------------------------------------*/
bool CheckAPPS(hall * pAPPS1, hall * pAPPS2) {
    float APPS1 = pAPPS1->GetPercentRequest() * 100;
    float APPS2 = pAPPS2->GetPercentRequest() * 100;

    // Check if pedals disagree
    return abs(APPS1 - APPS2) <= APPS_AGREEMENT;
}

/*-----------------------------------------------------------------------------
 Check if a pedal is OOR - Returns true if it is OOR
-----------------------------------------------------------------------------*/
bool CheckPedalOOR(hall * pSensor) {
	uint16_t raw = pSensor->GetRawOutput();
	float request = pSensor->GetPercentRequest();

    // Check the raw analog pin reading has not been shorted/opened
	// Check the percent request did not go beneath -10% or above 110% due to slipping
    bool bRawValueOOR = raw < OOR_LOWER_BOUND || raw >= OOR_UPPER_BOUND;
    bool bPercentOOR = request <= OOR_LOWER_PERCENT || request >= OOR_UPPER_PERCENT;

	return bRawValueOOR || bPercentOOR;
}

/*-----------------------------------------------------------------------------
 Check for pedals OOR - Returns true if any signals are OOR
-----------------------------------------------------------------------------*/
bool CheckPedalsOOR(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
    // Check if pedals are out of range
	return CheckPedalOOR(pAPPS1) || CheckPedalOOR(pAPPS2) || CheckPedalOOR(pBSE);
}

/*-----------------------------------------------------------------------------
 APPS BSE error check - Returns true if APPS & BSE are pressed (25% & 15%)
-----------------------------------------------------------------------------*/
bool CheckPedalPlausibility(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
    float APPS = GetLowerPercentAPPS(pAPPS1, pAPPS2) * 100;
    float BSE  = pBSE->GetPercentRequest() * 100;
    
    // Check if both pedals are pressed
    return APPS > PERCENT_ACCEL && BSE > PERCENT_BRAKE;
}

/*-----------------------------------------------------------------------------
 Check OOR & signal agreement - Returns true if error lasts for over 100 ms
-----------------------------------------------------------------------------*/
bool CheckPedalImplausibility(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
    static elapsedMillis pedalErrorTimer = 0;
    static bool bPedalError = false;
    bool bResult = false;

    // Check if APPS signals disagree or any signals are out of range
    if ( !CheckAPPS(pAPPS1, pAPPS2) || CheckPedalsOOR(pAPPS1, pAPPS2, pBSE) ) {
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
 Check All Errors - Returns true if any errors occur
-----------------------------------------------------------------------------*/
bool CheckAllErrors(hall * pAPPS1, hall * pAPPS2, hall * pBSE, bool b100msPassed) {
    bool bResult = false;
    uint8_t errors = IRQHandler::GetErrorBuffer();

    // Check if pedal sensors disagree
    if ( b100msPassed && !CheckAPPS(pAPPS1, pAPPS2) ) {
        // Set the APPS disagrement error bit high
        IRQHandler::SetErrorBuffer( errors | (1 << ERROR_CODE_DISAGREE) );
        bResult = true;

        DebugPrintln("ERROR: APPS DISAGREE");
    }

    // Check if accelerator and brake are both pressed
    if ( CheckPedalPlausibility(pAPPS1, pAPPS2, pBSE) ) {
        // Set the APPS & BSE disagrement error bit high
        IRQHandler::SetErrorBuffer( errors | (1 << ERROR_CODE_APPS_BSE) );
        bResult = true;

        DebugPrintln("ERROR: APPS & BSE PRESSED");
    }

    // Check if any of the three sensors are out of range
    if ( b100msPassed && CheckPedalsOOR(pAPPS1, pAPPS2, pBSE) ) {
        // Set the pedals out of range disagrement error bit high
        IRQHandler::SetErrorBuffer( errors | (1 << ERROR_CODE_OOR) );
        bResult = true;

        DebugPrintln("ERROR: SENSOR(S) OUT OF RANGE");
    }

    return bResult;
}

/*-----------------------------------------------------------------------------
 Evalues bit zero of error buffer (Shutdown circuit error)
-----------------------------------------------------------------------------*/
bool ShutdownCircuitOpen() {
    // Check if shutdown circuit open
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_SHUTDOWN);
}

/*-----------------------------------------------------------------------------
 Evalues bit one of error buffer (APPS signal agreement error)
-----------------------------------------------------------------------------*/
bool PedalsDisagree() {
    // Check if pedals disagree
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_DISAGREE);
}

/*-----------------------------------------------------------------------------
 Evalues bit three of error buffer (Both pedals pressed error)
-----------------------------------------------------------------------------*/
bool BothPedalsPressed() {
    // Check if both pedals are pressed
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_APPS_BSE);
}

/*-----------------------------------------------------------------------------
 Evalues bit four of error buffer (Pedals out of range error)
-----------------------------------------------------------------------------*/
bool PedalsOOR() {
    // Check if pedals are out of range
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_OOR);
}

/*-----------------------------------------------------------------------------
 Extract encoded values from SD file to set bounds for percent requests
-----------------------------------------------------------------------------*/
bool SetPedalBounds(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
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
            pAPPS1->SetPercentRequestUpperBound( pPedalBounds[0] );
            pAPPS2->SetPercentRequestUpperBound( pPedalBounds[1] );
            pBSE->SetPercentRequestUpperBound( pPedalBounds[2] );

            // Set lower bounds for percent request (add a tolerance)
            pAPPS1->SetPercentRequestLowerBound( (uint16_t) pPedalBounds[3] * 0.97 );
            pAPPS2->SetPercentRequestLowerBound( (uint16_t) pPedalBounds[4] * 0.97 );
            pBSE->SetPercentRequestLowerBound( (uint16_t) pPedalBounds[5] * 0.97 );

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
