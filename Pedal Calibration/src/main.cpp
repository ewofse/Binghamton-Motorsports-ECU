#include "main.h"

// Hall sensor objects
hall APPS1(PIN_APPS_ONE);
hall APPS2(PIN_APPS_TWO);
hall BSE(PIN_BSE);

String strPedalBounds = "";

// FSM current state
calibrate_t CAL_state = PERCENT_REQ_LOWER;

// Extern variables declared here but not used
volatile uint8_t errorBuf;
volatile bool bShutdownCircuitOpen;

/*-------------------------------------------------------------------------------------------------
 Setup
-------------------------------------------------------------------------------------------------*/
void setup() {
	// Setup I/O pins
    SetPinModes();

    // Connect serial comms
    DebugBegin(SERIAL_RATE);
	DebugPrintln("SERIAL COMMS INITIALIZED");

    // Setup SD memory storage
    if ( !SD.begin(BUILTIN_SDCARD) ) {
		DebugErrorPrint("ERROR STARTING SD MEMORY");
        EXIT
    }

	DebugPrintln("SD MEMORY INITIALIZED");
}

/*-------------------------------------------------------------------------------------------------
 Main Loop
-------------------------------------------------------------------------------------------------*/
void loop() {
	// Update sensor readings until button is pressed
	while ( !ButtonPulser( ButtonDebouncer(PIN_RTD_BUTTON) ) ) {
		UpdatePedalStructures(&APPS1, &APPS2, &BSE);
	}

	switch (CAL_state) {
		uint16_t boundAPPS1, boundAPPS2, boundBSE;

		/*-----------------------------------------------------------------------------
		 Set the Lower Bounds for Pedal Percent Requests
		-----------------------------------------------------------------------------*/
		case (PERCENT_REQ_LOWER):
			// Add tolerance for lower bound
			boundAPPS1 = (uint16_t) APPS1.GetCookedOutput() * 0.99;
			boundAPPS2 = (uint16_t) APPS2.GetCookedOutput() * 0.99;
			boundBSE = (uint16_t) BSE.GetCookedOutput() * 0.99;

			// Set current cooked output as lower bound for all sensors
			APPS1.SetPercentRequestLowerBound(boundAPPS1);
			APPS2.SetPercentRequestLowerBound(boundAPPS2);
			BSE.SetPercentRequestLowerBound(boundBSE);

			strPedalBounds += String(boundAPPS1) + DELIMITER;
			strPedalBounds += String(boundAPPS2) + DELIMITER;
			strPedalBounds += String(boundBSE) + DELIMITER;

			CAL_state = PERECENT_REQ_UPPER;

			DebugPrintln("LOWER BOUND CALIBRATED");

			break;

		/*-----------------------------------------------------------------------------
		 Set the Upper Bounds for Pedal Percent Requests
		-----------------------------------------------------------------------------*/
		case (PERECENT_REQ_UPPER):
			// Add tolerance for lower bound
			boundAPPS1 = (uint16_t) APPS1.GetCookedOutput() * 1.01;
			boundAPPS2 = (uint16_t) APPS2.GetCookedOutput() * 1.01;
			boundBSE = (uint16_t) BSE.GetCookedOutput() * 1.01;

			// Set current cooked output as lower bound for all sensors
			APPS1.SetPercentRequestUpperBound(boundAPPS1);
			APPS2.SetPercentRequestUpperBound(boundAPPS2);
			BSE.SetPercentRequestUpperBound(boundBSE);

			strPedalBounds += String(boundAPPS1) + DELIMITER;
			strPedalBounds += String(boundAPPS2) + DELIMITER;
			strPedalBounds += String(boundBSE) + DELIMITER;

			CAL_state = DONE;

			DebugPrintln("UPPER BOUND CALIBRATED");

		/*-----------------------------------------------------------------------------
		 Output the bounds before ending program
		-----------------------------------------------------------------------------*/
		case (DONE): {
			// File variables
			uint16_t * pPedalBounds;
			File fPedalBounds;

			// Update SD file with new data
			WriteDataToFile(FILE_PEDAL_BOUNDS, strPedalBounds, OVERWRITE);

			DebugPrintln("CALIBRATION DONE");

			// Convert string of sensor values into array of integers
			pPedalBounds = SplitIntegerString(strPedalBounds, DELIMITER);

			DebugPrint("BOUNDS: ");

			// APPS1, APPS2, BSE (lower, upper) value format
			for (int i = 0; i < NUM_SENSORS; ++i) {
				DebugPrint("(");
				DebugPrint(pPedalBounds[i]);
				DebugPrint(", ");
				DebugPrint(pPedalBounds[i + 3]);
				DebugPrint(")");
				DebugPrint(" ");
			}

			free(pPedalBounds);

			// At this point, turn off the controller
            EXIT
		}
	}

	// Sample at a slow rate
	delay(500);
}
