/*-------------------------------------------------------------------------------------------------
 ECU Pedal Calibration Program
 Programmer: Ethan Wofse
 Last Updated: 03.06.25
-------------------------------------------------------------------------------------------------*/
#include "ECU.h"

/*-----------------------------------------------------------------------------
 Detect if driver initiates calibration mode startup sequence
-----------------------------------------------------------------------------*/
bool DetectCalibrationStartup() {
	static uint32_t timer = millis();
	bool bResult = false;

	// Detect if RTD button has been held for three seconds
	if ( bPedalCalibrationMode && ( millis() - timer >= PEDAL_CALIBRATION_TIME ) ) {
		// Reset timer
		timer = millis();

		bResult = true;
	}	

	return bResult;
}

/*-----------------------------------------------------------------------------
 Calibrate pedal sensor encoded values
-----------------------------------------------------------------------------*/
void CalibratePedals(hall * pAPPS1, hall * pAPPS2, hall * pBSE, digitalPin pin) {
	uint16_t boundAPPS1 = 0;
	uint16_t boundAPPS2 = 0;
	uint16_t boundBSE = 0;

	calibrate_t FSM_State = UPDATE_PEDALS;
	bool bDone = false;
	char strPedalData[50] = "";
	
	// Blocking operation - Nothing on the car should be active during calibration
	while (!bDone) {
		// Feed Watchdog
		FeedWDT();

		// Pedal Calibration FSM
		switch (FSM_State) {
			/*-----------------------------------------------------------------------------
			 Update Pedal Sensor Readings
			-----------------------------------------------------------------------------*/
			case (UPDATE_PEDALS):
				UpdatePedalStructures(pAPPS1, pAPPS2, pBSE);

				// Await driver RTD button input to set bounds
				if ( pin.ReadDebouncedPin() ) {
					FSM_State = PERCENT_REQ_UPPER;
				}

				break;

			/*-----------------------------------------------------------------------------
			 Set the Upper Bounds for Pedal Percent Requests
			-----------------------------------------------------------------------------*/
			case (PERCENT_REQ_UPPER): {
				char strUpperBounds[25] = "";

				// Set upper bounds
				boundAPPS1 = pAPPS1->GetCookedOutput();
				boundAPPS2 = pAPPS2->GetCookedOutput();
				boundBSE = pBSE->GetCookedOutput();

				// Set current cooked output as upper bound for all sensors
				pAPPS1->SetPercentRequestUpperBound(boundAPPS1);
				pAPPS2->SetPercentRequestUpperBound(boundAPPS2);
				pBSE->SetPercentRequestUpperBound(boundBSE);

				// Add pedal bound values to the string
				sprintf(strUpperBounds, "%d,%d,%d,", boundAPPS1, boundAPPS2, boundBSE);
				strcat(strPedalData, strUpperBounds);

				FSM_State = UPDATE_PEDALS;

				DebugPrintln("UPPER BOUND CALIBRATED");

				break;
			}

			/*-----------------------------------------------------------------------------
			 Set the Lower Bounds for Pedal Percent Requests
			-----------------------------------------------------------------------------*/
			case (PERCENT_REQ_LOWER): {
				char strLowerBounds[25] = "";

				// Set lower bounds
				boundAPPS1 = pAPPS1->GetCookedOutput();
				boundAPPS2 = pAPPS2->GetCookedOutput();
				boundBSE = pBSE->GetCookedOutput();

				// Set current cooked output as lower bound for all sensors
				pAPPS1->SetPercentRequestLowerBound(boundAPPS1);
				pAPPS2->SetPercentRequestLowerBound(boundAPPS2);
				pBSE->SetPercentRequestLowerBound(boundBSE);

				// Add pedal bound values to the string
				sprintf(strLowerBounds, "%d,%d,%d", boundAPPS1, boundAPPS2, boundBSE);
				strcat(strPedalData, strLowerBounds);

				FSM_State = DONE;

				DebugPrintln("LOWER BOUND CALIBRATED");
			}

			/*-----------------------------------------------------------------------------
			 Output the bounds before ending calibration
			-----------------------------------------------------------------------------*/
			case (DONE): {
				// Update SD file with new data
				WriteDataToFile(FILE_PEDAL_BOUNDS, strPedalData, OVERWRITE);

				// Calibration complete
				bDone = true;

				DebugPrintln("CALIBRATION COMPLETE");
			}
		}
	}
}
