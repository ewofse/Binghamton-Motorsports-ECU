#include "core/FSM.h"

/*-----------------------------------------------------------------------------
 Calibrate pedal sensor encoded values
-----------------------------------------------------------------------------*/
void systemData::CalibratePedals() {
	uint16_t boundAPPS1 = 0;
	uint16_t boundAPPS2 = 0;
	uint16_t boundBSE = 0;

	calibrate_t FSM_State = UPDATE_PEDALS;
	uint8_t stateCounter = 0;
	bool bDone = false;
	char strPedalData[50] = "";
	
	// Blocking operation - Nothing on the car should be active during calibration
	while (!bDone) {
		// Feed Watchdog
		IRQHandler::FeedWDT();

		// Pedal Calibration FSM
		switch (FSM_State) {
			/*-----------------------------------------------------------------------------
			 Update Pedal Sensor Readings
			-----------------------------------------------------------------------------*/
			case (UPDATE_PEDALS):
				UpdatePedalStructures();

				// Await driver RTD button input to set bounds
				if ( pinRTDButton.ReadPulsedPin( pinRTDButton.ReadDebouncedPin() ) ) {
					// Increment state counter
					++stateCounter;

					// Move to next state based on how many times RTD button is pressed
					if (stateCounter == 1) {
						FSM_State = PERCENT_REQ_UPPER;
					} else if (stateCounter == 2) {
						FSM_State = PERCENT_REQ_LOWER;
					}
				}

				break;

			/*-----------------------------------------------------------------------------
			 Set the Upper Bounds for Pedal Percent Requests
			-----------------------------------------------------------------------------*/
			case (PERCENT_REQ_UPPER): {
				char strUpperBounds[25] = "";

				// Set upper bounds
				boundAPPS1 = APPS1.GetCookedOutput();
				boundAPPS2 = APPS2.GetCookedOutput();
				boundBSE = BSE.GetCookedOutput();

				// Set current cooked output as upper bound for all sensors
				APPS1.SetPercentRequestUpperBound(boundAPPS1);
				APPS2.SetPercentRequestUpperBound(boundAPPS2);
				BSE.SetPercentRequestUpperBound(boundBSE);

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

				// Set upper bounds
				boundAPPS1 = APPS1.GetCookedOutput();
				boundAPPS2 = APPS2.GetCookedOutput();
				boundBSE = BSE.GetCookedOutput();

				// Set current cooked output as upper bound for all sensors
				APPS1.SetPercentRequestLowerBound(boundAPPS1);
				APPS2.SetPercentRequestLowerBound(boundAPPS2);
				BSE.SetPercentRequestLowerBound(boundBSE);

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
