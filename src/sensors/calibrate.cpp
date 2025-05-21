#include "core/FSM.h"

/*-----------------------------------------------------------------------------
 Calibrate pedal sensor encoded values
-----------------------------------------------------------------------------*/
void systemData::CalibratePedals() {
	uint16_t boundAPPS1 = 0;
	uint16_t boundAPPS2 = 0;
	uint16_t boundBSE = 0;

	pedalCalibrate state = pedalCalibrate::UPDATE_PEDALS;
	uint8_t buttonCounter = 0;
	bool bDone = false;
	char strPedalData[50] = "";

	// Turn off brake light
	pinBrakeLight.WriteOutput(LOW);
	
	// Blocking operation - Nothing on the car should be active during calibration
	while (!bDone) {
		// Feed Watchdog
		IRQHandler::FeedWDT();

		// Pedal Calibration FSM
		switch (state) {
			/*-----------------------------------------------------------------------------
			 Update Pedal Sensor Readings
			-----------------------------------------------------------------------------*/
			case (pedalCalibrate::UPDATE_PEDALS):
				// Continuously update pedal readings until button pressed
				UpdatePedalStructures();

				// Await driver RTD button input to set bounds
				if ( pinRTDButton.ReadPulsedPin( pinRTDButton.ReadDebouncedPin() ) ) {
					// Increment state counter
					++buttonCounter;

					// Move to next state based on how many times RTD button is pressed
					if (buttonCounter == 1) {
						state = pedalCalibrate::PERCENT_REQ_UPPER;
					} else if (buttonCounter == 2) {
						state = pedalCalibrate::PERCENT_REQ_LOWER;
					}
				}

				break;

			/*-----------------------------------------------------------------------------
			 Set the Upper Bounds for Pedal Percent Requests
			-----------------------------------------------------------------------------*/
			case (pedalCalibrate::PERCENT_REQ_UPPER): {
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
				snprintf(strUpperBounds, 25, "%d,%d,%d,", boundAPPS1, boundAPPS2, boundBSE);
				strncat(strPedalData, strUpperBounds, 50 - strlen(strPedalData) - 1);

				state = pedalCalibrate::UPDATE_PEDALS;

				DebugPrintln("UPPER BOUND CALIBRATED");

				break;
			}

			/*-----------------------------------------------------------------------------
			 Set the Lower Bounds for Pedal Percent Requests
			-----------------------------------------------------------------------------*/
			case (pedalCalibrate::PERCENT_REQ_LOWER): {
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
				snprintf(strLowerBounds, 25, "%d,%d,%d", boundAPPS1, boundAPPS2, boundBSE);
				strncat(strPedalData, strLowerBounds, 50 - strlen(strPedalData) - 1);

				state = pedalCalibrate::DONE;

				DebugPrintln("LOWER BOUND CALIBRATED");
			}

			/*-----------------------------------------------------------------------------
			 Output the bounds before ending calibration
			-----------------------------------------------------------------------------*/
			case (pedalCalibrate::DONE): {
				// Update SD file with new data
				WriteDataToFile(FILE_PEDAL_BOUNDS, strPedalData, OVERWRITE);

				// Calibration complete
				bDone = true;

				DebugPrintln("CALIBRATION COMPLETE");
			}

			/*-----------------------------------------------------------------------------
			 Uknown State
			-----------------------------------------------------------------------------*/
			default:
				// Drive system to done state
				state = pedalCalibrate::DONE;
				break;
		}
	}

	// Add a small delay to prevent double button press
	delay(100);
}

/*-----------------------------------------------------------------------------
 Initiate Bamocar calibration sequence
-----------------------------------------------------------------------------*/
void systemData::CalibrateMotor() {
	bool bDone = false;
	uint8_t buttonCounter = 0;

	// Blocking operation - Nothing on the car should be active during calibration
	while (!bDone) {
		// Feed Watchdog
		IRQHandler::FeedWDT();

		// Wait until RTD button is pressed
		if ( pinRTDButton.ReadPulsedPin( pinRTDButton.ReadDebouncedPin() ) ) {
			// Increment button counter
			++buttonCounter;

			// Evaluate button count value and transition to next state
			if (buttonCounter == 1) {
				// Enable the RFE signal
				pinRFE.WriteOutput(HIGH);
				DebugPrintln("RFE HIGH");
			} else if (buttonCounter == 2) {
				// Enable the run signal
				pinRUN.WriteOutput(HIGH);
				DebugPrintln("RUN HIGH");
			} else if (buttonCounter == 3) {
				// Turn off the run signal
				pinRUN.WriteOutput(LOW);
				DebugPrintln("RUN LOW");
			} else if (buttonCounter == 4) {
				// Turn off the RFE signal
				pinRFE.WriteOutput(LOW);
			
				// Exit calibration
				bDone = true;

				DebugPrintln("CALIBRATION COMPLETE");
			}
		}
	}

	// Add a small delay to prevent double button press
	delay(100);
}
