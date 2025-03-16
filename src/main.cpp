/*-------------------------------------------------------------------------------------------------
 ECU Master Program
 Programmers: Markus Higgins, Vansh Joishar, Jake Lin, Ethan Wofse
 Last Updated: 03.07.25
-------------------------------------------------------------------------------------------------*/
#include "ECU.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;
WDT_T4<WDT1> WDT;

// Hall sensor objects
hall APPS1(PIN_APPS_ONE);
hall APPS2(PIN_APPS_TWO);
hall BSE(PIN_BSE);

// GPIO Pins
digitalPin pinRTDButton(PIN_RTD_BUTTON, BUTTON_DEBOUNCE_TIME, INPUT);
digitalPin pinSDCTap(PIN_SHUTDOWN_TAP, SHUTDOWN_STABLE_TIME, INPUT);

digitalPin pinRUN(PIN_RUN, OUTPUT);
digitalPin pinGO(PIN_GO, OUTPUT);
digitalPin pinRTDSound(PIN_RTD_SOUND, OUTPUT);
digitalPin pinBrakeLight(PIN_BRAKE_LIGHT, OUTPUT);
digitalPin pinReset(PIN_RESET, OUTPUT);

// EV1 additional Teensy pins
PIN_AIR_PLUS_INSTANTIATION;

// EV1.5 additional Teensy pins
PIN_PUMP_INSTANTIATION;
PIN_REGEN_INSTANTIATION;
PIN_CHARGE_ENABLE_INSTANTIATION;
PIN_LED_FAULT_INSTANTIATION;

// Timers and flags
timers_t timers;

// FSM current state
state_t FSM_State;

// Interrupt variables
volatile uint8_t errorBuf = 0;
volatile bool bShutdownCircuitOpen = false;
volatile bool bPedalCalibrationMode = false;

// TODO: Use an external timer module on Teensy and interrupts to sample pedal sensors instead of polling
// TODO: extern variables are bad!!! Fix please...
IntervalTimer samplingTimer;

/*-------------------------------------------------------------------------------------------------
 Setup
-------------------------------------------------------------------------------------------------*/
void setup() {
    // Connect serial comms for debugging
    DebugBegin(SERIAL_RATE);
    DebugPrintln("SERIAL COMMS INITIALIZED");

    // Setup SD memory storage for pedal calibration & telemetry
    if ( !SD.begin(BUILTIN_SDCARD) ) {
		DebugErrorPrint("ERROR: SD CARD FAILED");
        EXIT;
    }
    
    DebugPrintln("SD MEMORY INITIALIZED");

	// Initialize CAN communications
    ConfigureCANBus();
	DebugPrintln("CAN BUS INITIALIZED");

    // Setup WDT for potential software hangs
    ConfigureWDT();
	DebugPrintln("WATCHDOG CONFIGURED");

    // Setup data read requests to Bamocar
    RequestBamocarData();
    DebugPrintln("REQUESTING BAMOCAR DATA...");

	// Intialize timer boolean variables
	timers.bResetTimerStarted  = false;
	timers.bChargeTimerStarted = false;
	timers.bBuzzerActive       = false;

	// Set RTD button pin to be interrupt controlled for pedal calibration
	attachInterrupt(digitalPinToInterrupt( pinRTDButton.GetPin() ), PedalCalibrationISR, CHANGE);

    FSM_State = PEDALS;
}

/*-------------------------------------------------------------------------------------------------
 Main Loop
-------------------------------------------------------------------------------------------------*/
void loop() {
	uint8_t stateBuf = 0;
	uint8_t faultBuf = 0;

    // Feed the WDT
    FeedWDT();

    // Driving logic FSM
    switch (FSM_State) {
		/*-----------------------------------------------------------------------------
         Pedal Calibration State
        -----------------------------------------------------------------------------*/
		case (CALIBRATE):
            DebugPrintln("STATE: CALIBRATE");

			// Disable RTD button interrupt during pedal calibration
			detachInterrupt( pinRTDButton.GetPin() );

			// Begin calibration
			CalibratePedals(&APPS1, &APPS2, &BSE, pinRTDButton);

			bPedalCalibrationMode = false;

			// Set RTD button pin to be interrupt controlled for pedal calibration
			attachInterrupt(digitalPinToInterrupt( pinRTDButton.GetPin() ), PedalCalibrationISR, CHANGE);

			// Transition back to PEDALS to read in new encoded values
			FSM_State = PEDALS;
			break;

        /*-----------------------------------------------------------------------------
         Pedal Configuration State
        -----------------------------------------------------------------------------*/
        case (PEDALS): {
            // Check for a successful load of pedal bounds
            if ( !SetPedalBounds(&APPS1, &APPS2, &BSE) ) {
                DebugErrorPrint("ERROR: PEDAL BOUNDS NOT SET");
                EXIT;
            }

            FSM_State = INIT;
            break;
        }

        /*-----------------------------------------------------------------------------
         INITIALIZATION State
        -----------------------------------------------------------------------------*/
        case (INIT):
            DebugPrintln("STATE: INIT");

            // Reset shutdown circuit after RC has charged
            if (timers.bResetTimerStarted) {
                if (timers.resetTimer >= RESET_TIME) {
                    // Activate reset
                    pinReset.WriteOutput(HIGH);

                    DebugPrintln("RESET HIGH");

                    FSM_State = PRECHARGE;
                }
            } else {
                // Start reset timer
                timers.bResetTimerStarted = true;
                timers.resetTimer = 0;
            }

            break;

        /*-----------------------------------------------------------------------------
         PRECHARGE State
        -----------------------------------------------------------------------------*/
        case (PRECHARGE): {
            DebugPrintln("STATE: PRECHARGE");

			stateBuf = 0;

            // Precharge tractive system
            if ( pinSDCTap.ReadDebouncedPin() ) {
                DebugPrintln("SHUTDOWN HIGH");
                
				// Wait for one second before continuing to RTD
                if (timers.bChargeTimerStarted) {
                    if (timers.chargeTimer >= CHARGE_TIME) {
						// Set AIR plus pin high
                        EV1_AIR_PLUS_HIGH();

						// Set SDC error to be interrupt controlled
						attachInterrupt(digitalPinToInterrupt( pinSDCTap.GetPin() ), ShutdownCircuitISR, FALLING);

						// Remove pedal calibration mode control
						detachInterrupt( pinRTDButton.GetPin() );

                        timers.bChargeTimerStarted = false;

                        DebugPrintln("SYSTEM CHARGED");

                        // Transition to RTD when vehicle is powered
                        FSM_State = WAIT_FOR_RTD;
                    }
                } else {
					// Start precharge timer
                    timers.chargeTimer = 0;
                    timers.bChargeTimerStarted = true;
                }
			} else if ( DetectCalibrationStartup() ) {
				// Begin pedal calibration when startup detected
				FSM_State = CALIBRATE;
				break;
			}

            break;
        }

        /*-----------------------------------------------------------------------------
         WAIT FOR RTD State
        -----------------------------------------------------------------------------*/
        case (WAIT_FOR_RTD):
            DebugPrintln("STATE: RTD");

            stateBuf = 1;

			// Transition to FAULT if any possible error occurs
			if ( CheckAllErrors(&APPS1, &APPS2, &BSE, timers.b100msPassed) || bShutdownCircuitOpen ) {
				FSM_State = FAULT;
				return;
			}

            // Await driver input to activate vehicle
            if ( ReadyToDrive(&BSE, pinRTDButton) ) {
                // Set motor controller enable signals high
                ActivateBamocar(pinRUN, pinGO);

                // Activate buzzer pin
                pinRTDSound.WriteOutput(HIGH);

                // Begin timer for buzzer
                timers.buzzerTimer = 0;
                timers.bBuzzerActive = true;
            }

            // Play buzzer for 1 second and transition to IDLE
            if (timers.bBuzzerActive && timers.buzzerTimer >= BUZZER_TIME) {
                pinRTDSound.WriteOutput(LOW);
                timers.bBuzzerActive = false;

                FSM_State = IDLE;
            }
            
            break;

        /*-----------------------------------------------------------------------------
         IDLE State
        -----------------------------------------------------------------------------*/
        case (IDLE): {
            CAN_message_t msgTorque;
            uint8_t torqueBuf[PAR_RX_DLC] = {0, 0, 0};

            DebugPrintln("STATE: IDLE");

			stateBuf = 2;

            // Transition to FAULT if any possible error occurs
            if ( CheckAllErrors(&APPS1, &APPS2, &BSE, timers.b100msPassed) || bShutdownCircuitOpen ) {
                FSM_State = FAULT;
                return;
            }
            
            // Switch to DRIVE or BRAKE state based on pedal readings
            if ( GetLowerPercentAPPS(&APPS1, &APPS2) * 100 > PERCENT_THRESHOLD ) {
                FSM_State = DRIVE;
            } else if ( BSE.GetPercentRequest() * 100 > PERCENT_BRAKE ) {
                FSM_State = BRAKE;
            } else {
                torqueBuf[1] = 0x00;
                torqueBuf[2] = 0x00;
                
                // Send torque command of zero to Bamocar
                PopulateCANMessage(&msgTorque, ID_CAN_MESSAGE_RX, PAR_RX_DLC, torqueBuf, REG_DIG_TORQUE_SET);
                SendCANMessage(msgTorque);
            }
            
            break;
        }

        /*-----------------------------------------------------------------------------
         DRIVE State
        -----------------------------------------------------------------------------*/
        case (DRIVE): {
            CAN_message_t msgTorque;
            uint8_t torqueBuf[PAR_RX_DLC] = {0, 0, 0};

            DebugPrintln("STATE: DRIVE");

			stateBuf = 3;

            // Transition to FAULT if any possible error occurs
            if ( CheckAllErrors(&APPS1, &APPS2, &BSE, timers.b100msPassed) || bShutdownCircuitOpen ) {
                FSM_State = FAULT;
                return;
            }

            // Transition to IDLE state if APPS receive minimal force (braking)
            if ( GetLowerPercentAPPS(&APPS1, &APPS2) * 100 < PERCENT_THRESHOLD ) {
                FSM_State = IDLE;
            } else {
                // Update CAN message buffer with updated pedal readings
                ProcessAPPS(&APPS1, &APPS2, torqueBuf);

                // Send torque command to Bamocar
                PopulateCANMessage(&msgTorque, ID_CAN_MESSAGE_RX, PAR_RX_DLC, torqueBuf, REG_DIG_TORQUE_SET);
                SendCANMessage(msgTorque);
            }
            
            break;
        }

        /*-----------------------------------------------------------------------------
         BRAKE State
        -----------------------------------------------------------------------------*/
        case (BRAKE): {
            CAN_message_t msgTorque;
            uint8_t torqueBuf[PAR_RX_DLC] = {0, 0, 0};;

            DebugPrintln("STATE: BRAKE");

			stateBuf = 4;

            // Transition to FAULT if any possible error occurs
            if ( CheckAllErrors(&APPS1, &APPS2, &BSE, timers.b100msPassed) || bShutdownCircuitOpen ) {
                FSM_State = FAULT;
                return;
            }

            // Enter BRAKE or IDLE based on brake reading
            if ( BSE.GetPercentRequest() * 100 < PERCENT_BRAKE ) {
                FSM_State = IDLE;
            } else {
                torqueBuf[1] = 0x00;
                torqueBuf[2] = 0x00;

                // Send torque command of zero to Bamocar
                PopulateCANMessage(&msgTorque, ID_CAN_MESSAGE_RX, PAR_RX_DLC, torqueBuf, REG_DIG_TORQUE_SET);
                SendCANMessage(msgTorque);
            }
            
            break;
        }

        /*-----------------------------------------------------------------------------
         FAULT State
        -----------------------------------------------------------------------------*/
        case (FAULT): {
            static bool bErrorsWritten = false;

            DebugPrintln("STATE: FAULT");

			stateBuf = 5;
            
			// Print which errors occurred (binary format) (OOR, BPP, DIS, SDC)
			DebugPrint("VEHICLE ERRORS: ");
			DebugPrint( bitRead(errorBuf, ERROR_CODE_OOR) );
			DebugPrint( bitRead(errorBuf, ERROR_CODE_APPS_BSE) );
			DebugPrint( bitRead(errorBuf, ERROR_CODE_DISAGREE) );
			DebugPrintln( bitRead(errorBuf, ERROR_CODE_SHUTDOWN) );

            DeactivateBamocar(pinRUN, pinGO);

			// Toggle fault LED on EV1.5 ECU PCB
			EV1_5_ACTIVATE_FAULT_LED();
            
            // Revert to another state when pedal signals are not implausible
            if ( !PedalsDisagree() && !PedalsOOR() ) {
                // APPS / Brake Pedal Plausability Check resolved
                if ( BothPedalsPressed() && GetLowerPercentAPPS(&APPS1, &APPS2) * 100 < PLAUSIBILITY_CHECK ) {
                    // Re-activate motor controller enable signals
                    ActivateBamocar(pinRUN, pinGO);
                    
                    // Clear both pedals pressed error bit and revert to IDLE
                    errorBuf &= ~(1 << ERROR_CODE_APPS_BSE);
                    bErrorsWritten = false;
                    FSM_State = IDLE;

                    break;
                }

                // Check shutdown tap closes again
                if ( ShutdownCircuitOpen() && pinSDCTap.ReadDebouncedPin() ) {
                    // Clear both shutdown open error bit and revert to RTD
                    errorBuf &= ~(1 << ERROR_CODE_SHUTDOWN);
                    bShutdownCircuitOpen = false;
                    bErrorsWritten = false;
                    FSM_State = WAIT_FOR_RTD;

                    break;
                }
            }
            
            // Write ECU errors to a data file on the SD card
            if (!bErrorsWritten) {
                ErrorToSD();
                
                // Set flag high to only write once
                bErrorsWritten = true;
            }
            
            break;
        }
            
        /*-----------------------------------------------------------------------------
         DEFAULT
        -----------------------------------------------------------------------------*/
        default:
            DebugPrintln("ERROR: Unknown FSM State");
            FSM_State = FAULT;
            break;
    }

    /*-----------------------------------------------------------------------------
     Pedal Processing
    -----------------------------------------------------------------------------*/
    UpdatePedalStructures(&APPS1, &APPS2, &BSE);

    /*-----------------------------------------------------------------------------
     Brake Light Updates
    -----------------------------------------------------------------------------*/
    ActivateBrakeLight(&BSE, pinBrakeLight);

    /*-----------------------------------------------------------------------------
     Pedal Implausibility (Timer Check)
    -----------------------------------------------------------------------------*/
    timers.b100msPassed = CheckPedalImplausibility(&APPS1, &APPS2, &BSE);

	/*-----------------------------------------------------------------------------
     Telemetry & Status CAN Messages
    -----------------------------------------------------------------------------*/
    faultBuf = errorBuf;
    SendCANStatusMessages(&faultBuf, &stateBuf);
}	
