/*-------------------------------------------------------------------------------------------------
 ECU Master Program
 Programmers: Markus Higgins, Vansh Joishar, Jake Lin, Ethan Wofse
Last Updated: 01.29.25
-------------------------------------------------------------------------------------------------*/
#include "main.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;
WDT_T4<WDT1> WDT;

// Hall sensor objects
hall APPS1(PIN_APPS_ONE);
hall APPS2(PIN_APPS_TWO);
hall BSE(PIN_BSE);

// Timers and flags
timers_t timers;

// FSM current state
state_t FSM_State;

// Interrupt variables
volatile uint8_t errorBuf = 0;
volatile bool bShutdownCircuitOpen = false;

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
		DebugErrorPrint("ERROR: SD CARD FAILED");
        EXIT
    }
    
    DebugPrintln("SD MEMORY INITIALIZED");

    // Setup WDT
    ConfigureWDT();

	// Intialize timer boolean variables
	timers.bResetTimerStarted  = false;
	timers.bChargeTimerStarted = false;
	timers.bBuzzerActive       = false;

    FSM_State = PEDALS;
}

/*-------------------------------------------------------------------------------------------------
 Main Loop
-------------------------------------------------------------------------------------------------*/
void loop() {
    // CAN message objects and buffers
	CAN_message_t msgBamocarTempRX, msgBamocarCurrentRX, msgBamocarVoltageRX, msgBamocarSpeedRX;
	CAN_message_t msgBamocarPhaseRX;
    CAN_message_t msgVehicleFault, msgCurrentState;
    uint8_t stateBuf, faultBuf;

	// Array of status messages
	CAN_message_t messageBuf[NUM_MESSAGES_TX] = {
		msgVehicleFault,
		msgCurrentState,
		msgBamocarTempRX,
		msgBamocarSpeedRX,
		msgBamocarCurrentRX,
		msgBamocarVoltageRX
	};

	// Array of status message mailboxes
	FLEXCAN_MAILBOX mailboxBuf[NUM_MESSAGES_TX] = {
		MAILBOX_FAULT,
		MAILBOX_STATE,
		MAILBOX_TEMP,
		MAILBOX_SPEED,
		MAILBOX_CURRENT,
		MAILBOX_VOLTAGE
	};

	// myCan.events();

    // Feed the WDT
    FeedWDT();

    switch (FSM_State) {
        /*-----------------------------------------------------------------------------
         Pedal Configuration State
        -----------------------------------------------------------------------------*/
        case (PEDALS): {
            // Open text file in SD card
            File fPedalBounds = SD.open(FILE_PEDAL_BOUNDS, FILE_READ);

            // Check file has opened
            if (fPedalBounds) {
                // Read sensor data and convert to array of integers
                String strPedalBounds = fPedalBounds.readString();
                uint16_t * pPedalBounds = SplitIntegerString(strPedalBounds, DELIMITER);

                // Set lower bounds for percent request
                APPS1.SetPercentRequestLowerBound( pPedalBounds[0] );
                APPS2.SetPercentRequestLowerBound( pPedalBounds[1] );
                BSE.SetPercentRequestLowerBound( pPedalBounds[2] );

                // Set upper bounds for percent request
                APPS1.SetPercentRequestUpperBound( pPedalBounds[3] );
                APPS2.SetPercentRequestUpperBound( pPedalBounds[4] );
                BSE.SetPercentRequestUpperBound( pPedalBounds[5] );

                DebugPrintln("PEDAL BOUNDS SET");

                free(pPedalBounds);
                pPedalBounds = nullptr;
            } else {
                DebugErrorPrint("ERROR: OPENING HALL SENSOR FILE");
                EXIT
            }

            // Close the text file
            fPedalBounds.close();

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
                // Check 5 seconds has passed
                if (timers.resetTimer >= RESET_TIME) {
                    // Activate reset
                    digitalWrite(PIN_RESET, HIGH);

                    DebugPrintln("RESET HIGH");

                    // Initialize CAN communications
                    ConfigureCANBus();

                    FSM_State = PRECHARGE;
                }
            } else {
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
            if ( digitalRead(PIN_SHUTDOWN_TAP) ) {
                DebugPrintln("SHUTDOWN HIGH");
                
				// Wait for 500 ms before continuing to RTD
                if (timers.bChargeTimerStarted) {
                    DebugPrint("CHARGE TIMER: "); DebugPrintln(timers.chargeTimer);

                    if (timers.chargeTimer >= CHARGE_TIME) {
						// Set AIR plus pin high
                        EV1_AIR_PLUS_HIGH();

                        timers.bChargeTimerStarted = false;

                        DebugPrintln("SYSTEM CHARGED");

                        // Transition to RTD when vehicle is powered
                        FSM_State = WAIT_FOR_RTD;
                    }
                } else {
                    timers.chargeTimer = 0;
                    DebugPrint("CHARGE TIMER INIT: "); DebugPrintln(timers.chargeTimer);

                    timers.bChargeTimerStarted = true;
                }
			}

            break;
        }

        /*-----------------------------------------------------------------------------
         WAIT FOR RTD State
        -----------------------------------------------------------------------------*/
        case (WAIT_FOR_RTD):
            DebugPrintln("STATE: RTD");

            // Transition to FAULT if any possible error occurs
            if ( bShutdownCircuitOpen || CheckAllErrors(&APPS1, &APPS2, &BSE, timers.b100msPassed) ) {
                FSM_State = FAULT;
                return;
            }

            // Await driver input to activate vehicle
            if ( ReadyToDrive(&BSE) ) {
                ActivateBamocar();

                // Activate buzzer
                digitalWrite(PIN_RTD_SOUND, HIGH);
                DebugPrintln("BUZZER & SOUND");

                timers.buzzerTimer = 0;
                timers.bBuzzerActive = true;
            }

            // Play buzzer for 1 second
            if (timers.bBuzzerActive && timers.buzzerTimer >= BUZZER_TIME) {
                // Transition to IDLE when vehicle is RTD
                digitalWrite(PIN_RTD_SOUND, LOW);
                timers.bBuzzerActive = false;
                FSM_State = IDLE;
            }
            
            break;

        /*-----------------------------------------------------------------------------
         IDLE State
        -----------------------------------------------------------------------------*/
        case (IDLE): {
            CAN_message_t msgTorque;
            uint8_t torqueBuf[PAR_RX_DLC];

            DebugPrintln("STATE: IDLE");

			stateBuf = 1;

            // Transition to FAULT if any possible error occurs
            if ( bShutdownCircuitOpen || CheckAllErrors(&APPS1, &APPS2, &BSE, timers.b100msPassed) ) {
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
                SendCANMessage(msgTorque, MAILBOX_TORQUE);
            }
            
            break;
        }

        /*-----------------------------------------------------------------------------
         DRIVE State
        -----------------------------------------------------------------------------*/
        case (DRIVE): {
            CAN_message_t msgTorque;
            uint8_t torqueBuf[PAR_RX_DLC];

            DebugPrintln("STATE: DRIVE");

			stateBuf = 2;

            // Transition to FAULT if any possible error occurs
            if ( bShutdownCircuitOpen || CheckAllErrors(&APPS1, &APPS2, &BSE, timers.b100msPassed) ) {
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
                SendCANMessage(msgTorque, MAILBOX_TORQUE);
            }

            // Request 3-phase current data from bamocar
            PopulateCANMessage(&msgBamocarPhaseRX, ID_CAN_MESSAGE_RX, PAR_RX_DLC, REG_CURRENT_PHASE_1);
            SendCANMessage(msgBamocarPhaseRX, MAILBOX_PHASE_ONE);

            PopulateCANMessage(&msgBamocarPhaseRX, ID_CAN_MESSAGE_RX, PAR_RX_DLC, REG_CURRENT_PHASE_2);
            SendCANMessage(msgBamocarPhaseRX, MAILBOX_PHASE_TWO);

            PopulateCANMessage(&msgBamocarPhaseRX, ID_CAN_MESSAGE_RX, PAR_RX_DLC, REG_CURRENT_PHASE_3);
            SendCANMessage(msgBamocarPhaseRX, MAILBOX_PHASE_THREE);
            
            break;
        }

        /*-----------------------------------------------------------------------------
         BRAKE State
        -----------------------------------------------------------------------------*/
        case (BRAKE): {
            CAN_message_t msgTorque;
            uint8_t torqueBuf[PAR_RX_DLC];

            DebugPrintln("STATE: BRAKE");

			stateBuf = 3;

            // Transition to FAULT if any possible error occurs
            if ( bShutdownCircuitOpen || CheckAllErrors(&APPS1, &APPS2, &BSE, timers.b100msPassed) ) {
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
                SendCANMessage(msgTorque, MAILBOX_TORQUE);
            }
            
            break;
        }

        /*-----------------------------------------------------------------------------
         FAULT State
        -----------------------------------------------------------------------------*/
        case (FAULT): {
            DebugPrintln("STATE: FAULT");

			stateBuf = 4;
            
			// Print which errors occurred (binary format)
			DebugPrint("VEHICLE ERRORS: ");
			DebugPrint( bitRead(errorBuf, ERROR_CODE_OOR) );
			DebugPrint( bitRead(errorBuf, ERROR_CODE_APPS_BSE) );
			DebugPrint( bitRead(errorBuf, ERROR_CODE_DISAGREE) );
			DebugPrintln( bitRead(errorBuf, ERROR_CODE_SHUTDOWN) );

            DeactivateBamocar();

			// Toggle fault LED on EV1.5 ECU PCB
			EV1_5_ACTIVATE_FAULT_LED();
            
            // Revert to another state when pedal signals are not implausible
            if ( !PedalsDisagree() && !PedalsOOR() ) {
                // APPS / Brake Pedal Plausability check resolved
                if ( BothPedalsPressed() && GetLowerPercentAPPS(&APPS1, &APPS2) * 100 < BSPD_CHECK ) {
                    // Re-activate motor controller enable signals
                    ActivateBamocar();
                    
                    // Clear both pedals pressed error bit and revert to IDLE
                    errorBuf &= ~(1 << 2);
                    FSM_State = IDLE;
                    break;
                }
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
    ActivateBrakeLight(&BSE);

    /*-----------------------------------------------------------------------------
     Pedal Implausibility (Timer Check)
    -----------------------------------------------------------------------------*/
    timers.b100msPassed = CheckPedalImplausibility(&APPS1, &APPS2, &BSE, &timers.pedalErrorTimer);

	/*-----------------------------------------------------------------------------
     Telemetry & Status CAN Messages
    -----------------------------------------------------------------------------*/
    faultBuf = errorBuf;

	// Populate all messages
	PopulateCANMessage(&msgVehicleFault, ID_ERROR_CODE, PAR_ERROR_DLC, &faultBuf);
	PopulateCANMessage(&msgCurrentState, ID_CURRENT_STATE, PAR_STATE_DLC, &stateBuf);
	PopulateCANMessage(&msgBamocarTempRX, ID_CAN_MESSAGE_RX, PAR_RX_DLC, REG_TEMP);
	PopulateCANMessage(&msgBamocarSpeedRX, ID_CAN_MESSAGE_RX, PAR_RX_DLC, REG_CURRENT);
	PopulateCANMessage(&msgBamocarCurrentRX, ID_CAN_MESSAGE_RX, PAR_RX_DLC, REG_VOLTAGE);
	PopulateCANMessage(&msgBamocarVoltageRX, ID_CAN_MESSAGE_RX, PAR_RX_DLC, REG_SPEED_ACTUAL);

	// Sequentially send each message every 10ms
	SendCANMessagePeriodic(messageBuf, mailboxBuf);
}
