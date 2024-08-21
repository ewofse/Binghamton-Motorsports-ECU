#include "main.h"

// CAN network
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;

// CAN message objects
CAN_message_t torque, temp, current, RPM, voltage;
CAN_message_t customError, currentState;

// Timer variables
uint32_t chargeTimer;
uint32_t buzzerTimer;
uint32_t resetTimer;

bool bTimerStarted;
bool bCharged;
bool bReady;
bool bPedalError;
bool b100msPassed;

elapsedMillis pedalErrorTimer;

// CAN message buffers
int8_t torqueBuf[PAR_SEND_DLC];
int8_t tempBuf[PAR_RECEIVE_DLC]; 
int8_t currentBuf[PAR_RECEIVE_DLC];
int8_t RPMBuf[PAR_RECEIVE_DLC];
int8_t voltageBuf[PAR_RECEIVE_DLC];

int8_t errorBuf;
int8_t stateBuf;
int8_t tempCode;

// Hall sensor objects
hall APPS1(PIN_ACCELERATOR_ONE);
hall APPS2(PIN_ACCELERATOR_TWO);
hall BSE(PIN_BRAKE);

state_t FSM_State;

/*-------------------------------------------------------------------------------------------------
 Setup
-------------------------------------------------------------------------------------------------*/
void setup() {
    // Configure pins, serial comms, and FSM state
    FSM_State = INIT;

    Serial.begin(SERIAL_RATE);
    digitalWrite(PIN_RTD_SOUND, LOW);
    SetPinModes();
}

/*-------------------------------------------------------------------------------------------------
 Main Loop
-------------------------------------------------------------------------------------------------*/
void loop() {
    switch (FSM_State) {
        /*-----------------------------------------------------------------------------
         Initialization State
        -----------------------------------------------------------------------------*/
        case (INIT):
            PrintDebugMessage("IN INIT STATE");

            /*-------------------------------------
             Reset Shutdown Circuit
            -------------------------------------*/
            digitalWrite(PIN_RESET, HIGH);
            resetTimer = millis() + RESET;

            /*-------------------------------------
             Initialize Function Scope Variables
            -------------------------------------*/
            bReady        = false;
            bCharged      = false;
            bTimerStarted = false;
            bPedalError   = false;

            /*-------------------------------------
             Initialize CAN Communications
            -------------------------------------*/
            myCan.begin();
            myCan.setBaudRate(BAUD_RATE);

            /*-------------------------------------
             Next State
            -------------------------------------*/
            FSM_State = PRECHARGE;
        break;

        /*-----------------------------------------------------------------------------
         Precharge State
        -----------------------------------------------------------------------------*/
        case (PRECHARGE):
            Serial.println("STATE: PRECHARGE");

            if (millis() >= resetTimer) {
                digitalWrite(PIN_RESET, LOW);
            }

            // Precharge tractive system
            if ( digitalRead(PIN_SHUTDOWN_TAP) ) {
                Serial.println("SHUTDOWN HIGH");
                
                if (bTimerStarted) {
                    Serial.println("B TIMER STARTED");
                    Serial.print("MILLIS: "); Serial.println( millis() );
                    Serial.print("MICROS: "); Serial.println( micros() );
                    Serial.print("CHARGE TIMER: "); Serial.println(chargeTimer);

                    if ( millis() >= chargeTimer ) {
                        Serial.println("B TIMER > CHARGE TIME");

                        digitalWrite(PIN_AIR_PLUS, HIGH);
                        bCharged = true;

                        FSM_State = WAIT_FOR_RTD;
                    }
                } else {
                    Serial.print("CHARGE TIMER INIT: "); Serial.println(chargeTimer);

                    chargeTimer = millis() + DELAY;

                    Serial.print("CHARGE TIMER CHANGED: "); Serial.println(chargeTimer);
                    Serial.println("B TIMER NOT STARTED");

                    bTimerStarted = true;
                }
            }
        break;

        /*-----------------------------------------------------------------------------
         Wait For RTD State
        -----------------------------------------------------------------------------*/
        case (WAIT_FOR_RTD):
            Serial.println("STATE: RTD");

            // Transition to FAULT if any possible error occurs
            if ( CheckAllErrors(&APPS1, &APPS2, &BSE, &errorBuf, b100msPassed, &tempCode) ) {
                PrintDebugMessage("IN FAULT STATE");

                FSM_State = FAULT;
                break;
            }

            // Await driver input and activate vehicle
            if ( !bReady && ReadyToDrive(&bReady, &BSE) ) {
                ActivateBamocar();
                buzzerTimer = millis() + 1000;

                PrintDebugMessage("BUZZER");
            }

            // Transition to IDLE when vehicle is RTD
            if ( bReady && ReadyToDrive(&bReady, &BSE) ) {
                digitalWrite(PIN_RTD_SOUND, HIGH);

                if (millis() >= buzzerTimer) {
                    digitalWrite(PIN_RTD_SOUND, LOW);

                    PrintDebugMessage("SOUND & IDLE");

                    FSM_State = IDLE;
                }
            }
        break;

        /*-----------------------------------------------------------------------------
         Idle State
        -----------------------------------------------------------------------------*/
        case (IDLE):
            Serial.println("STATE: IDLE");

            // stateBuf = 0;

            // // Send CAN message to Pi regarding vehicle state
            // PopulatePiCANMessage(&currentState, PAR_STATE_DLC, &stateBuf, ID_CURRENT_STATE);
            // SendCANMessage(&currentState);

            // Transition to FAULT if any possible error occurs
            if ( CheckAllErrors(&APPS1, &APPS2, &BSE, &errorBuf, b100msPassed, &tempCode) ) {
                FSM_State = FAULT;
                break;
            }
            
            // Choose DRIVE or BRAKE state based on pedal readings
            if ( (GetLowerPercentAPPS(&APPS1, &APPS2) * 100) > PERCENT_THRESHOLD ) {
                FSM_State = DRIVE;
            } else if ( (BSE.GetPercentRequest(hallBSE) * 100) > PERCENT_BRAKE ) {
                FSM_State = BRAKE;
            } else {
                torqueBuf[1] = 0x00;
                torqueBuf[2] = 0x00;
                
                // Send torque command of zero to Bamocar
                PopulateCANMessage(&torque, PAR_SEND_DLC, REG_DIG_TORQUE_SET, torqueBuf, ID_CAN_MESSAGE_RX);
                SendCANMessage(&torque);

                FSM_State = IDLE;
            }
        break;  

        /*-----------------------------------------------------------------------------
         Drive State
        -----------------------------------------------------------------------------*/
        case (DRIVE):
            Serial.println("STATE: DRIVE");

            // stateBuf = 1;

            // // Send CAN message to Pi regarding vehicle state
            // PopulatePiCANMessage(&currentState, PAR_STATE_DLC, &stateBuf, ID_CURRENT_STATE);
            // SendCANMessage(&currentState);

            // Transition to FAULT if any possible error occurs
            if ( CheckAllErrors(&APPS1, &APPS2, &BSE, &errorBuf, b100msPassed, &tempCode) ) {
                FSM_State = FAULT;
                break;
            }

            // Transition to IDLE state if APPS receive minimal force (braking)
            if ( (GetLowerPercentAPPS(&APPS1, &APPS2) * 100) < PERCENT_THRESHOLD ) {
                torqueBuf[1] = 0x00;
                torqueBuf[2] = 0x00;

                // Send torque command of zero to Bamocar
                PopulateCANMessage(&torque, PAR_SEND_DLC, REG_DIG_TORQUE_SET, torqueBuf, ID_CAN_MESSAGE_RX);
                SendCANMessage(&torque);

                FSM_State = IDLE;
            } else {
                // Update CAN message buffer with updated pedal readings
                ProcessAPPS(&APPS1, &APPS2, torqueBuf);

                // Send torque command to Bamocar
                PopulateCANMessage(&torque, PAR_SEND_DLC, REG_DIG_TORQUE_SET, torqueBuf, ID_CAN_MESSAGE_RX);
                SendCANMessage(&torque);

                FSM_State = DRIVE;
            }
        break;

        /*-----------------------------------------------------------------------------
         Brake State
        -----------------------------------------------------------------------------*/
        case (BRAKE):
            Serial.println("STATE: BRAKE");

            // Transition to FAULT if any possible error occurs
            if ( CheckAllErrors(&APPS1, &APPS2, &BSE, &errorBuf, b100msPassed, &tempCode) ) {
                FSM_State = FAULT;
                break;
            }

            torqueBuf[1] = 0x00;
            torqueBuf[2] = 0x00;

            // Send torque command of zero to Bamocar
            PopulateCANMessage(&torque, PAR_SEND_DLC, REG_DIG_TORQUE_SET, torqueBuf, ID_CAN_MESSAGE_RX);
            SendCANMessage(&torque);

            // Enter BRAKE or IDLE based on brake reading
            if ( (BSE.GetPercentRequest(hallBSE) * 100) >= PERCENT_BRAKE ) {
                FSM_State = BRAKE;
            } else if ( (BSE.GetPercentRequest(hallBSE) * 100) < PERCENT_BRAKE ) {
                FSM_State = IDLE;
            }
        break;

        /*-----------------------------------------------------------------------------
         Fault State
        -----------------------------------------------------------------------------*/
        case (FAULT):
            // At this point, the vehicle must be turned off and rekeyed
            Serial.println("STATE: FAULT");
            Serial.println(tempCode);

            DeactivateBamocar();

            torqueBuf[1] = 0x00;
            torqueBuf[2] = 0x00;

            // Send torque command of zero to Bamocar - Redundancy
            PopulateCANMessage(&torque, PAR_SEND_DLC, REG_DIG_TORQUE_SET, torqueBuf, ID_CAN_MESSAGE_RX);
            SendCANMessage(&torque);

            // stateBuf = 2;

            // // Send current state to dashboard
            // PopulatePiCANMessage(&currentState, PAR_STATE_DLC, &stateBuf, ID_CURRENT_STATE);
            // SendCANMessage(&currentState);

            bReady        = false;
            bCharged      = false;
            bTimerStarted = false;

            // APPS & BSE error - If resolved, revert to PRECHARGE state
            if ( errorBuf & (1 << 2) ) {
                if ( ( GetLowerPercentAPPS(&APPS1, &APPS2) * 100 ) < BSPD_CHECK ) {
                    FSM_State = PRECHARGE;
                }
            }

            // Shutdown tap error - If tap closes, rever to PRECHARGE state
            if ( errorBuf & (1 << 0) ) {
                if ( digitalRead(PIN_SHUTDOWN_TAP) ) {
                    FSM_State = PRECHARGE;
                }
            }

        break;

        /*-----------------------------------------------------------------------------
         Default
        -----------------------------------------------------------------------------*/
        default:
            Serial.println("DEFAULT");
        break;
    }

    /*-------------------------------------
     Process Pedals
    -------------------------------------*/
    UpdatePedalStructures(&APPS1, &APPS2, &BSE); 

    /*-------------------------------------
     Brake Light
    -------------------------------------*/
    ActivateBrakeLight(&BSE);

    // /*-------------------------------------
    //  BSE & APPS timer check
    // -------------------------------------*/
    // if ( !CheckAPPS(&APPS1, &APPS2) || CheckPedalsOOR(&APPS1, &APPS2, &BSE) ) {
    //     if (!bPedalError) {
    //         pedalErrorTimer = 0;
    //         bPedalError = true;
    //     }

    //     if (pedalErrorTimer > 1000 && bPedalError) {
    //         b100msPassed = true;
    //         bPedalError = false;
    //     }
    // }

    // if ( CheckAPPS(&APPS1, &APPS2) && !CheckPedalsOOR(&APPS1, &APPS2, &BSE) ) {
    //     bPedalError = false;
    //     b100msPassed = false;
    // }

    // /*-------------------------------------
    // Custom error code protocol
    // -------------------------------------*/
    // PopulatePiCANMessage(&customError, PAR_ERROR_DLC, &errorBuf, ID_ERROR_CODE);
    // SendCANMessage(&customError);

    // /*-------------------------------------
    // Send vital data to dashboard
    // -------------------------------------*/
    // PopulateCANMessage(&temp, PAR_RECEIVE_DLC, REG_MOTOR_TEMP, tempBuf, ID_CAN_MESSAGE_TX);
    // SendCANMessage(&temp);
    // PopulatePiCANMessage(&temp, PAR_SEND_DLC, tempBuf, ID_MOTOR_TEMP);
    // SendCANMessage(&temp);

    // PopulateCANMessage(&RPM, PAR_RECEIVE_DLC, REG_RPM, RPMBuf, ID_CAN_MESSAGE_TX);
    // SendCANMessage(&RPM);
    // PopulatePiCANMessage(&RPM, PAR_SEND_DLC, RPMBuf, ID_RPM);
    // SendCANMessage(&RPM);

    // PopulateCANMessage(&current, PAR_RECEIVE_DLC, REG_CURRENT, currentBuf, ID_CAN_MESSAGE_TX);
    // SendCANMessage(&current);
    // PopulatePiCANMessage(&current, PAR_SEND_DLC, tempBuf, ID_CURRENT);
    // SendCANMessage(&current);

    // PopulateCANMessage(&voltage, PAR_RECEIVE_DLC, REG_VOLTAGE, voltageBuf, ID_CAN_MESSAGE_TX);
    // SendCANMessage(&voltage);
    // PopulatePiCANMessage(&voltage, PAR_SEND_DLC, voltageBuf, ID_VOLTAGE);
    // SendCANMessage(&voltage);
}