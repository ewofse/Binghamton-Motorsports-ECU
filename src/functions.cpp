#include "main.h"

/*-----------------------------------------------------------------------------
 Set pin modes
-----------------------------------------------------------------------------*/
void SetPinModes() {
    pinMode(PIN_BRAKE,           INPUT);
    pinMode(PIN_ACCELERATOR_ONE, INPUT);
    pinMode(PIN_ACCELERATOR_TWO, INPUT);
    pinMode(PIN_RTD_BUTTON,      INPUT);
    pinMode(PIN_SHUTDOWN_TAP,    INPUT);

    pinMode(PIN_RUN,             OUTPUT);
    pinMode(PIN_GO,              OUTPUT);
    pinMode(PIN_RTD_SOUND,       OUTPUT);
    pinMode(PIN_BRAKE_LIGHT,     OUTPUT);
    pinMode(PIN_AIR_PLUS,        OUTPUT);   
}

/*-----------------------------------------------------------------------------
 Check for driver RTD input
-----------------------------------------------------------------------------*/
bool ReadyToDrive(bool * bReady, hall * pBSE) {
    // Redundancy
    if (*bReady) {
        return true;
    }

    // Brake and RTD Button are pressed
    if ( ( pBSE->GetPercentRequest(hallBSE) * 100 ) >= PERCENT_BRAKE ) {
        if ( digitalRead(PIN_RTD_BUTTON) ) {
            *bReady = true;

            return *bReady;
        }
    }

    return false;
}

/*-----------------------------------------------------------------------------
 Check and activate brake light
-----------------------------------------------------------------------------*/
void ActivateBrakeLight(hall * pBSE) {
    if ( ( pBSE->GetPercentRequest(hallBSE) * 100 ) > PERCENT_BRAKE ) {
        digitalWrite(PIN_BRAKE_LIGHT, HIGH);
    } else {
        digitalWrite(PIN_BRAKE_LIGHT, LOW);
    }
}

/*-----------------------------------------------------------------------------
 Activate motor controller
-----------------------------------------------------------------------------*/
void ActivateBamocar() {
    // Digital parameters required by the motor controller to drive
    digitalWrite(PIN_RUN, HIGH);
    digitalWrite(PIN_GO, HIGH);
}

/*-----------------------------------------------------------------------------
 Deactivate motor controller
-----------------------------------------------------------------------------*/
void DeactivateBamocar() {
    // Digital parameters required by the motor controller to drive
    digitalWrite(PIN_RUN, LOW);
    digitalWrite(PIN_GO, LOW);
}

/*-----------------------------------------------------------------------------
 Discharge high voltage system
-----------------------------------------------------------------------------*/
void DischargeHighVoltage() {
    // Shut the discharge relay for 1.5s
    digitalWrite(PIN_DISCHARGE, LOW);
    delay(1500);
    digitalWrite(PIN_DISCHARGE, HIGH);
}

/*-----------------------------------------------------------------------------
 Populate CAN message frame
-----------------------------------------------------------------------------*/
void PopulateCANMessage(CAN_message_t * pMessage, uint8_t DLC, uint8_t bamocarDestReg, int8_t messageBuf[], uint16_t ID) {
    int index = 0;

    // Head of Message
    pMessage->flags.extended  =  PAR_EXTENDED;
    pMessage->flags.remote    =  PAR_REMOTE;
    pMessage->flags.overrun   =  PAR_OVERRUN;
    pMessage->flags.reserved  =  PAR_RESERVED;
    pMessage->id              =  ID;
    pMessage->len             =  DLC;

    // Bamocar read register request
    if (ID == ID_CAN_MESSAGE_TX) {
        pMessage-> buf[0] = REG_READ;
        pMessage-> buf[1] = bamocarDestReg;

        index = 2;
    }

    // Bamocar set register request
    if (ID == ID_CAN_MESSAGE_RX) {
        pMessage->buf[0] = bamocarDestReg;

        index = 1;
    }

    // Bamocar wants Little Endian Format (LIFO)
    for (int i = index; i < DLC; ++i) {
        pMessage->buf[i] = messageBuf[DLC - i];
    }
}

/*-----------------------------------------------------------------------------
 Populate CAN message frame (Raspberry Pi Specific)
-----------------------------------------------------------------------------*/
void PopulatePiCANMessage(CAN_message_t * pMessage, uint8_t DLC, int8_t * messageBuf, uint16_t ID) {
    // Head of Message
    pMessage->flags.extended  =  PAR_EXTENDED;
    pMessage->flags.remote    =  PAR_REMOTE;
    pMessage->flags.overrun   =  PAR_OVERRUN;
    pMessage->flags.reserved  =  PAR_RESERVED;
    pMessage->id              =  ID;
    pMessage->len             =  DLC;

    // Populate data buffer for Bamocar data
    if (DLC == PAR_RECEIVE_DLC) {
        for (int i = 1; i < DLC; ++i) {
            pMessage->buf[i - 1] = messageBuf[i];
        }
    } else {
        for (int i = 0; i < DLC; ++i) {
            pMessage->buf[i] = messageBuf[i];
        }
    }
}

/*-----------------------------------------------------------------------------
 Send CAN Message
-----------------------------------------------------------------------------*/
void SendCANMessage(CAN_message_t * pMessage) {
    myCan.write(*pMessage);
}

/*-----------------------------------------------------------------------------
 Read CAN message
-----------------------------------------------------------------------------*/
void ReadCANMessage(CAN_message_t * pMessage) {
    myCan.read(*pMessage);
 
    Serial.print("  ID: 0x"); Serial.print((*pMessage).id, HEX);
    Serial.print("  LEN: "); Serial.print((*pMessage).len);
    Serial.print(" DATA: ");

    for (int i = 0; i < pMessage->len; ++i) {
        Serial.print(pMessage->buf[i], HEX);
        Serial.print(" ");
    }

    Serial.print("  TS: "); Serial.println((*pMessage).timestamp);
    Serial.println();
}

/*-----------------------------------------------------------------------------
 Print serial comms information for a short amount of time
-----------------------------------------------------------------------------*/
void PrintDebugMessage(const char message[]) {
    for (int i = 0; i < 10000; ++i) {
        Serial.println(message);
    }
}