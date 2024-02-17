#include "CANRead.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

CAN_message_t msgRX;
CAN_message_t msgTX;

void setup() 
{
    // Initialize Teensy LED to low
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);

    // Initializing CAN
    Can1.begin();
    Can1.setBaudRate(500000);

    // Serial communication
    Serial.begin(9600);

    delay(500);
}

void loop()
{
    // Send a CAN message
    SendMessage(msgRX);

    // Read the CAN message
    ReadMessage(msgTX, pin);

    delay(100);
}