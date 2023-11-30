#include "ReadyToDrive.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

CAN_message_t TransmittingSpeedCommandValue;

bool isAlreadyReady = false;

int accelPedal;
int brakePedal;

int total = 0;
int average = 0;  
int counter = 0;
int array[ARRAY_SIZE];

void setup()
{
    // Initializing CAN
    Can1.begin();
    Can1.setBaudRate(500000);

    // Serial communication
    Serial.begin(115200);

    // pinMode(brakePin, OUTPUT);

    InitializeArray(array, ARRAY_SIZE);
}

void loop()
{
    // Check ready to drive state
    isAlreadyReady = ReadyToDrive(&isAlreadyReady, buttonPin, brakePin);

    if(isAlreadyReady)
    {
        Serial.println("This should not be printing");
        Serial.println();
    }
}
