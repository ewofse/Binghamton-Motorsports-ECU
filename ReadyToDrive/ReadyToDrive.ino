#include "ReadyToDrive.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

CAN_message_t TransmittingSpeedCommandValue;

int accelPedal1023;
int accelPedal65535;

int brakePedal1023;
int brakePedal65535;

bool isAlreadyReady = false;

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
    Serial.begin(9600);

    // Set array elements to 0
    InitializeArray(array, ARRAY_SIZE);
}

void loop()
{
    // Check ready to drive state
    isAlreadyReady = ReadyToDrive(&isAlreadyReady, buttonPin, brakePin);

    // Send commands when RTD
    if(isAlreadyReady)
    {
        // Reduce noise from accelerator signal
        accelPedal65535 = AverageSignal(accelPedal1023, accelPedal65535, accelPin, \
          &total, &counter, array, &average);

        // Serial.print("Averaged accelerator pedal signal: ");
        // Serial.println(accelPedal65535);
        // Serial.println();

        // Reduce noise from brake signal
        brakePedal65535 = AverageSignal(brakePedal1023, brakePedal65535, brakePin, \
            &total, &counter, array, &average);

        // Send revsied signal to motor
        SendPedalMessage(accelPedal65535, TransmittingSpeedCommandValue, accelMessage);
    }

    // delay(500);
}