#include "ECU.h"

// Set array elements to 0
void InitializeArray(int array[], const int size)
{
    // Loop through each element of array
    for (int index = 0; index < size; index++)
    {
        // Set each element to 0
        array[index] = 0;
    }
}

// Check if the vehice is ready to drive
bool ReadyToDrive(bool * isAlreadyReady, const int buttonPin, const int brakePin, \
    const int soundPin, const int runPin, const int goPin)
{
    // Check ready to drive state
    if (*isAlreadyReady)
    {
        Serial.println("Vehicle is ready to drive");
        Serial.println();

        return *isAlreadyReady;
    }

    // Check if button and brake are pressed
    if (digitalRead(buttonPin) == HIGH && digitalRead(brakePin) == HIGH)
    {
        // Set RTD to true
        *isAlreadyReady = true;

        // Activate RTD sound pin
        digitalWrite(soundPin, HIGH);
        delay(500);
        digitalWrite(soundPin, LOW);

        // Turn of Run & Go signal pins
        digitalWrite(runPin, HIGH);
        digitalWrite(goPin, HIGH);

        Serial.println("Vehicle is ready to drive");
        Serial.println();

        return *isAlreadyReady;
    }

    Serial.println("Not ready");
    Serial.println();

    return 0;
}

// Reduce noise from pedal signals
int AverageSignal(int initialPedalSignal, int finalPedalSignal, const int pin, \
    int * total, int * counter, int array[], int * average)
{
    // Read voltage signal
    initialPedalSignal = analogRead(pin);

    // Check if the brake pedal is used
    if (pin == BRAKE) {initialPedalSignal = 1023 - initialPedalSignal;}

    // Check and set analog value of signal
    if (initialPedalSignal > 1023) {initialPedalSignal = 1023;}

    // Average the analog read value to reduce noise
    *total -= array[*counter];
    array[*counter] = initialPedalSignal;
    *total += array[*counter];
    (*counter)++;

    // Reset counter
    if (*counter >= ARRAY_SIZE) {*counter = 0;}

    // Calculate new average
    *average = (*total) / ARRAY_SIZE;

    // Map value to a 2 byte value
    finalPedalSignal = (65536 * (*average)) / 1023;

    return finalPedalSignal;
}

// Send the up-converted signal to Bamocar
void SendPedalMessage(int finalPedalSignal, CAN_message_t messageValue, int messageID)
{
    // Head of Message
    messageValue.flags.extended = 0;
    messageValue.flags.remote = 0;
    messageValue.flags.overrun = 0;
    messageValue.flags.reserved = 0;
    messageValue.id = messageID;
    messageValue.len = 3;

    // Data Field
    int firstByte = finalPedalSignal & (0b0000000011111111);
    int secondByte = (finalPedalSignal & (0b1111111100000000)) >> 8;

    // Message data
    messageValue.buf[0] = 0x31;
    messageValue.buf[1] = firstByte;
    messageValue.buf[2] = secondByte;

    // Send to motor
    Can1.write(messageValue);
}

// Check to activate the brake light
void BrakeLight(const int brakePin, const int brakeLightPin)
{
    // Check if the brake is being pressed
    while (digitalRead(brakePin) == HIGH)
    {
        // Activate pin that sends voltage to brake light
        digitalWrite(brakeLightPin, HIGH);
    }

    // Turn off brake light
    digitalWrite(brakeLightPin, LOW);
}

// Check if two accelerator sensors have matching outputs
void CheckAPPS(int * signal1, int * signal2)
{
    // Determine if each sensor is within 10% of each other
    if (abs((signal1 - signal2) / (float)signal2) > 0.1)
    {
        // Assign signals to 0 to send no power
        *signal1 = 0;
        *signal2 = 0;
    }
}