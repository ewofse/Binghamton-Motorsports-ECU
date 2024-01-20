#include "ReadyToDrive.h"

// Set array elements to 0
void InitializeArray(int array[], int size)
{
    for(int index = 0; index < size; index++)
    {
        array[index] = 0;
    }
}

// Check if the vehice is ready to drive
bool ReadyToDrive(bool * isAlreadyReady, int buttonPin, int brakePin)
{
    // Check ready to drive state
    if(*isAlreadyReady)
    {
        Serial.println("Vehicle is ready to drive");
        Serial.println();

        return *isAlreadyReady;
    }

    // Serial.print("Button digital read: ");
    // Serial.println(digitalRead(buttonPin));
    // Serial.println();

    // Serial.print("Brake digital read: ");
    // Serial.println(digitalRead(brakePin));
    // Serial.println();

    // Check if button and brake are pressed
    if(digitalRead(buttonPin) == HIGH && digitalRead(brakePin) == HIGH)
    {
        // Set RTD to true
        *isAlreadyReady = true;

        Serial.println("Vehicle is ready to drive");
        Serial.println();

        return *isAlreadyReady;
    }

    Serial.println("Not ready");
    Serial.println();

    return 0;
}

// Average the signal
int AverageSignal(int initialPedalSignal, int finalPedalSignal, const int pin, \
    int * total, int * counter, int array[], int * average)
{
    // Read voltage signal
    initialPedalSignal = analogRead(pin);

    // Check if the brake pedal is used
    if(pin == brakePin) {initialPedalSignal = 1023 - initialPedalSignal;}

    // Check and set analog value of signal
    if(initialPedalSignal > 1023) {initialPedalSignal = 1023;}

    // Serial.print("Initial pedal signal: ");
    // Serial.println(initialPedalSignal);
    // Serial.println();

    // Average the analog read value to reduce noise
    *total -= array[*counter];
    array[*counter] = initialPedalSignal;
    *total += array[*counter];
    (*counter)++;

    // Reset counter
    if(*counter >= ARRAY_SIZE) {*counter = 0;}

    // Serial.print("Counter: ");
    // Serial.println(*counter);
    // Serial.println();

    // Calculate new average
    *average = (*total) / ARRAY_SIZE;

    // Serial.print("Average: ");
    // Serial.println(*average);
    // Serial.println();

    // Map value to a 2 byte value
    finalPedalSignal = (65536 * (*average)) / 1023;

    return finalPedalSignal;
}

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

    // Serial.println("First byte of value:");
    // Serial.println(messageValue.buf[1]);
    // Serial.println();

    // Serial.println("Second byte of value:");
    // Serial.println(messageValue.buf[2]);
    // Serial.println();

    // Send to motor
    Can1.write(messageValue);
}