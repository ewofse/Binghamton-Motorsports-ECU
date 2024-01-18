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
    // Return if vehicle already is ready to drive
    if(*isAlreadyReady)
    {
        Serial.println("Vehicle is already ready to drive");
        Serial.println();

        return *isAlreadyReady;
    }

    Serial.println(digitalRead(buttonPin));
    Serial.println();

    Serial.println(digitalRead(brakePin));
    Serial.println();

    // Check if button and brake are pressed
    if(digitalRead(buttonPin) == HIGH && digitalRead(brakePin) == HIGH)
    {
        *isAlreadyReady = true;

        Serial.println("Vehicle is now ready to drive");
        Serial.println();

        return *isAlreadyReady;
    }
    else
    {
        Serial.println("Not ready");
        Serial.println();
    }

    return 0;
}

// Average the signal
int AverageSignal(int initialPedalSignal, int finalPedalSignal, const int pin, int * total, \
    int * counter, int array[], int * average)
{
    short int temp;

    // Read voltage signal
    initialPedalSignal = analogRead(pin);

    // Serial.println(initialPedalSignal);
    // Serial.println();

    // Set pedal signal to max value if signal read in is greater
    if(initialPedalSignal > 1023)
    {
        initialPedalSignal = 1023;
    }

    // Average the analog read value to reduce noise
    *total -= array[*counter];
    array[*counter] = initialPedalSignal;
    *total += array[*counter];
    (*counter)++;

    // Reset counter
    if(*counter >= ARRAY_SIZE) 
    {
        *counter = 0;
    }

    *average = (*total) / ARRAY_SIZE;

    temp = *average;

    // Serial.println("Average:");
    // Serial.println(temp);
    // Serial.println();

    // Map value to a 2 byte value
    finalPedalSignal = (65536 * temp) / 1023;

    // Serial.println("10 bit value:");
    // Serial.println(temp);
    // Serial.println();

    // Serial.println("2 byte value:");
    // Serial.println(finalPedalSignal);
    // Serial.println();

    return finalPedalSignal;
}

void SendPedalMessage(int finalPedalSignal, CAN_message_t messageValue)
{
    // Head of Message
    messageValue.flags.extended = 0;
    messageValue.flags.remote = 0;
    messageValue.flags.overrun = 0;
    messageValue.flags.reserved = 0;
    messageValue.id = 0x201;
    messageValue.len = 3;

    // Data Field
    int firstByte = finalPedalSignal & (0b0000000011111111);
    int secondByte = (finalPedalSignal & (0b1111111100000000)) >> 8;

    messageValue.buf[0] = 0x31;
    messageValue.buf[1] = firstByte;
    messageValue.buf[2] = secondByte;

    Serial.println("First byte of value:");
    Serial.println(messageValue.buf[1]);
    Serial.println();

    Serial.println("Second byte of value:");
    Serial.println(messageValue.buf[2]);
    Serial.println();

    // Send to motor
    Can1.write(messageValue);
}
