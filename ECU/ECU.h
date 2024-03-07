// Header file safe guards
#ifndef ECU_H
#define ECU_H

// Libraries
#include <stdint.h>
#include <stdlib.h>

#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

// Macros - Pins
#define BRAKE 0
#define ACCELERATOR_ONE 0
#define ACCELERATOR_TWO 0
#define RTD_SOUND 0
#define RTD_BUTTON 0
#define RUN 0
#define GO 0
#define BRAKE_LIGHT 0

// Macros - CAN & Serial rates
#define BAUD_RATE 500000
#define BIT_RATE 9600

// Macros - CAN message IDs
#define CAN_MESSAGE_ACCELERATOR 0
#define CAN_MESSAGE_BRAKE 0

// Macros - OTHER
#define ARRAY_SIZE 40

// Prototypes

// Set array elements to 0
void InitializeArray(int array[], const int size);

// Check if the vehicle is ready to drive
bool ReadyToDrive(bool * isAlreadyReady, const int buttonPin, const int brakePin, \
    const int soundPin, const int runPin, const int goPin);

// Reduce noise from pedal signals
int AverageSignal(int initialPedalSignal, int finalPedalSignal, const int pin, \
    int * total, int * counter, int array[], int * average);

// Send the up-converted signal to Bamocar
void SendPedalMessage(int finalPedalSignal, CAN_message_t messageValue, int messageID);

// Check to activate the brake light
void BrakeLight(const int brakePin, const int brakeLightPin);

// Check APPS
void CheckAPPS(int * signal1, int * signal2);

// End safe guards
#endif