// Safe guards
#ifndef READYTODRIVE_H
#define READYTODRIVE_H

// Libraries
#include <stdint.h>

#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

// Macros
#define ARRAY_SIZE 40

// Constants
const int buttonPin = 31;
const int brakePin = 22;
const int accelPin = 22;

// Prototypes
bool ReadyToDrive(bool * isAlreadyReady, int buttonPin, int brakePin);

int AverageSignal(int initialPedalSignal, int finalPedalSignal, const int pin, int * total, \
    int * counter, int array[], int * average);

void InitializeArray(int array[], int size);
void SendPedalMessage(int finalPedalSignal, CAN_message_t messageValue);

/*
    Ready to drive:

    The car should be ready to drive when:
        1. The driver presses the "start engine" button
        2. The driver is pressing the brake
    
    Once the car is ready to drive... it should bypass the check
    Brake pedal functionality should 

*/

// End safe guards
#endif