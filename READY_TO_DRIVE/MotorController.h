// Header file safeguards
#ifndef MOTORCONTROLLER_H
#define MOTOROCONTROLLER_H

// Include for serial monitor output
#include <stdint.h>

// CAN libraries
#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

// Macros
#define SIZE 40
#define ACCELERATOR_PIN 23
#define BUTTON_PIN 30
#define BRAKE_PIN 23

// Prototypes
void Pedal(uint16_t pedal_10, uint16_t pedal_16, int pin);
int ReadyToDrive();

// End safeguards
#endif