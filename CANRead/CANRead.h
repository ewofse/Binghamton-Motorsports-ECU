// Safe guards
#ifndef CANREAD_H
#define CANREAD_H

// Libraries
#include <stdint.h>

#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

// Constants
const int pin = 13;

// Prototypes
void SendMessage(CAN_message_t message);

void ReadMessage(CAN_message_t message, const int pin);

void OutputMessageContents(CAN_message_t message);

// End safe guards
#endif