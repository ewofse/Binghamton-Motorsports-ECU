// Safe guards
#ifndef ISR_H
#define ISR_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>

/*------------------------------------------
 Interrupt flags / external variables
------------------------------------------*/
extern volatile bool bShutdownCircuitOpen;
extern volatile uint8_t errorBuf;
extern WDT_T4<WDT1> WDT;

/*-------------------------------------------------------------------------------------------------
 Prototypes for Interrupt Service Routines (ISRs)
-------------------------------------------------------------------------------------------------*/
void ProcessCANMessage(const CAN_message_t &message);

void ShutdownCircuitISR();

void CallbackWDT();

void ConfigureWDT();

void FeedWDT();

// End safe guards
#endif /* ISR_H */
