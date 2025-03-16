// Safe guards
#ifndef ISR_H
#define ISR_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "general.h"
#include "pin.h"

/*------------------------------------------
 Interrupt flags / external variables
------------------------------------------*/
extern volatile bool bShutdownCircuitOpen;
extern volatile bool bPedalCalibrationMode;
extern volatile uint8_t errorBuf;
extern WDT_T4<WDT1> WDT;

/*-------------------------------------------------------------------------------------------------
 Prototypes for Interrupt Service Routines (ISRs)
-------------------------------------------------------------------------------------------------*/
void ShutdownCircuitISR();

void PedalCalibrationISR();

void CallbackWDT();

void ConfigureWDT();

void FeedWDT();

// End safe guards
#endif /* ISR_H */
