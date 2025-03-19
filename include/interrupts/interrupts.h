// Safe guards
#ifndef INTERRUPTS_H
#define INTERRUPTS_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "core/general.h"
#include "core/pin.h"

/*-------------------------------------------------------------------------------------------------
 // Interrupts with encapsulation fancy buzzword ahh
-------------------------------------------------------------------------------------------------*/
class IRQHandler {
    public:
        // Getters
        static bool GetShutdownState() { return bShutdownCircuitOpen; }
        static bool GetCalibrationMode() { return bPedalCalibrationMode; }
        static uint8_t GetErrorBuffer() { return errorBuf; }

        // Setters
        static void SetShutdownState(bool flag) { bShutdownCircuitOpen = flag; }
        static void SetCalibrationMode(bool flag) { bPedalCalibrationMode = flag; }
        static void SetErrorBuffer(uint8_t value) { errorBuf = value; }

        // Watchdog methods
        static void ConfigureWDT();
        static void FeedWDT();
        static void CallbackWDT();

    private:
        // Data modified in ISRs
        static volatile bool bShutdownCircuitOpen;
        static volatile bool bPedalCalibrationMode;
        static volatile uint8_t errorBuf;

        // Watchdog timer object
        static WDT_T4<WDT1> WDT;
};

/*-------------------------------------------------------------------------------------------------
 Prototypes for Interrupt Service Routines (ISRs)
-------------------------------------------------------------------------------------------------*/
void SetupInterrupts();

void ShutdownCircuitISR();

void PedalCalibrationISR();

// End safe guards
#endif /* INTERRUPTS_H */
