// Safe guards
#ifndef INTERRUPTS_H
#define INTERRUPTS_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include <Arduino.h>
#include <stdint.h>

#include <Watchdog_t4.h>

#include "core/general.h"
#include "core/pin.h"

/*-------------------------------------------------------------------------------------------------
 Interrupt handler (through a static class)
-------------------------------------------------------------------------------------------------*/
class IRQHandler {
    public:
        // Getters
        static bool GetShutdownState(void) { return bShutdownCircuitOpen; }
        static bool GetButtonHeld(void) { return bButtonHeld; }
        static uint8_t GetErrorBuffer(void) { return errorBuf; }
        static uint32_t GetLastPressTime(void) { return lastPressTime; }

        static uint8_t GetMotorTemperature(void) { return motorTemperature; }

        // Setters
        static void SetShutdownState(bool flag) { bShutdownCircuitOpen = flag; }
        static void SetButtonHeld(bool flag) { bButtonHeld = flag; }
        static void SetErrorBuffer(uint8_t value) { errorBuf = value; }
        static void SetLastPressTime(uint32_t value) { lastPressTime = value; }
        
        static void SetMotorTemperature(uint8_t value) { motorTemperature = value; }

        // Watchdog methods
        static void ConfigureWDT(void);
        static void FeedWDT(void);
        static void CallbackWDT(void);
        static void ResetWDT(void);

        // LED methods
        static void EnableFaultLEDTimer(void);
        static void DisableFaultLEDTimer(void);

        static void EnableCalibrationTimer(void);
        static void DisableCalibrationTimer(void);

    private:
        // Data modified in ISRs
        static volatile bool bShutdownCircuitOpen;
        static volatile bool bButtonHeld;
        static volatile uint8_t errorBuf;
        static volatile uint32_t lastPressTime;

        // Watchdog timer object
        static WDT_T4<WDT1> WDT;

        // Timers for ECU PCB EV1.5
        static IntervalTimer faultLEDTimer;
        static IntervalTimer fadeLEDTimer;

        // Pump control
        static volatile uint8_t motorTemperature; // In Celsius
};

/*-------------------------------------------------------------------------------------------------
 Prototypes for Interrupt Service Routines (ISRs)
-------------------------------------------------------------------------------------------------*/
void SetupInterrupts(void);

void ShutdownCircuitISR(void);

void RTDButtonISR(void);

void ToggleFaultLED(void);

void CalibrationHeartbeat(void);

/*-------------------------------------------------------------------------------------------------
 ISR data related functions
-------------------------------------------------------------------------------------------------*/
bool ShutdownCircuitOpen(void);

bool PedalsDisagree(void);

bool BothPedalsPressed(void);

bool PedalsOOR(void);

// End safe guards
#endif /* INTERRUPTS_H */
