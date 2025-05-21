// Safe guards
#ifndef INTERRUPTS_H
#define INTERRUPTS_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "core/general.h"
#include "core/pin.h"

/*-------------------------------------------------------------------------------------------------
 Interrupt handler (through a static class)
-------------------------------------------------------------------------------------------------*/
class IRQHandler {
    public:
        // Getters
        static bool GetShutdownState() { return bShutdownCircuitOpen; }
        static bool GetButtonHeld() { return bButtonHeld; }
        static uint8_t GetErrorBuffer() { return errorBuf; }
        static uint32_t GetLastPressTime() { return lastPressTime; }

        static uint8_t GetMotorTemperature() { return motorTemperature; }

        // Setters
        static void SetShutdownState(bool flag) { bShutdownCircuitOpen = flag; }
        static void SetButtonHeld(bool flag) { bButtonHeld = flag; }
        static void SetErrorBuffer(uint8_t value) { errorBuf = value; }
        static void SetLastPressTime(uint32_t value) { lastPressTime = value; }
        
        static void SetMotorTemperature(uint8_t value) { motorTemperature = value; }

        // Watchdog methods
        static void ConfigureWDT();
        static void FeedWDT();
        static void CallbackWDT();
        static void ResetWDT();

        // LED methods
        static void EnableFaultLEDTimer();
        static void DisableFaultLEDTimer();

        static void EnableCalibrationTimer();
        static void DisableCalibrationTimer();

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
void SetupInterrupts();

void ShutdownCircuitISR();

void RTDButtonISR();

void ToggleFaultLED();

void CalibrationHeartbeat();

/*-------------------------------------------------------------------------------------------------
 ISR data related functions
-------------------------------------------------------------------------------------------------*/
bool ShutdownCircuitOpen();

bool PedalsDisagree();

bool BothPedalsPressed();

bool PedalsOOR();

// End safe guards
#endif /* INTERRUPTS_H */
