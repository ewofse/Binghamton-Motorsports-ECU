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
        static bool GetCalibrationMode() { return bPedalCalibrationMode; }
        static uint8_t GetErrorBuffer() { return errorBuf; }
        static uint32_t GetLastPressTime() { return lastPressTime; }

        static uint8_t GetBatteryTemperature() { return batteryTemperature; }

        // Setters
        static void SetShutdownState(bool flag) { bShutdownCircuitOpen = flag; }
        static void SetCalibrationMode(bool flag) { bPedalCalibrationMode = flag; }
        static void SetErrorBuffer(uint8_t value) { errorBuf = value; }
        static void SetLastPressTime(uint32_t value) { lastPressTime = value; }
        
        static void SetBatteryTemperature(uint8_t value) { batteryTemperature = value; }

        // Watchdog methods
        static void ConfigureWDT();
        static void FeedWDT();
        static void CallbackWDT();

        // LED methods
        static void EnableFaultLEDTimer();
        static void DisableFaultLEDTimer();

    private:
        // Data modified in ISRs
        static volatile bool bShutdownCircuitOpen;
        static volatile bool bPedalCalibrationMode;
        static volatile uint8_t errorBuf;
        static volatile uint32_t lastPressTime;

        // Watchdog timer object
        static WDT_T4<WDT1> WDT;

        // Timer for ECU PCB EV1.5
        static IntervalTimer faultLEDTimer;

        // Pump control
        static volatile uint8_t batteryTemperature; // In Celsius
};

/*-------------------------------------------------------------------------------------------------
 Prototypes for Interrupt Service Routines (ISRs)
-------------------------------------------------------------------------------------------------*/
void SetupInterrupts();

void ShutdownCircuitISR();

void PedalCalibrationISR();

void ToggleFaultLED();

/*-------------------------------------------------------------------------------------------------
 ISR data related functions
-------------------------------------------------------------------------------------------------*/
bool ShutdownCircuitOpen();

bool PedalsDisagree();

bool BothPedalsPressed();

bool PedalsOOR();

// End safe guards
#endif /* INTERRUPTS_H */
