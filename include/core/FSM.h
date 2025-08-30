// Safe guards
#ifndef FSM_H
#define FSM_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include <stdint.h>

#include "interrupts/interrupts.h"
#include "sensors/hall.h"
#include "comms/CAN.h"
#include "daq/DAQ.h"
#include "general.h"
#include "pump.h"
#include "pin.h"

/*-------------------------------------------------------------------------------------------------
 System Data
-------------------------------------------------------------------------------------------------*/
class systemData {
    public:
        // Constructor
        systemData(void);

        // Getters
        hall GetAPPS1(void) { return APPS1; }
        hall GetAPPS2(void) { return APPS2; }
        hall GetBSE(void) { return BSE; }

        pumpController GetPumpController(void) { return pump; }

        elapsedMillis GetResetTimer(void) { return timers.resetTimer; }
        bool GetResetTimerFlag(void) { return timers.bResetTimerStarted; }

        elapsedMillis GetBuzzerTimer(void) { return timers.buzzerTimer; }
        bool GetBuzzerTimerFlag(void) { return timers.bBuzzerActive; }

        elapsedMillis GetChargeTimer(void) { return timers.chargeTimer; }
        bool GetChargeTimerFlag(void) { return timers.bChargeTimerStarted; }

        bool Get100msFlag(void) { return timers.b100msPassed; }

        digitalPin & GetRTDButtonPin(void) { return pinRTDButton; }
        analogPin & GetSDCTapPin(void) { return pinSDCTap; }

        digitalPin & GetRUNPin(void) { return pinRUN; }
        digitalPin & GetGOPin(void) { return pinRFE; }
        digitalPin & GetRTDBuzzerPin(void) { return pinRTDBuzzer; }
        digitalPin & GetBrakeLightPin(void) { return pinBrakeLight; }
        digitalPin & GetResetPin(void) { return pinReset; }

        digitalPin & GetAIRPlusPin(void) { return pinAIRPlus; }

        digitalPin & GetPumpPin(void) { return pinPump; }
        digitalPin & GetPumpSwitchPin(void) { return pinPumpSwitch; }
        digitalPin & GetFaultLEDPin(void) { return pinFaultLED; }

        uint8_t GetStateBuffer(void) { return stateBuf; }
        uint8_t GetFaultBuffer(void) { return faultBuf; }

        // Setters
        void SetAPPS1(hall sensor) { APPS1 = sensor; }
        void SetAPPS2(hall sensor) { APPS2 = sensor; }
        void SetBSE(hall sensor) { BSE = sensor; }

        void SetResetTimer(size_t value) { timers.resetTimer = value; }
        void SetResetTimerFlag(bool flag) { timers.bResetTimerStarted = flag; }

        void SetBuzzerTimer(size_t value) { timers.buzzerTimer = value; }
        void SetBuzzerTimerFlag(bool flag) { timers.bBuzzerActive = flag; }

        void SetChargeTimer(size_t value) { timers.chargeTimer = value; }
        void SetChargeTimerFlag(bool flag) { timers.bChargeTimerStarted = flag; }

        void Set100msFlag(bool flag) { timers.b100msPassed = flag; }

        void SetStateBuffer(systemState state) { stateBuf = static_cast<uint8_t>(state); }
        void SetFaultBuffer(uint8_t value) { faultBuf = value; }

        // Data methods
        void ActivateBrakeLight(void);

        bool ReadyToDrive(void);

        void ActivateBamocar(void);

        void DeactivateBamocar(void);

        void ProcessAPPS(uint8_t * pTorqueRequest);

        void UpdatePedalStructures(void);

        void UpdateSDCTapBuffer(void);

        float GetLowerPercentAPPS(void);

        bool CheckAPPS(void);

        bool CheckPedalsOOR(void);

        bool CheckPedalPlausibility(void);
        
        bool CheckPedalImplausibility(void);

        bool CheckAllErrors(void);

        bool SetPedalBounds(void);

        void CalibratePedals(void);

        void CalibrateMotor(void);

        void RampPump(bool direction);

        void RunPump(void);

        void DebugPrintErrors(void);

    private:
        // Hall sensor objects
        hall APPS1;
        hall APPS2;
        hall BSE;

        // Pump controller
        pumpController pump;

        // Timers
        timers_t timers;

        // GPIO Pins
        digitalPin pinRTDButton;
        analogPin pinSDCTap;

        digitalPin pinRUN;
        digitalPin pinRFE;
        digitalPin pinRTDBuzzer;
        digitalPin pinBrakeLight;
        digitalPin pinReset;

        // EV1 specific pins
        digitalPin pinAIRPlus;

        // EV1.5 specific pins
        digitalPin pinPump;
        digitalPin pinPumpSwitch;
        digitalPin pinFaultLED;

        uint8_t stateBuf;
        uint8_t faultBuf;
};

/*-------------------------------------------------------------------------------------------------
 Top Level Vehicle Driving Control System
-------------------------------------------------------------------------------------------------*/
class systemVehicle {
    public:
        // Constructor
        systemVehicle(void);

        // Getters
        systemData GetSystemData(void) { return system; }

        // Methods
        void ProcessState(void);

    private:
        // State methods
        void PEDALS(void);
        void INIT(void);
        void PRECHARGE(void);
        void RTD(void);
        void IDLE(void);
        void DRIVE(void);
        void BRAKE(void);
        void FAULT(void);
        void CALIBRATE_PEDALS(void);
        void CALIBRATE_MOTOR(void);

        // All data modified and used within system
        systemData system;

        // Declare a function pointer to point to state member functions
        void (systemVehicle::*state)(void);
};

// End safe guards
#endif /* FSM_H */
