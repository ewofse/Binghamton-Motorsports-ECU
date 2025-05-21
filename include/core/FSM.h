// Safe guards
#ifndef FSM_H
#define FSM_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "sensors/hall.h"
#include "pump.h"

/*-------------------------------------------------------------------------------------------------
 System Data
-------------------------------------------------------------------------------------------------*/
class systemData {
    public:
        // Constructor
        systemData();

        // Getters
        hall GetAPPS1() { return APPS1; }
        hall GetAPPS2() { return APPS2; }
        hall GetBSE() { return BSE; }

        pumpController GetPumpController() { return pump; }

        elapsedMillis GetResetTimer() { return timers.resetTimer; }
        bool GetResetTimerFlag() { return timers.bResetTimerStarted; }

        elapsedMillis GetBuzzerTimer() { return timers.buzzerTimer; }
        bool GetBuzzerTimerFlag() { return timers.bBuzzerActive; }

        elapsedMillis GetChargeTimer() { return timers.chargeTimer; }
        bool GetChargeTimerFlag() { return timers.bChargeTimerStarted; }

        bool Get100msFlag() { return timers.b100msPassed; }

        digitalPin & GetRTDButtonPin() { return pinRTDButton; }
        analogPin & GetSDCTapPin() { return pinSDCTap; }

        digitalPin GetRUNPin() { return pinRUN; }
        digitalPin GetGOPin() { return pinRFE; }
        digitalPin GetRTDBuzzerPin() { return pinRTDBuzzer; }
        digitalPin GetBrakeLightPin() { return pinBrakeLight; }
        digitalPin GetResetPin() { return pinReset; }

        digitalPin GetAIRPlusPin() { return pinAIRPlus; }

        digitalPin GetPumpPin() { return pinPump; }
        digitalPin GetPumpSwitchPin() { return pinPumpSwitch; }
        digitalPin GetFaultLEDPin() { return pinFaultLED; }

        uint8_t GetStateBuffer() { return stateBuf; }
        uint8_t GetFaultBuffer() { return faultBuf; }

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

        void SetStateBuffer(uint8_t value) { stateBuf = value; }
        void SetFaultBuffer(uint8_t value) { faultBuf = value; }

        // Data methods
        void ActivateBrakeLight();

        bool ReadyToDrive();

        void ActivateBamocar();

        void DeactivateBamocar();

        void ProcessAPPS(uint8_t * pTorqueRequest);

        void UpdatePedalStructures();

        void UpdateSDCTapBuffer();

        float GetLowerPercentAPPS();

        bool CheckAPPS();

        bool CheckPedalsOOR();

        bool CheckPedalPlausibility();
        
        bool CheckPedalImplausibility();

        bool CheckAllErrors();

        bool SetPedalBounds();

        void CalibratePedals();

        void CalibrateMotor();

        void RampPump(bool direction);

        void RunPump();

        void DebugPrintErrors();

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
 Vehicle Driving Control FSM
-------------------------------------------------------------------------------------------------*/
class systemFSM {
    public:
        // Constructor
        systemFSM();

        // Getters
        systemData GetSystemData() { return system; }

        // Methods
        void ProcessState();

    private:
        // State methods
        void PEDALS();
        void INIT();
        void PRECHARGE();
        void RTD();
        void IDLE();
        void DRIVE();
        void BRAKE();
        void FAULT();
        void CALIBRATE_PEDALS();
        void CALIBRATE_MOTOR();

        // All data modified and used within system
        systemData system;

        // Declare a function pointer to point to state member functions
        void (systemFSM::*state)();
};

// End safe guards
#endif /* FSM_H */
