// Safe guards
#ifndef PUMP_H
#define PUMP_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "general.h"
#include "pin.h"
#include "interrupts/interrupts.h"
#include "comms/CAN.h"

/*------------------------------------------
 Macros - Temperature thresholds
------------------------------------------*/
#define ACTIVATE_PUMP_THRESHOLD     60
#define DEACTIVATE_PUPM_THRESHOLD   50

/*-------------------------------------------------------------------------------------------------
 Pump PID controller
-------------------------------------------------------------------------------------------------*/
class pumpController {
    public:
        // Constructor
        pumpController(uint8_t pinValue, double Kp, double Ki, double Kd);

        // Getters
        digitalPin GetPin() { return pin; }

        float GetPIDInput() { return input; }
        float GetPIDOutput() { return output; }
        float GetPIDSetpoint() { return setpoint; }

        uint8_t GetPWMFrequency() { return frequency; }
        uint8_t GetPWMDutyCycle() { return dutyCycle; }
        
        // Setters
        void SetPIDInput(float value) { input = value; }
        void SetPIDOutput(float value) { output = value; }
        void SetPIDSetpoint(float value) { setpoint = value; }

        void SetPWMFrequency(uint8_t value) { frequency = value; }
        void SetPWMDutyCycle(uint8_t value) { dutyCycle = value; }

        // Data methods
        void RunPWM(float frequency, float dutyCycle);
        void BeginPID();
        void RunPID();
        void TunePIDGains(double Kp, double Ki, double Kd);

    private:
        PID_v2 controller;
        digitalPin pin;
        Teensy_PWM * timer; // To be used only with a PWM pin and if pump allows for PWM

        float input;
        float output;
        float setpoint;

        uint8_t frequency;
        uint8_t dutyCycle;
};

// End safe guards
#endif /* PUMP_H */
