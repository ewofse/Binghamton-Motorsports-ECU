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

/*-------------------------------------------------------------------------------------------------
 Pump PID controller
-------------------------------------------------------------------------------------------------*/
class pumpController {
    public:
        // Constructor
        pumpController(uint8_t pinValue, double Kp, double Ki, double Kd);

        // Getters
        float GetPWMFrequency() { return frequency; }
        float GetPWMDutyCycle() { return dutyCycle; }

        float GetPIDInput() { return input; }
        float GetPIDOutput() { return output; }
        float GetPIDSetpoint() { return setpoint; }

        digitalPin GetPin() { return pin; }
        
        // Setters
        void SetPWMFrequency(float value) { frequency = value; }
        void SetPWMDutyCycle(float value) { dutyCycle = value; }

        void SetPIDInput(float value) { input = value; }
        void SetPIDOutput(float value) { output = value; }
        void SetPIDSetpoint(float value) { setpoint = value; }

        // Data methods
        void RunPWM(float frequency, float dutyCycle);
        void BeginPID();
        void RunPID();
        void TunePIDGains(double Kp, double Ki, double Kd);

        // Pump control - Current setup follows no PWM in water pump
        // A nice change for EV2 would be using PWM and PID along with a PWM GPIO pin
        void TogglePump();

    private:
        PID_v2 controller;
        digitalPin pin;
        Teensy_PWM * timer; // To be used only with a PWM pin and pump allows for PWM

        float frequency;
        float dutyCycle;
        float input;
        float output;
        float setpoint;
};

// End safe guards
#endif /* PUMP_H */
