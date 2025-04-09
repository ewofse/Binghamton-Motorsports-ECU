#include "core/pump.h"

/*-----------------------------------------------------------------------------
 Pump controller constructor
-----------------------------------------------------------------------------*/
pumpController::pumpController(uint8_t pinValue, double Kp, double Ki, double Kd) :
    // Initialize PID controller and PWM timer
    controller(Kp, Ki, Kd, PID::Direct),
    pin(pinValue, OUTPUT),
    timer( new Teensy_PWM(pinValue, 0.0, 0.0) )
{
    frequency = 0.0;
    dutyCycle = 0.0;
    input = 0.0;
    output = 0.0;
    setpoint = 0.0;
}

/*-----------------------------------------------------------------------------
 Change the frequency and duty cycle of the PWM
-----------------------------------------------------------------------------*/
void pumpController::RunPWM(float frequency, float dutyCycle) {
    // Update the frequency and duty cycle of the output pin signal
    timer->setPWM(pin.GetPin(), frequency, dutyCycle);
}

/*-----------------------------------------------------------------------------
 Initiate the PID controller
-----------------------------------------------------------------------------*/
void pumpController::BeginPID() {
    // Begin the PID feedback control system
    controller.Start(input, output, setpoint);
}

/*-----------------------------------------------------------------------------
 Get the PID output
-----------------------------------------------------------------------------*/
void pumpController::RunPID() {
    output = controller.Run(input);
}

/*-----------------------------------------------------------------------------
 Modify the gain terms of PID controller
-----------------------------------------------------------------------------*/
void pumpController::TunePIDGains(double Kp, double Ki, double Kd) {
    // Set controller with new gain terms
    controller.SetTunings(Kp, Ki, Kd);
}

/*-----------------------------------------------------------------------------
 Drive the pump pin to drive the relay and supply power to the pump
-----------------------------------------------------------------------------*/
void pumpController::TogglePump() {
    // Check battery pack threshold has been reached
    if ( IRQHandler::GetBatteryTemperature() > PACK_THRESHOLD ) {
        // Enable the pump
        pin.WriteOutput(HIGH);
    } else {
        // Disable the pump
        pin.WriteOutput(LOW);
    }
}
