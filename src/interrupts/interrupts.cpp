#include "interrupts/interrupts.h"

// Initialize variables
volatile bool IRQHandler::bShutdownCircuitOpen = false;
volatile bool IRQHandler::bButtonHeld = false;
volatile uint8_t IRQHandler::motorTemperature = 0;
volatile uint8_t IRQHandler::errorBuf = 0;
volatile uint32_t IRQHandler::lastPressTime = 0;
WDT_T4<WDT1> IRQHandler::WDT;
IntervalTimer IRQHandler::faultLEDTimer;
IntervalTimer IRQHandler::fadeLEDTimer;

/*-----------------------------------------------------------------------------
 Watchdog timer (WDT) timeout
-----------------------------------------------------------------------------*/
void IRQHandler::CallbackWDT(void) {
	// Warn user to feed
    DebugPrintln("WARNING: FEED SOON OR RESET");
}

/*-----------------------------------------------------------------------------
 Initialize WDT timeout period and functionality
-----------------------------------------------------------------------------*/
void IRQHandler::ConfigureWDT(void) {
    WDT_timings_t config;

    config.trigger = 5; // Callback function is called every 5 seconds
    config.timeout = 10; // MCU resets if WDT not fed after 10 seconds
    config.callback = CallbackWDT; // Set callback function
    WDT.begin(config);

    DebugPrintln("WATCHDOG CONFIGURED");
}

/*-----------------------------------------------------------------------------
 Initialize WDT timeout period and functionality
-----------------------------------------------------------------------------*/
void IRQHandler::FeedWDT(void) {
    static uint32_t feed = millis();
    
    // Feed every 4 seconds
    if ( millis() - feed > 4000 ) {
        feed = millis();
        WDT.feed();
    }
}

/*-----------------------------------------------------------------------------
 Reset the system using WDT
-----------------------------------------------------------------------------*/
void IRQHandler::ResetWDT(void) {
    // Reset WDT
    WDT.reset();
}

/*-----------------------------------------------------------------------------
 Set the ISR and frequency of the fault LED timer interrupt
-----------------------------------------------------------------------------*/
void IRQHandler::EnableFaultLEDTimer(void) {
    // Toggle the LED at 2 Hz
    faultLEDTimer.begin(ToggleFaultLED, 250000);
}

/*-----------------------------------------------------------------------------
 Unattach ISR from hardware timer
-----------------------------------------------------------------------------*/
void IRQHandler::DisableFaultLEDTimer(void) {
    // Disable timer interrupt for the fault LED
    faultLEDTimer.end();

	// Set fault indicator LED low
	digitalWriteFast(PIN_LED_FAULT, LOW);
}

/*-----------------------------------------------------------------------------
 Set the ISR and frequency of the calibration timer interrupt
-----------------------------------------------------------------------------*/
void IRQHandler::EnableCalibrationTimer(void) {
    // Call the LED calibration timer functions at 1 kHz
    fadeLEDTimer.begin(CalibrationHeartbeat, 500);
}

/*-----------------------------------------------------------------------------
 Unattach ISR from hardware timer
-----------------------------------------------------------------------------*/
void IRQHandler::DisableCalibrationTimer(void) {
    // Disable the timer interrupts for the fault LED
    fadeLEDTimer.end();

    // Set the fault indactor LED low
    digitalWriteFast(PIN_LED_FAULT, LOW);
}

/*-----------------------------------------------------------------------------
 Configure pins to be interrupt controlled
-----------------------------------------------------------------------------*/
void SetupInterrupts(void) {
    // Set RTD button pin to be interrupt controlled for pedal calibration
    attachInterrupt(digitalPinToInterrupt(PIN_RTD_BUTTON), RTDButtonISR, CHANGE);
}

/*-----------------------------------------------------------------------------
 Set shutdown circuit error high
-----------------------------------------------------------------------------*/
void ShutdownCircuitISR(void) {
    // Set SDC error flag high
    IRQHandler::SetShutdownState(true);

    // Set the SDC error bit high
    IRQHandler::SetErrorBuffer( IRQHandler::GetErrorBuffer() | (1 << ERROR_CODE_SHUTDOWN) );
}

/*-----------------------------------------------------------------------------
 Set a flag high to initiate RTD button pressed
-----------------------------------------------------------------------------*/
void RTDButtonISR(void) {
    // Get current system run time
    static uint32_t pressStartTime = millis();

    // Default the flag to low
    IRQHandler::SetButtonHeld(false);

    // Check if RTD button was pressed and held for 2 seconds
    if ( digitalRead(PIN_RTD_BUTTON) ) {
        // Begin a timer
        pressStartTime = millis();
    } else if ( millis() - pressStartTime >= PEDAL_CALIBRATION_TIME ) {
        // Set flag high
        IRQHandler::SetButtonHeld(true);
    }
}

/*-----------------------------------------------------------------------------
 Toggle the LED on the EV1.5 ECU PCB indicating an error
-----------------------------------------------------------------------------*/
void ToggleFaultLED(void) {
    // Toggle the LED
    EV1_5_TOGGLE_FAULT_LED();
}

/*-----------------------------------------------------------------------------
  Set the fault LED to fade in and out
-----------------------------------------------------------------------------*/
void CalibrationHeartbeat(void) {
    static bool bIncreaseDutyCycle = true;
    static uint8_t PWMCounter = 0;
    static uint8_t currentDutyCycle = 0;
    static uint8_t timeCounter = 0;

    // Check the PWM counter is less than duty cycle
    if (PWMCounter < currentDutyCycle) {
        // Turn on the LED
        digitalWriteFast(PIN_LED_FAULT, HIGH);
    } else {
        // Turn off the LED
        digitalWriteFast(PIN_LED_FAULT, LOW);
    }

    // Update PWM counter
    PWMCounter = (PWMCounter + 1) % 256;

    // Check if 10 ms has elapsed
    if (++timeCounter >= 10) {
        // Reset time counter
        timeCounter = 0;

        // Check if duty cycle is currently increasing
        if (bIncreaseDutyCycle) {
            // Check if maximum duty cycle has been reached
            if (++currentDutyCycle >= 255) {
                // Begin decrementing duty cycle
                bIncreaseDutyCycle = false;
            }
        } else {
            // Check if minimum duty cycle has been reached
            if (--currentDutyCycle == 0) {
                // Begin incrementing duty cycle
                bIncreaseDutyCycle = true;
            }
        }
    }
}

/*-----------------------------------------------------------------------------
 Evalues bit zero of error buffer (Shutdown circuit error)
-----------------------------------------------------------------------------*/
bool ShutdownCircuitOpen(void) {
    // Check if shutdown circuit open
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_SHUTDOWN);
}

/*-----------------------------------------------------------------------------
 Evalues bit one of error buffer (APPS signal agreement error)
-----------------------------------------------------------------------------*/
bool PedalsDisagree(void) {
    // Check if pedals disagree
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_DISAGREE);
}

/*-----------------------------------------------------------------------------
 Evalues bit three of error buffer (Both pedals pressed error)
-----------------------------------------------------------------------------*/
bool BothPedalsPressed(void) {
    // Check if both pedals are pressed
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_APPS_BSE);
}

/*-----------------------------------------------------------------------------
 Evalues bit four of error buffer (Pedals out of range error)
-----------------------------------------------------------------------------*/
bool PedalsOOR(void) {
    // Check if pedals are out of range
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_OOR);
}
