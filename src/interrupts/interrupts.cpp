#include "interrupts/interrupts.h"

// Initialize variables
volatile bool IRQHandler::bShutdownCircuitOpen = false;
volatile bool IRQHandler::bPedalCalibrationMode = false;
volatile uint8_t IRQHandler::batteryTemperature = 0;
volatile uint8_t IRQHandler::errorBuf = 0;
volatile uint32_t IRQHandler::lastPressTime = 0;
WDT_T4<WDT1> IRQHandler::WDT;
IntervalTimer IRQHandler::faultLEDTimer;

/*-----------------------------------------------------------------------------
 Watchdog timer (WDT) timeout
-----------------------------------------------------------------------------*/
void IRQHandler::CallbackWDT() {
	// Warn user to feed
    DebugPrintln("WARNING: FEED SOON OR RESET");
}

/*-----------------------------------------------------------------------------
 Initialize WDT timeout period and functionality
-----------------------------------------------------------------------------*/
void IRQHandler::ConfigureWDT() {
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
void IRQHandler::FeedWDT() {
    static uint32_t feed = millis();
    
    // Feed every 4 seconds
    if ( millis() - feed > 4000 ) {
        feed = millis();
        WDT.feed();
    }
}

/*-----------------------------------------------------------------------------
 Set the ISR and frequency of the timer interrupt
-----------------------------------------------------------------------------*/
void IRQHandler::EnableFaultLEDTimer() {
    // Toggle the LED at 2 Hz
    faultLEDTimer.begin(ToggleFaultLED, 250000);
}

/*-----------------------------------------------------------------------------
 Unattach ISR from hardware timer
-----------------------------------------------------------------------------*/
void IRQHandler::DisableFaultLEDTimer() {
    // Disable timer interrupt for the fault LED
    faultLEDTimer.end();

	// Set fault indicator LED low
	digitalWriteFast(PIN_LED_FAULT, LOW);
}

/*-----------------------------------------------------------------------------
 Configure pins to be interrupt controlled
-----------------------------------------------------------------------------*/
void SetupInterrupts() {
    // Set RTD button pin to be interrupt controlled for pedal calibration
	attachInterrupt(digitalPinToInterrupt(PIN_RTD_BUTTON), PedalCalibrationISR, FALLING);
}

/*-----------------------------------------------------------------------------
 Set shutdown circuit error high
-----------------------------------------------------------------------------*/
void ShutdownCircuitISR() {
    // Set SDC error flag high
    IRQHandler::SetShutdownState(true);

    // Set the SDC error bit high
    IRQHandler::SetErrorBuffer( IRQHandler::GetErrorBuffer() | (1 << ERROR_CODE_SHUTDOWN) );
}

/*-----------------------------------------------------------------------------
 Set a flag high to initiate pedal calibration mode
-----------------------------------------------------------------------------*/
void PedalCalibrationISR() {
    // Get current system run time
    uint32_t currentPressTime = millis();

	// Detect if RTD button has been held for at least two seconds
	if ( currentPressTime - IRQHandler::GetLastPressTime() >= PEDAL_CALIBRATION_TIME ) {
		// Update timer
        IRQHandler::SetLastPressTime(currentPressTime);

        // Set pedal calibration mode flag high
		IRQHandler::SetCalibrationMode(true);
    }
}

/*-----------------------------------------------------------------------------
 Toggle the LED on the EV1.5 ECU PCB indicating an error
-----------------------------------------------------------------------------*/
void ToggleFaultLED() {
    // Toggle the LED
    EV1_5_TOGGLE_FAULT_LED();
}

/*-----------------------------------------------------------------------------
 Evalues bit zero of error buffer (Shutdown circuit error)
-----------------------------------------------------------------------------*/
bool ShutdownCircuitOpen() {
    // Check if shutdown circuit open
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_SHUTDOWN);
}

/*-----------------------------------------------------------------------------
 Evalues bit one of error buffer (APPS signal agreement error)
-----------------------------------------------------------------------------*/
bool PedalsDisagree() {
    // Check if pedals disagree
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_DISAGREE);
}

/*-----------------------------------------------------------------------------
 Evalues bit three of error buffer (Both pedals pressed error)
-----------------------------------------------------------------------------*/
bool BothPedalsPressed() {
    // Check if both pedals are pressed
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_APPS_BSE);
}

/*-----------------------------------------------------------------------------
 Evalues bit four of error buffer (Pedals out of range error)
-----------------------------------------------------------------------------*/
bool PedalsOOR() {
    // Check if pedals are out of range
    return IRQHandler::GetErrorBuffer() & (1 << ERROR_CODE_OOR);
}
