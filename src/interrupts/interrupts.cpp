#include "interrupts/interrupts.h"

// Initialize variables
volatile bool IRQHandler::bShutdownCircuitOpen = false;
volatile bool IRQHandler::bPedalCalibrationMode = false;
volatile uint8_t IRQHandler::errorBuf = 0;
WDT_T4<WDT1> IRQHandler::WDT;

/*-------------------------------------------------------------------------------------------------
 Watchdog timer (WDT) timeout
-------------------------------------------------------------------------------------------------*/
void IRQHandler::CallbackWDT() {
	// Warn user to feed
    DebugPrintln("WARNING: FEED SOON OR RESET");
}

/*-------------------------------------------------------------------------------------------------
 Initialize WDT timeout period and functionality
-------------------------------------------------------------------------------------------------*/
void IRQHandler::ConfigureWDT() {
    WDT_timings_t config;

    config.trigger = 5; // Callback function is called every 5 seconds
    config.timeout = 10; // MCU resets if WDT not fed after 10 seconds
    config.callback = CallbackWDT; // Set callback function
    WDT.begin(config);

    DebugPrintln("WATCHDOG CONFIGURED");
}

/*-------------------------------------------------------------------------------------------------
 Periodically feed the watchdog
-------------------------------------------------------------------------------------------------*/
void IRQHandler::FeedWDT() {
    static uint32_t feed = millis();
    
    // Feed every 4 seconds
    if ( millis() - feed > 4000 ) {
        feed = millis();
        WDT.feed();
    }
}

/*-------------------------------------------------------------------------------------------------
 Configure pins to be interrupt controlled
-------------------------------------------------------------------------------------------------*/
void SetupInterrupts() {
    // Set RTD button pin to be interrupt controlled for pedal calibration
	attachInterrupt(digitalPinToInterrupt(PIN_RTD_BUTTON), PedalCalibrationISR, CHANGE);
}

/*-------------------------------------------------------------------------------------------------
 Set shutdown circuit error high
-------------------------------------------------------------------------------------------------*/
void ShutdownCircuitISR() {
    // Set SDC error flag high
    IRQHandler::SetShutdownState(true);

    // Set the SDC error bit high
    IRQHandler::SetErrorBuffer( IRQHandler::GetErrorBuffer() | (1 << ERROR_CODE_SHUTDOWN) );
}

/*-------------------------------------------------------------------------------------------------
 Set a flag high to initiate pedal calibration mode
-------------------------------------------------------------------------------------------------*/
void PedalCalibrationISR() {
    // Set pedal calibration mode flag high when RTD button is pressed
    IRQHandler::SetCalibrationMode( digitalRead(PIN_RTD_BUTTON) );
}
