#include "main.h"

/*-------------------------------------------------------------------------------------------------
 Set shutdown circuit error high
-------------------------------------------------------------------------------------------------*/
void ShutdownCircuitISR() {
    bShutdownCircuitOpen = true;
    errorBuf |= (1 << 0);
}

/*-------------------------------------------------------------------------------------------------
 Watchdog timer (WDT) timeout
-------------------------------------------------------------------------------------------------*/
void CallbackWDT() {
    DebugPrintln("WARNING: FEED SOON OR RESET");
}

/*-------------------------------------------------------------------------------------------------
 Initialize WDT timeout period and functionality
-------------------------------------------------------------------------------------------------*/
void ConfigureWDT() {
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
void FeedWDT() {
    static uint32_t feed = millis();
    
    // Feed every 4 seconds
    if ( millis() - feed > 4000 ) {
        feed = millis();
        WDT.feed();
    }
}