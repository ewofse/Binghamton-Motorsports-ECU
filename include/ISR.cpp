#include "main.h"

/*-----------------------------------------------------------------------------
 Read a CAN message using interrupts (FIFO)
-----------------------------------------------------------------------------*/
void ProcessCANMessage(const CAN_message_t &message) {
	CAN_message_t msgPiRX = message;
	// CAN_message_t msgCurrentTX = message;
	// static uint16_t currentData[4] = {0, 0, 0, 0};
	// static String strCurrentDraw = "";

	// Print the message ID and DLC
	DebugPrint("ID: 0x"); DebugPrintHEX(message.id);
	DebugPrint(" LEN: "); DebugPrint(message.len);
	DebugPrint(" DATA: ");

	// Print the message data buffer
	for (int i = 0; i < message.len; ++i) {
		DebugPrintHEX(message.buf[i]);
		DebugPrint(" ");
	}
	
	// Print the message timestamp
	DebugPrint("  TS: "); DebugPrintln(message.timestamp);

	// Check the message is from the Bamocar and send to dashboard
	if ( MapCANMessage(message.buf[0], msgPiRX) ) {
		SendCANMessage(msgPiRX);
	}

	// Save new current data if current message is read
    // if ( UpdateCurrentData(&msgCurrentTX, strCurrentDraw, currentData) ) {
	// 	WriteDataToFile(FILE_CURRENT_DRAW, strCurrentDraw, OVERWRITE);
	// 	strCurrentDraw = "";
	// }
}

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
