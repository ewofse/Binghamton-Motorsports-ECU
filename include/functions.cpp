#include "main.h"

/*-----------------------------------------------------------------------------
 Set pin modes
-----------------------------------------------------------------------------*/
void SetPinModes() {
	pinMode(PIN_BSE,             INPUT);
	pinMode(PIN_APPS_ONE,        INPUT);
	pinMode(PIN_APPS_TWO,        INPUT);
	pinMode(PIN_RTD_BUTTON,      INPUT);
	// pinMode(PIN_VREF,            INPUT);
	
	// Shutdown circuit error controlled with interrupts
	pinMode(PIN_SHUTDOWN_TAP,    INPUT);
	attachInterrupt(digitalPinToInterrupt(PIN_SHUTDOWN_TAP), ShutdownCircuitISR, FALLING);

	pinMode(PIN_RUN,             OUTPUT);
	pinMode(PIN_GO,              OUTPUT);
	pinMode(PIN_RTD_SOUND,       OUTPUT);
	pinMode(PIN_BRAKE_LIGHT,     OUTPUT);
	pinMode(PIN_LED,             OUTPUT);

	// EV1.5 additional Teensy pins
	EV1_5_SET_PIN_MODES();
}

/*-----------------------------------------------------------------------------
 Configure the CAN bus network
-----------------------------------------------------------------------------*/
void ConfigureCANBus() {
	// Enable CAN bus
	myCan.begin();
	myCan.setBaudRate(BAUD_RATE);

	// Setup interrupts to receive messages in a queue
	myCan.setMaxMB(NUM_MAILBOXES);

	// Declare mailboxes for transmitting
	for (int i = NUM_RX_MAILBOXES; i < NUM_TX_MAILBOXES + NUM_RX_MAILBOXES; ++i) {
		myCan.setMB( (FLEXCAN_MAILBOX)i, TX, STD );
	}

	// Setup FIFO and message processing with interrupts
	myCan.enableFIFO();
	myCan.enableFIFOInterrupt();
	myCan.setFIFOFilter(REJECT_ALL);
	myCan.setFIFOFilter(0, ID_CAN_MESSAGE_TX, STD);
	myCan.onReceive(FIFO, ProcessCANMessage);
}

/*-----------------------------------------------------------------------------
 Switch debouncer that elimates bouncing periods of 5ms
-----------------------------------------------------------------------------*/
bool ButtonDebouncer(uint8_t pin) {
    const uint8_t debounceTime = 5;
    static elapsedMillis timer = 0;
    static uint8_t lastStableState = LOW;
    static uint8_t lastReading = LOW;

    // Read the current pin state
    uint8_t currentReading = digitalRead(pin);

    // Reset the timer if the input changes
    if (currentReading != lastReading) {
        timer = 0;
    }

    // Update the last raw reading
    lastReading = currentReading;

    // Update stable state when debounce time elapses
    if (timer >= debounceTime) {
        lastStableState = currentReading;
    }

    return lastStableState;
}

/*-----------------------------------------------------------------------------
 Rising edge pulser for buttons
-----------------------------------------------------------------------------*/
bool ButtonPulser(bool signal) {
    static uint8_t previousState = LOW;

    // Output is rising edge detection
    bool pulse = signal && !previousState;

    // Update the previous state
    previousState = signal;

    return pulse;
}

/*-----------------------------------------------------------------------------
 Check for driver RTD input
-----------------------------------------------------------------------------*/
bool ReadyToDrive(hall * pBSE) {
	float BSE = pBSE->GetPercentRequest() * 100;

	// Check brake and RTD Button are pressed
	return BSE >= PERCENT_BRAKE && ButtonPulser( ButtonDebouncer(PIN_RTD_BUTTON) );
}

/*-----------------------------------------------------------------------------
 Activate brake light when brake is pressed (10%)
-----------------------------------------------------------------------------*/
void ActivateBrakeLight(hall * pBSE) {
	if ( ( pBSE->GetPercentRequest() * 100 ) > PERCENT_BRAKE ) {
		digitalWrite(PIN_BRAKE_LIGHT, HIGH);
	} else {
		digitalWrite(PIN_BRAKE_LIGHT, LOW);
	}
}

/*-----------------------------------------------------------------------------
 Activate motor controller
-----------------------------------------------------------------------------*/
void ActivateBamocar() {
	// Digital parameters required by the motor controller to drive
	digitalWrite(PIN_RUN, HIGH);
	digitalWrite(PIN_GO, HIGH);
}

/*-----------------------------------------------------------------------------
 Deactivate motor controller
-----------------------------------------------------------------------------*/
void DeactivateBamocar() {
	// Digital parameters required by the motor controller to drive
	digitalWrite(PIN_RUN, LOW);
	digitalWrite(PIN_GO, LOW);
}

/*-----------------------------------------------------------------------------
 Blinking fault indicator LED on ECU PCB
-----------------------------------------------------------------------------*/
void ActivateFaultLED() {
	static uint32_t blinked = millis();

	// Toggle LED every 100 milliseconds
	if ( millis() - blinked > 100 ) {
		blinked = millis();
		EV1_5_TOGGLE_FAULT_LED();
	}
}

/*-----------------------------------------------------------------------------
 Populate CAN message frame (Bamocar write specific)
-----------------------------------------------------------------------------*/
void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, 
	uint8_t * pMessageBuf, uint8_t bamocarDestReg) {
	// Head of Message
	pMessage->flags.extended  =  PAR_EXTENDED;
	pMessage->flags.remote    =  PAR_REMOTE;
	pMessage->flags.overrun   =  PAR_OVERRUN;
	pMessage->flags.reserved  =  PAR_RESERVED;
	pMessage->id              =  ID;
	pMessage->len             =  DLC;

	pMessage->buf[0] = bamocarDestReg;

	// Bamocar wants Litte Endian Format (LIFO)
	for (int i = 1; i < DLC; ++i) {
		pMessage->buf[i] = pMessageBuf[DLC - i];
	}
}

/*-----------------------------------------------------------------------------
 Populate CAN message frame (Bamocar read specific)
-----------------------------------------------------------------------------*/
void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, uint8_t bamocarDestReg) {
	// Head of Message
	pMessage->flags.extended  =  PAR_EXTENDED;
	pMessage->flags.remote    =  PAR_REMOTE;
	pMessage->flags.overrun   =  PAR_OVERRUN;
	pMessage->flags.reserved  =  PAR_RESERVED;
	pMessage->id              =  ID;
	pMessage->len             =  DLC;

	// Bamocar read register request - See CAN Complete
	if (pMessage->len == PAR_RX_DLC) {
		pMessage->buf[0] = REG_READ;
		pMessage->buf[1] = bamocarDestReg;
		pMessage->buf[2] = TRANSMIT_ONCE;
	}
}

/*-----------------------------------------------------------------------------
Populate CAN message frame (Generalized)
-----------------------------------------------------------------------------*/
void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, uint8_t * pMessageBuf) {
	// Head of Message
	pMessage->flags.extended  =  PAR_EXTENDED;
	pMessage->flags.remote    =  PAR_REMOTE;
	pMessage->flags.overrun   =  PAR_OVERRUN;
	pMessage->flags.reserved  =  PAR_RESERVED;
	pMessage->id              =  ID;
	pMessage->len             =  DLC;

	// Populate buffer for custom message (error, state, etc.)
	for (int i = 0; i < DLC; ++i) {
		pMessage->buf[i] = pMessageBuf[i];
	}
}

/*-----------------------------------------------------------------------------
 Map a CAN message's ID, DLC, and data to another message
-----------------------------------------------------------------------------*/
bool MapCANMessage(uint8_t REGID, CAN_message_t &message) {
	bool bMapped = true;

	// Match first byte of Bamocar message buffer to ID of Raspberry Pi message
	switch (REGID) {
		case (REG_TEMP):
			message.id = ID_TEMP;
			break;

		case (REG_CURRENT):
			message.id = ID_CURRENT;
			break;

		case (REG_VOLTAGE):
			message.id = ID_VOLTAGE;
			break;

		case (REG_SPEED_ACTUAL):
			message.id = ID_SPEED;
			break;

		default:
			bMapped = false;
			// DebugPrintln("NO ID MATCH");
			break;
	}

	// Return true when the ID has been re-mapped
	return bMapped;
}

/*-----------------------------------------------------------------------------
 Send a CAN message (to any transmitting mailbox)
-----------------------------------------------------------------------------*/
void SendCANMessage(const CAN_message_t &message) {
	static uint8_t cnt = 0;
	myCan.write(message);
	++cnt;
	DebugPrintln(cnt);
	// DebugPrintln("MESSAGE SENT");
}

/*-----------------------------------------------------------------------------
 Send a CAN message to a specified transmitting mailbox
-----------------------------------------------------------------------------*/
void SendCANMessage(const CAN_message_t &message, const FLEXCAN_MAILBOX MB) {
	static uint8_t cnt = 0;
	myCan.write(MB, message);
	++cnt;
	DebugPrintln("MESSAGE SENT");
	DebugPrintln(cnt);
}

/*-----------------------------------------------------------------------------
 Periodically send less critical CAN messages (every 10ms)
-----------------------------------------------------------------------------*/
void SendCANMessagePeriodic(CAN_message_t * pMessages, FLEXCAN_MAILBOX * pMailboxes) {
	static uint32_t time = millis();
	static uint8_t messageIndex = 0;
	static uint8_t mailboxIndex = 0;

	// Check if 10ms has psased
	if ( millis() - time > MESSAGE_INTERVAL ) {
		// Reset timer
		time = millis();

		// Send the message
		SendCANMessage(pMessages[messageIndex], pMailboxes[mailboxIndex]);

		// Increment pointers and wrap around if necessary
		++messageIndex;
		++mailboxIndex;
		messageIndex %= NUM_MESSAGES_TX;
		mailboxIndex %= NUM_MESSAGES_TX;
	}
}

/*-----------------------------------------------------------------------------
 Split sensor data into array of integers
-----------------------------------------------------------------------------*/
uint16_t * SplitIntegerString(String strValue, const char delimiter) {
	String strData;

	uint8_t delimiterCount = 0;
	uint8_t start = 0;
	uint8_t end = strValue.indexOf(delimiter);

	// Count number of delimiters in data string
	for (uint8_t i = 0; i < strValue.length(); ++i) {
		if (strValue[i] == delimiter) {
			++delimiterCount;
		}
	}

	// Allocate memory for number of values
	uint16_t * result = (uint16_t *) malloc( delimiterCount * sizeof(uint16_t) );

	if (result != nullptr) {
		// Split the string and store values in array
		for (int i = 0; i < delimiterCount; ++i) {
			// Obtain the sensor value
			strData = strValue.substring(start, end);
			result[i] = strData.toInt();

			// Change bounds for next substring sensor value
			start = ++end;
			end = strValue.indexOf(delimiter, start);
		}
	} else {
		DebugErrorPrint("ERROR: NULL POINTER");
		EXIT
	}

	return result;
}

/*-----------------------------------------------------------------------------
 Write a string of data to a data file
-----------------------------------------------------------------------------*/
void WriteDataToFile(const char * strFileName, const String &strData, bool overwrite) {
	File fData;
	
	// Delete old data to overwrite file
	if (overwrite) {
		if ( SD.exists(strFileName) ) {
			SD.remove(strFileName);

			DebugPrint("OLD FILE DELETED: "); DebugPrintln(strFileName);
		}
	}

	// Open file for writing
	fData = SD.open(strFileName, FILE_WRITE);

	// Check file exists and add data
	if (fData) {
		fData.println(strData);
		fData.close();

		DebugPrint("DATA WRITTEN TO: "); DebugPrintln(strFileName);
	} else {
		DebugPrint("ERROR: OPENING "); DebugPrintln(strFileName);
	}
}

/*-----------------------------------------------------------------------------
 Store each phase current
-----------------------------------------------------------------------------*/
bool UpdateCurrentData(CAN_message_t * pMessage, String &strData, uint16_t pDataBuf[]) {
	bool bResult = true;
	uint8_t index = 0;

	// Match index to data array for proper storage
	switch (pMessage->buf[0]) {
		case (REG_CURRENT_PHASE_1):
			index = 0;
			break;

		case (REG_CURRENT_PHASE_2):
			index = 1;
			break;

		case (REG_CURRENT_PHASE_3):
			index = 2;
			break;

		case (REG_CURRENT):
			index = 3;
			break;

		default:
			bResult = false;
			DebugPrintln("NO CURRENT DATA READ");
	}

	// Check the incoming message is phase current
	if (bResult) {
		// Store most recent phase current data to array
		pDataBuf[index] = (pMessage->buf[1] << 8) | pMessage->buf[2];

		// Convert data to string
		for (size_t i = 0; i < 4; ++i) {
			strData += String(pDataBuf[i]) + DELIMITER;
		}
	}

	return bResult;
}
