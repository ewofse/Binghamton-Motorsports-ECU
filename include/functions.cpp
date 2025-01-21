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
	#ifdef EV1_5
		pinMode(PIN_REGEN,           INPUT);
		pinMode(PIN_LED_FAULT,       OUTPUT);
		pinMode(PIN_CHARGE_ENABLE,   OUTPUT);
	#endif
}

/*-----------------------------------------------------------------------------
 Configure CAN bus baud rate and message FIFO and filters
-----------------------------------------------------------------------------*/
void ConfigureCANBus() {
	myCan.begin();
	myCan.setBaudRate(BAUD_RATE);
	myCan.enableFIFO();

	// Set message filters for reading
	myCan.setFIFOFilter(REJECT_ALL);
	myCan.setFIFOFilter(0, ID_CAN_MESSAGE_TX, STD);
}

/*-----------------------------------------------------------------------------
 Switch debouncer that elimates bouncing periods of 5ms
-----------------------------------------------------------------------------*/
bool ButtonDebouncer(uint8_t pin) {
	// The timer here acts as a counter to count 5ms
	static elapsedMillis timer = 0;

	// State register
	static bool state[2] = {LOW, LOW};
	const uint32_t debounceTime = 5;

	// Reset the timer when there is a change in input
	if ( state[0] != state[1] ) {
		timer = 0;
	}

	// Save the current and past signal values in the state register
	state[0] = state[1];
	state[1] = digitalRead(pin);

	// Counter is enabling the DFF every 5ms
	return (timer >= debounceTime) ? state[1] : state[0];
}

/*-----------------------------------------------------------------------------
 Rising edge pulser for buttons
-----------------------------------------------------------------------------*/
bool ButtonPulser(bool signal) {
	// State register
	static bool state[2] = {LOW, LOW};

	// On the next "rising clock edge" shift outputs in (FIFO)
	state[0] = state[1];
	state[1] = signal;

	// Q1 AND NOT Q0
	return state[1] & !state[0];
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
#ifdef EV1_5
	void ActivateFaultLED() {
		static uint32_t blinked = millis();

		// Toggle LED every 100 milliseconds
		if ( millis() - blinked > 100 ) {
			blinked = millis();
			digitalWrite( PIN_LED_FAULT, !digitalRead(PIN_LED_FAULT) );
		}
	}
#endif

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
bool MapCANMessage(CAN_message_t * pMessage1, CAN_message_t * pMessage2) {
	bool bMapped = true;

	switch (pMessage1->buf[0]) {
		case (REG_TEMP):
			pMessage2->id = ID_TEMP;
			break;

		case (REG_CURRENT):
			pMessage2->id = ID_CURRENT;
			break;

		case (REG_VOLTAGE):
			pMessage2->id = ID_VOLTAGE;
			break;

		case (REG_SPEED_ACTUAL):
			pMessage2->id = ID_SPEED;
			break;

		default:
			pMessage2->id = pMessage1->id;
			bMapped = false;

			// DebugPrintln("NO ID MATCH");
			break;
	}

	pMessage2->len = pMessage1->len;
	
	// Populate buffer for message
	for (int i = 0; i < pMessage2->len; ++i) {
		pMessage2->buf[i] = pMessage1->buf[i];
	}

	// Return true when the ID has been re-mapped
	return bMapped;
}

/*-----------------------------------------------------------------------------
 Send CAN Message
-----------------------------------------------------------------------------*/
void SendCANMessage(CAN_message_t * pMessage) {
	myCan.write(*pMessage);
	DebugPrintln("MESSAGE SENT");
}

/*-----------------------------------------------------------------------------
 Read CAN message
-----------------------------------------------------------------------------*/
void ReadCANMessage(CAN_message_t * pMessage) {
	// Attempt to read a message
	if ( myCan.read(*pMessage) ) {
		// Print the message ID and DLC
		DebugPrint("ID: 0x"); DebugPrintHEX( (*pMessage).id );
		DebugPrint(" LEN: "); DebugPrint( (*pMessage).len );
		DebugPrint(" DATA: ");

		// Print the message data buffer
		for (int i = 0; i < pMessage->len; ++i) {
			DebugPrintHEX(pMessage->buf[i]);
			DebugPrint(" ");
		}

		// Print the message timestamp
		DebugPrint("  TS: "); DebugPrintln( (*pMessage).timestamp );
		DebugPrintln("");
	}
}

/*-----------------------------------------------------------------------------
 Send CAN message of current state to dashboard
-----------------------------------------------------------------------------*/
void SendVehicleState(uint8_t state) {
	CAN_message_t msgCurrentState;

	// Apply correct ID and state value and send to dashboard
	PopulateCANMessage(&msgCurrentState, ID_CURRENT_STATE, PAR_STATE_DLC, &state);
	SendCANMessage(&msgCurrentState);
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
void WriteDataToFile(const char * strFileName, const String & strData, bool overwrite) {
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
void UpdateCurrentData(CAN_message_t * pMessage, String & strData, uint16_t pDataBuf[]) {
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
			DebugPrintln("NO CURRENT DATA READ");
			return;
	}

	// Store most recent phase current data to array
	pDataBuf[index] = (pMessage->buf[1] << 8) | pMessage->buf[2];

	// Convert data to string
	for (size_t i = 0; i < 4; ++i) {
		strData += String(pDataBuf[i]) + DELIMITER;
	}
}
