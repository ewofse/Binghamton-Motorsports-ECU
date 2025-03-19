#include "comms/CAN.h"

/*-----------------------------------------------------------------------------
 Configure the CAN bus network
-----------------------------------------------------------------------------*/
void ConfigureCANBus() {
	// Enable CAN bus
	myCan.begin();
	myCan.setBaudRate(BAUD_RATE);

	// Setup FIFO and message processing with interrupts
	myCan.enableFIFO();
	myCan.enableFIFOInterrupt();

	// Set message filters for reading messages
	myCan.setFIFOFilter(REJECT_ALL);
	myCan.setFIFOFilter(0, ID_CAN_MESSAGE_TX, STD);
	myCan.setFIFOFilter(1, ID_BATTERY_TEMP, STD);

	// Set a callback function used to process incoming messages
	myCan.onReceive(ProcessCANMessage);

	DebugPrintln("CAN BUS INITIALIZED");
}

/*-----------------------------------------------------------------------------
 Read a CAN message using interrupts (FIFO)
-----------------------------------------------------------------------------*/
void ProcessCANMessage(const CAN_message_t &message) {
	CAN_message_t msgPiRX = message;

	// Print the message ID and DLC
	DebugPrint("ID: 0x"); DebugPrintHEX(message.id);
	DebugPrint(" LEN: "); DebugPrint(message.len);
	DebugPrint(" DATA: ");

	// Print the message data buffer
	for (int i = 0; i < message.len; ++i) {
		DebugPrintHEX(message.buf[i]);
		DebugPrint(" ");
	}
	
	// Check the message is from the Bamocar and send to dashboard
	if ( message.id == ID_CAN_MESSAGE_TX && MapCANMessage(message.buf[0], msgPiRX) ) {
		SendCANMessage(msgPiRX);
	}

	// Check the message is from the Orion
	if ( message.id == ID_BATTERY_TEMP ) {
		// TODO: Update PID controller with current temp reading
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
	// Requesting data to be sent back instantaneously
	if (pMessage->len == PAR_RX_DLC) {
		pMessage->buf[0] = REG_READ;
		pMessage->buf[1] = bamocarDestReg;
		pMessage->buf[2] = TRANSMIT_ONCE;
	}
}

/*-----------------------------------------------------------------------------
 Populate CAN message frame (Bamocar read specific with transmission interval)
-----------------------------------------------------------------------------*/
void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, uint8_t bamocarDestReg,
    uint8_t transmissionInterval) {
	// Head of Message
	pMessage->flags.extended  =  PAR_EXTENDED;
	pMessage->flags.remote    =  PAR_REMOTE;
	pMessage->flags.overrun   =  PAR_OVERRUN;
	pMessage->flags.reserved  =  PAR_RESERVED;
	pMessage->id              =  ID;
	pMessage->len             =  DLC;

	// Bamocar read register request - See CAN Complete
	// Requesting data to be sent back every 100ms
	if (pMessage->len == PAR_RX_DLC) {
		pMessage->buf[0] = REG_READ;
		pMessage->buf[1] = bamocarDestReg;
		pMessage->buf[2] = transmissionInterval;
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
		// Motor temperature
		case (REG_MOTOR_TEMP):
			message.id = ID_TEMP;
			break;

		// Motor RPM
		case (REG_SPEED_ACTUAL):
			message.id = ID_SPEED;
			break;

		default:
			bMapped = false;
			break;
	}

	// Return true when the ID has been re-mapped
	return bMapped;
}

/*-----------------------------------------------------------------------------
 Send a CAN message (to any transmitting mailbox)
-----------------------------------------------------------------------------*/
void SendCANMessage(const CAN_message_t &message) {
	myCan.write(message);
	// DebugPrintln("MESSAGE SENT");
}

/*-----------------------------------------------------------------------------
 Send a CAN message to a specified transmitting mailbox
-----------------------------------------------------------------------------*/
void SendCANMessage(const CAN_message_t &message, const FLEXCAN_MAILBOX MB) {
	myCan.write(MB, message);
	// DebugPrintln("MESSAGE SENT");
}

/*-----------------------------------------------------------------------------
 Periodically vehicle status CAN messages
-----------------------------------------------------------------------------*/
void SendCANStatusMessages(uint8_t * errors, uint8_t * state) {
    static uint32_t timer = millis();
	CAN_message_t message;

	// Check if the timing period has passed to send CAN messages again
	if ( millis() - timer >= STATUS_MESSAGE_INTERVAL ) {
		// Send ECU fault errors to dashboard
		PopulateCANMessage(&message, ID_ERROR_CODE, PAR_ERROR_DLC, errors);
		SendCANMessage(message);

		// Send current vehicle state to dashboard
		PopulateCANMessage(&message, ID_CURRENT_STATE, PAR_STATE_DLC, state);
		SendCANMessage(message);

		// Reset timer
		timer = millis();
	}
}

/*-----------------------------------------------------------------------------
 Request data from Bamocar registers at a set periodic interval
-----------------------------------------------------------------------------*/
void RequestBamocarData() {
	CAN_message_t msgBamocarRequest;

	// Motor temperature
	PopulateCANMessage(&msgBamocarRequest, ID_CAN_MESSAGE_RX, PAR_RX_DLC, 
		REG_MOTOR_TEMP, TRANSMIT_100MS);
	SendCANMessage(msgBamocarRequest);

	// Bamocar temperature
	msgBamocarRequest.buf[1] = REG_BAMOCAR_TEMP;
	SendCANMessage(msgBamocarRequest);

	// Overall current
	msgBamocarRequest.buf[1] = REG_CURRENT;
	SendCANMessage(msgBamocarRequest);

	// Overall voltage
	msgBamocarRequest.buf[1] = REG_VOLTAGE;
	SendCANMessage(msgBamocarRequest);
	
	// Motor RPM
	msgBamocarRequest.buf[1] = REG_SPEED_ACTUAL;
	SendCANMessage(msgBamocarRequest);

	// Phase current
	msgBamocarRequest.buf[1] = REG_CURRENT_PHASE_1;
	SendCANMessage(msgBamocarRequest);
	msgBamocarRequest.buf[1] = REG_CURRENT_PHASE_2;
	SendCANMessage(msgBamocarRequest);
	msgBamocarRequest.buf[1] = REG_CURRENT_PHASE_3;
	SendCANMessage(msgBamocarRequest);

	DebugPrintln("REQUESTING BAMOCAR DATA...");
}
