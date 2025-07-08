/*-------------------------------------------------------------------------------------------------
 EV1 / EV1.5 ECU Master Program
 Programmers: Ethan Wofse, Jake Lin, Markus Higgins, Vansh Joishar
 Last Updated: 05.28.25
-------------------------------------------------------------------------------------------------*/
#include "core/ECU.h"

// Vehicle FSM & data
systemFSM vehicle;

/*-------------------------------------------------------------------------------------------------
 Setup
-------------------------------------------------------------------------------------------------*/
void setup() {
    // Connect serial comms for debugging
    DebugBegin(SERIAL_RATE);
    DebugPrintln("SERIAL COMMS INITIALIZED");

    // Setup WDT for potential software hangs
    IRQHandler::ConfigureWDT();

    // Setup the SD card for DAQ
    SetupSD();

    // SKIPPING DURING TEST BENCHING
	// Initialize CAN communications
    ConfigureCANBus();

    // SKIPPING DURING TEST BENCHING
    // Setup data read requests to Bamocar
    RequestBamocarData();

    // Set original interrupts
    SetupInterrupts();
}

/*-------------------------------------------------------------------------------------------------
 Main Loop
-------------------------------------------------------------------------------------------------*/
void loop() {
    // Get the data buffers sent to the dashboard
    uint8_t stateBuf = vehicle.GetSystemData().GetStateBuffer();
    uint8_t faultBuf = vehicle.GetSystemData().GetFaultBuffer();

    // Feed the WDT
    IRQHandler::FeedWDT();

    /*-----------------------------------------------------------------------------
     FSM Current State Processing
    -----------------------------------------------------------------------------*/
    vehicle.ProcessState();

	/*-----------------------------------------------------------------------------
     Telemetry & Status CAN Messages
    -----------------------------------------------------------------------------*/
	// SKIPPING DURING TEST BENCHING
    SendCANStatusMessages(&faultBuf, &stateBuf);
}	
