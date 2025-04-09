/*-------------------------------------------------------------------------------------------------
 EV1 / EV1.5 ECU Master Program
 Programmers: Ethan Wofse, Jake Lin, Markus Higgins, Vansh Joishar
 Last Updated: 04.03.25
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

    // Setup SD memory storage for pedal calibration & telemetry
    // if ( !SD.begin(BUILTIN_SDCARD) ) {
	// 	DebugErrorPrint("ERROR: SD CARD FAILED");
    //     EXIT;
    // }
    
    DebugPrintln("SD MEMORY INITIALIZED");

    // Setup WDT for potential software hangs
    IRQHandler::ConfigureWDT();

    // SKIPPING DURING TEST BENCHING
	// Initialize CAN communications
    // ConfigureCANBus();

    // SKIPPING DURING TEST BENCHING
    // Setup data read requests to Bamocar
    // RequestBamocarData();

    // Set original interrupts
    SetupInterrupts();
}

/*-------------------------------------------------------------------------------------------------
 Main Loop
-------------------------------------------------------------------------------------------------*/
void loop() {
    bool bErrorsLast100ms;

    // Get the buffers
    // uint8_t stateBuf = vehicle.GetSystemData().GetStateBuffer();
    // uint8_t faultBuf = vehicle.GetSystemData().GetFaultBuffer();

    // Feed the WDT
    IRQHandler::FeedWDT();

    /*-----------------------------------------------------------------------------
     FSM Current State Processing
    -----------------------------------------------------------------------------*/
    vehicle.ProcessState();

    /*-----------------------------------------------------------------------------
     Pedal Processing
    -----------------------------------------------------------------------------*/
    vehicle.GetSystemData().UpdatePedalStructures();

    /*-----------------------------------------------------------------------------
     Brake Light Updates
    -----------------------------------------------------------------------------*/
    vehicle.GetSystemData().ActivateBrakeLight();

    /*-----------------------------------------------------------------------------
     Pump Control
    -----------------------------------------------------------------------------*/
    vehicle.GetSystemData().GetPumpController().TogglePump();

    /*-----------------------------------------------------------------------------
     Pedal Implausibility (Timer Check)
    -----------------------------------------------------------------------------*/
    bErrorsLast100ms = vehicle.GetSystemData().CheckPedalImplausibility();
    vehicle.GetSystemData().Set100msFlag(bErrorsLast100ms);

	/*-----------------------------------------------------------------------------
     Telemetry & Status CAN Messages
    -----------------------------------------------------------------------------*/
	// SKIPPING DURING TEST BENCHING
    // SendCANStatusMessages(&faultBuf, &stateBuf);
}	
