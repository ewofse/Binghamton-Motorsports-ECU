#include <stdint.h>

#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

FlexCAN_T4<CAN2 , RX_SIZE_256, TX_SIZE_16> Can1; // use pins 30 and 31 on teensy for Can Rx/Tx (Recieve/Transmit) because
                                                // these are the pins capable of supporting CAN2

CAN_message_t TransmittingSpeedCommandValue;

void setup() 
{
  // Initializing CAN
  Can1.begin();
  Can1.setBaudRate(500000);
  
}

void loop() 
{

  // Transmitting Speed Command Value
  // ---------------------------------------

    // Head of Message
    TransmittingSpeedCommandValue.flags.extended = 0; 
    TransmittingSpeedCommandValue.flags.remote   = 0;
    TransmittingSpeedCommandValue.flags.overrun  = 0;
    TransmittingSpeedCommandValue.flags.reserved = 0;
    TransmittingSpeedCommandValue.id = 0x201; 
    TransmittingSpeedCommandValue.len = 3;
      
    // Data Field
    TransmittingSpeedCommandValue.buf[0] = 0x31;
    TransmittingSpeedCommandValue.buf[1] = 0x00; // These values combine to give 1000 in little endian format
    TransmittingSpeedCommandValue.buf[2] = 0x00; // 


  // ______________________________________________________________________________
  //                                  SEND MESSAGES
  // ______________________________________________________________________________
 
  Can1.write(TransmittingSpeedCommandValue);

}
