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
int AnalogPedal023;
short int AnalogPedal65536;

void setup() 
{
  // Initializing CAN
  Can1.begin();
  Can1.setBaudRate(500000);
}

void loop() 
{
  //______________________________________________________________________________
  //                         FETCHING PEDAL POSITION DATA
  //______________________________________________________________________________
   AnalogPedal023 = analogRead(23); // Accelerator pedal output hooked to pin 23. 0-5V
   AnalogPedal65536 = map(AnalogPedal023, 0, 1023, 0, 65536); // Short int because our message length is 2 bytes
   //Serial.println(AnalogPedal65536);

   // Notes
   // --------------------------------------
   /*
 
   Top Half:
   (AnalogPedal512 & (0b1111111100000000))>>8

   Bottom Half:
   AnalogPedal512 & (0b0000000011111111)

   Compare previous value - if within certain range keep it the same speed 
   
  */
  
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
    TransmittingSpeedCommandValue.buf[1] =  AnalogPedal65536 & (0b0000000011111111);
    TransmittingSpeedCommandValue.buf[2] = (AnalogPedal65536 & (0b1111111100000000)>>8);


  // ______________________________________________________________________________
  //                                  SEND MESSAGES
  // ______________________________________________________________________________
 
  Can1.write(TransmittingSpeedCommandValue);

}
