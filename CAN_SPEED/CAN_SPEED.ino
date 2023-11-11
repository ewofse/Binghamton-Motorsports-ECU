#include <stdint.h>

#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;  // use pins 30 and 31 on teensy for Can Rx/Tx (Recieve/Transmit) because
                                                 // these are the pins capable of supporting CAN2

CAN_message_t TransmittingSpeedCommandValue;
uint16_t AnalogPedal023;
uint16_t AnalogPedal65536;

// Declare values for noise
int total = 0;
int average = 0;
int counter = 0;
int initialize = 0; 
const int size = 40;
int array[size];

void setup() {
  // Initializing CAN
  Can1.begin();
  Can1.setBaudRate(500000);

  for (int i = 0; i < size; i++) {
    array[i] = 0;
  }
}

void loop() {
  //______________________________________________________________________________
  //                         FETCHING PEDAL POSITION DATA
  //______________________________________________________________________________
  AnalogPedal023 = analogRead(23);  // Accelerator pedal output hooked to pin 23. 0-5V
  Serial.println(AnalogPedal023);
  
  if (AnalogPedal023 > 1023) {
    AnalogPedal023 = 1023;
  }

  // Average the first "size" amount of inputs incrementally
  while (initialize < size){
    total -= array[initialize]; 
    array[initialize] = AnalogPedal023;
    total += array[initialize];
    initialize++;
    average = total/initialize; 
  }

  // Average the analog read value to reduce noise
  total -= array[counter];

  array[counter] = AnalogPedal023;

  total += array[counter];

  counter++;

  if (counter >= size) {
    counter = 0;
  }

  average = total / size;

  short int temp = average;

  Serial.println("Average:");
  Serial.println(temp);

  AnalogPedal65536 = (65536 * temp) / 1023;  //map(AnalogPedal023, 0, 1023, 0, 65536); // Short int because our message length is 2 bytes
  Serial.println("10 bit value:");
  Serial.println(temp);
  Serial.println("2 byte value:");
  Serial.println(AnalogPedal65536);

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
  TransmittingSpeedCommandValue.flags.remote = 0;
  TransmittingSpeedCommandValue.flags.overrun = 0;
  TransmittingSpeedCommandValue.flags.reserved = 0;
  TransmittingSpeedCommandValue.id = 0x201;
  TransmittingSpeedCommandValue.len = 3;

  // Data Field

  int firstByte = AnalogPedal65536 & (0b0000000011111111);
  int secondByte = (AnalogPedal65536 & (0b1111111100000000)) >> 8;

  TransmittingSpeedCommandValue.buf[0] = 0x31;
  TransmittingSpeedCommandValue.buf[1] = firstByte;
  TransmittingSpeedCommandValue.buf[2] = secondByte;

  Serial.println("First byte of value:");
  Serial.println(TransmittingSpeedCommandValue.buf[1]);
  Serial.println("Second byte of value:");
  Serial.println(TransmittingSpeedCommandValue.buf[2]);

  // ______________________________________________________________________________
  //                                  SEND MESSAGES
  // ______________________________________________________________________________

  Can1.write(TransmittingSpeedCommandValue);
}