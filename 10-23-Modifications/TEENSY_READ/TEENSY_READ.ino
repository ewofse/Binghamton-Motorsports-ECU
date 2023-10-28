#include <stdint.h>

#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

FlexCAN_T4<CAN2 , RX_SIZE_256, TX_SIZE_16> Can1; // use pins 30 and 31 on teensy for Can Rx/Tx (Recieve/Transmit) because
                                                 // these are the pins capable of supporting CAN2
CAN_message_t ReadingMessage; 

int counter = 0;


void setup() 
{
  // Initializing CAN
  Serial.begin(9600);
  Can1.begin();
  Can1.setBaudRate(500000);
  delay(500);

}



void loop() 
{
  
  if(Can1.read(ReadingMessage))
  {
    Serial.println("Start of Message");
    Serial.println(ReadingMessage.id);
    Serial.println(ReadingMessage.len);
    Serial.println(ReadingMessage.buf[0]);
    Serial.println(ReadingMessage.buf[1]);
    Serial.println(ReadingMessage.buf[2]);
    Serial.println("End of Message");
    Serial.println("");
    delay(500);
  }
  
  delay(100);
}
