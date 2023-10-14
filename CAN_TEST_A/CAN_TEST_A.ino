// Compatible with CAN_TEST_1B
 

#include <FlexCAN_T4.h>
#include <kinetis_flexcan.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

const int baudRate = 50000;
const int ledPin = 13;
const int delayTime = 100;

int count = 0;

CAN_message_t message;

void setup() {
  pinMode(ledPin,OUTPUT);
  Can1.setBaudRate(baudRate);
  Serial.begin(9600);
  Can1.begin();  
  Serial.println("Setup Complete");
}

void loop() {
  message.id = 0x123;
  message.len = 2;
  message.buf[0] = 0x01;
  Can1.write(message);
  Serial.println("Message Sent");

}
