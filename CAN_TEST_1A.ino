// Compatible with CAN_TEST_1B
#include <stdint.h>

#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

const int baudRate = 500000;
const int ledPin = 13;
const int delayTime = 50;

int count = 0;

CAN_message_t message;

void setup() {
  pinMode(ledPin,OUTPUT);
  Serial.begin(38400);
  Can1.begin();  
  Can1.setBaudRate(baudRate);
  Serial.println("Setup Complete");
}

uint8_t loop_control = 0;

void loop() {
  message.id = 0x201;
  message.len = 1;
  message.buf[0] = 0x01;


  Can1.write(message);

  digitalWrite(ledPin,HIGH);
  delay(delayTime);
  digitalWrite(ledPin,LOW);
  delay(delayTime);

  Serial.println("Message Sent");
}
