// Compatible with CAN_TEST_1A

#include <FlexCAN_T4.h>
#include <kinetis_flexcan.h>

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can1;

const int baudRate = 50000;
const int ledPin = 13;
const int delayTime = 1000;

int count = 0;
CAN_message_t message;

void setup() {
  pinMode(ledPin,OUTPUT);
  Can1.setBaudRate(baudRate);
  Serial.begin(9600);
  Can1.begin();
  Serial.println("Setup Complete B");
}

void loop() {
  if(Can1.read(message))
  {
    digitalWrite(ledPin,HIGH);
    delay(delayTime);
    digitalWrite(ledPin,LOW);
    delay(delayTime);

    Serial.println("Message Received");
    for (int i = 0; i < message.len; i++) {
      // Print each byte of the data payload as a hexadecimal value
      Serial.print(message.buf[i], HEX);
      Serial.print(" ");
    }
  }
  else {
    Serial.println("Message not read");
  }
}