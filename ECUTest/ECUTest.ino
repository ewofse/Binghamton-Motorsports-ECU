#include <stdint.h>

#include <FlexCAN_T4.h>
#include <circular_buffer.h>
#include <imxrt_flexcan.h>
#include <isotp.h>
#include <isotp_server.h>
#include <kinetis_flexcan.h>

const int LEDPin = 13;
const int delayTime = 50;
uint16_t Input1;

//tx4, rx4(issue), rx3(issue), tx3(issue) (left to right) of J5

void setup() {
  pinMode(LEDPin, OUTPUT);
}

void loop() {
  Input1 = digitalRead(17);
  if (Input1 == HIGH) {
    digitalWrite(LEDPin, HIGH);
    delay(delayTime);
    digitalWrite(LEDPin, LOW);
    delay(delayTime);
  }
}
