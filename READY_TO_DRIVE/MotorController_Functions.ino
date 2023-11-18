// Include header file
#include "MotorController.h"

// Set array values to 0
void InitializeArray(int array[], int size)
{
  for(int i = 0; i < size; i++)
  {
    array[i] = 0;
  }
}

// Read signals from pedals
void Pedal(uint16_t pedal_10, uint16_t pedal_16, int pin)
{
  short int temp;

  // Average the first "size" amount of inputs incrementally
  while(initialize < SIZE)
  {
    // Read in pedal analog signal from Teensy pin
    pedal_10 = analogRead(pin);
    
    // Set signal to max value if read is too large
    if(pedal_10 > 1023) 
    {
      pedal_10 = 1023;
    }

    // Average the value
    total -= array[initialize];
    array[initialize] = pedal_10;
    total += array[initialize];
    initialize++;
    average = total/initialize;
  }

  // Read in pedal analog signal from Teensy pin
  pedal_10 = analogRead(pin);
  
  // Set signal to max value if read is too large
  if(pedal_10 > 1023) 
  {
    pedal_10 = 1023;
  }

  // Average the analog read value to reduce noise
  total -= array[counter];
  array[counter] = pedal_10;
  total += array[counter];
  counter++;

  if(counter >= SIZE) 
  {
    counter = 0;
  }

  average = total / SIZE;
  temp = average;

  Serial.println("Average:");
  Serial.println(temp);

  pedal_16 = (65536 * temp) / 1023;

  Serial.println("10 bit value:");
  Serial.println(temp);
  Serial.println("2 byte value:");
  Serial.println(pedal_16);
  Serial.println("");

  // Head of Message
  TransmittingSpeedCommandValue.flags.extended = 0;
  TransmittingSpeedCommandValue.flags.remote = 0;
  TransmittingSpeedCommandValue.flags.overrun = 0;
  TransmittingSpeedCommandValue.flags.reserved = 0;
  TransmittingSpeedCommandValue.id = 0x201;
  TransmittingSpeedCommandValue.len = 3;

  // Data Field
  int firstByte = pedal_16 & (0b0000000011111111);
  int secondByte = (pedal_16 & (0b1111111100000000)) >> 8;

  TransmittingSpeedCommandValue.buf[0] = 0x31;
  TransmittingSpeedCommandValue.buf[1] = firstByte;
  TransmittingSpeedCommandValue.buf[2] = secondByte;

  // Serial.println("First byte of value:");
  // Serial.println(TransmittingSpeedCommandValue.buf[1]);
  // Serial.println("Second byte of value:");
  // Serial.println(TransmittingSpeedCommandValue.buf[2]);

  // Send message
  Can1.write(TransmittingSpeedCommandValue);
}

int ReadyToDrive(int * readyToDrive)
{
  // If ready to drive was already initiated return true
  if(*readyToDrive == 1)
  {
    Serial.println("Vehicle is already ready to drive.");
    return 1;
  }

  // Read start button signal
  startButton = digitalRead(BUTTON_PIN);

  // Read in brake pedal signal
  AnalogPedalBrake65535 = analogRead(BRAKE_PIN);

  // Set signal to max value if read is too large
  if(AnalogPedalBrake65535 > 1023) 
  {
    AnalogPedalBrake65535 = 1023;
  }

  // Check if the start button is pushed and brake is pressed
  // Brake should be pressed by a noticable amount
  if(startButton == HIGH && AnalogPedalBrake65535 > 0) //(int)(65535 * 1 / 8))
  {
    *readyToDrive = 1;
    Serial.println("Vehicle is ready to drive.");
    return 1;
  }
}