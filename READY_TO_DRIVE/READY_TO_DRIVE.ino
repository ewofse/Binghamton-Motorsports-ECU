// Include header file
#include "MotorController.h"

// Initialize CAN object
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1; 

// Initliaze CAN message object
CAN_message_t TransmittingSpeedCommandValue;

uint16_t AnalogPedalAccelerator1023;
uint16_t AnalogPedalAccelerator65535;

uint16_t AnalogPedalBrake1023;
uint16_t AnalogPedalBrake65535;

uint16_t startButton;

int total = 0;
int average = 0;
int counter = 0;
int initialize = 0;

short int readyToDrive = 0;

int array[SIZE];

// Setup
void setup() 
{
  // Initializing CAN
  Can1.begin();
  Can1.setBaudRate(500000);

  // Initialize average array
  InitializeArray(array, SIZE);

}

// Loop
void loop()
{
  // Check if car is ready to drive
  readyToDrive = ReadyToDrive();

  // Send messages to motor controller from pedals
  if(readyToDrive)
  {
    Pedal(AnalogPedalAccelerator1023, AnalogPedalAccelerator65535, ACCELERATOR_PIN);
    Pedal(AnalogPedalBrake1023, AnalogPedalBrake65535, BRAKE_PIN);
  }
}