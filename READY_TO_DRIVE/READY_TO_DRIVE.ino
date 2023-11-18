// Include header file
#include "MotorController.h"

// Initialize CAN object
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1; 

// Initliaze CAN message object
CAN_message_t TransmittingSpeedCommandValue;

// Accelerator pedal signal variables
uint16_t AnalogPedalAccelerator1023;
uint16_t AnalogPedalAccelerator65535;

// Brake pedal signal variables
uint16_t AnalogPedalBrake1023;
uint16_t AnalogPedalBrake65535;

// Start button signal
uint16_t startButton;

// Variables for noise reduction
int total = 0;
int average = 0;
int counter = 0;
int initialize = 0;

// Ready to drive condition
int readyToDrive = 0;

// Array for pedal reads
int array[SIZE];

// Setup
void setup() 
{
  // Initializing CAN
  Can1.begin();
  Can1.setBaudRate(500000);

  // Set start button pin as input
  pinMode(BUTTON_PIN, INPUT);

  // Initialize average array
  InitializeArray(array, SIZE);
}

// Loop
void loop()
{
  // Check if car is ready to drive
  readyToDrive = ReadyToDrive(&readyToDrive);

  // Send messages to motor controller from pedals
  if(readyToDrive == 1)
  {
    // Pedal(AnalogPedalAccelerator1023, AnalogPedalAccelerator65535, ACCELERATOR_PIN);
    Pedal(AnalogPedalBrake1023, AnalogPedalBrake65535, BRAKE_PIN);
  }
}