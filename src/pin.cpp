#include "pin.h"

/*-----------------------------------------------------------------------------
 GPIO constructor
-----------------------------------------------------------------------------*/
GPIO::GPIO(const uint8_t pinValue, bool bPinMode) {
	// Assign pin value
	pin = pinValue;

	// Declare pin as input or output
	SetPinMode(bPinMode);
}

/*-----------------------------------------------------------------------------
 Digital pin constructor with a debounce time
-----------------------------------------------------------------------------*/
digitalPin::digitalPin(const uint8_t pinValue, uint16_t debounceTimeValue, bool bPinMode)
	: GPIO {pinValue, bPinMode} {
		debounceTime = debounceTimeValue;

		// Initialize pin value readings to low
		lastRawState = 0;
		currentRawState = 0;
		currentStableState = 0;
		lastStableState = 0;
}

/*-----------------------------------------------------------------------------
 Digital pin constructor without debounce time
-----------------------------------------------------------------------------*/
digitalPin::digitalPin(const uint8_t pinValue, bool bPinMode)
	: GPIO {pinValue, bPinMode} {
		// Initialize pin value readings to low
		lastRawState = 0;
		currentRawState = 0;
		currentStableState = 0;
		lastStableState = 0;
	}

/*-----------------------------------------------------------------------------
 Analog pin constructor
-----------------------------------------------------------------------------*/
analogPin::analogPin(const uint8_t pinValue, bool bPinMode)
	// Initialize a GPIO pin with a specified pin value and mode
	: GPIO {pinValue, bPinMode} {}

/*-----------------------------------------------------------------------------
 Debounce the incoming signal into the pin
-----------------------------------------------------------------------------*/
bool digitalPin::ReadDebouncedPin() {
	// Read the current pin state
	currentRawState = ReadRawPinDigital();

	// Reset the timer if the input changes
	if (currentRawState != lastRawState) {
		debounceTimer = millis();
	}

	// Update stable state when debounce time elapses without input changing
	if ( millis() - debounceTimer >= debounceTime && currentStableState != currentRawState ) {
		currentStableState = currentRawState;
	}

	// Update the last raw reading
	lastRawState = currentRawState;

	return currentStableState;
}

/*-----------------------------------------------------------------------------
 Have a signal enabled for one clock cycle (rising edge pulser)
-----------------------------------------------------------------------------*/
bool digitalPin::ReadPulsedPin(bool signal) {
	// Output is rising edge detection
	bool pulse = !lastStableState && signal;

	// Update the previous state
	lastStableState = signal;

	return pulse;
}

/*-----------------------------------------------------------------------------
 Activate motor controller
-----------------------------------------------------------------------------*/
void ActivateBamocar(digitalPin pinRUN, digitalPin pinGO) {
	// Digital parameters required by the motor controller to drive
	pinRUN.WriteOutput(HIGH);
	pinGO.WriteOutput(HIGH);
}

/*-----------------------------------------------------------------------------
 Deactivate motor controller
-----------------------------------------------------------------------------*/
void DeactivateBamocar(digitalPin pinRUN, digitalPin pinGO) {
	// Digital parameters required by the motor controller to drive
	pinRUN.WriteOutput(LOW);
	pinGO.WriteOutput(LOW);
}

/*-----------------------------------------------------------------------------
 Blinking fault indicator LED on ECU PCB
-----------------------------------------------------------------------------*/
void ActivateFaultLED() {
	static uint32_t timer = millis();

	// Toggle LED every 100 milliseconds
	if ( millis() - timer > 100 ) {
		// Reset timer
		timer = millis();

		EV1_5_TOGGLE_FAULT_LED();
	}
}
