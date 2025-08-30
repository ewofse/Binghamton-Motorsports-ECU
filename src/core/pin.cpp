#include "core/pin.h"

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
digitalPin::digitalPin(const uint8_t pinValue, uint16_t debounceTimeValue, bool bPinMode) : 
	GPIO {pinValue, bPinMode}
{
	debounceTime = debounceTimeValue;

	// Initialize pin value readings to low
	lastRawState = 0;
	currentRawState = 0;
	debounceOutput = 0;
	lastDebounceOutput = 0;
}

/*-----------------------------------------------------------------------------
 Digital pin constructor without debounce time
-----------------------------------------------------------------------------*/
digitalPin::digitalPin(const uint8_t pinValue, bool bPinMode) : 
	GPIO {pinValue, bPinMode} 
{
	// Initialize pin value readings to low
	lastRawState = 0;
	currentRawState = 0;
	debounceOutput = 0;
	lastDebounceOutput = 0;
}

/*-----------------------------------------------------------------------------
 Analog pin constructor
-----------------------------------------------------------------------------*/
analogPin::analogPin(const uint8_t pinValue, bool bPinMode) :
	// Initialize a GPIO pin with a specified pin value and mode
	GPIO {pinValue, bPinMode},
	buffer( (size_t) 0 ) {}

/*-----------------------------------------------------------------------------
 Analog pin constructor with a circular buffer
-----------------------------------------------------------------------------*/
analogPin::analogPin(const uint8_t pinValue, bool bPinMode, size_t size) :
	// Initialize a GPIO pin with a specified pin value and mode
	GPIO {pinValue, bPinMode},
	buffer(size) {}

/*-----------------------------------------------------------------------------
 Debounce the incoming signal into the pin
-----------------------------------------------------------------------------*/
bool digitalPin::ReadDebouncedPin(void) {
	// Read the current pin state
	currentRawState = ReadRawPinDigital();

	// Reset the timer if the input changes
	if (currentRawState != lastRawState) {
		debounceTimer = millis();
	}

	// Update stable state when debounce time elapses without input changing
	if ( millis() - debounceTimer >= debounceTime && debounceOutput != currentRawState ) {
		debounceOutput = currentRawState;
	}

	// Update the last raw reading
	lastRawState = currentRawState;

	return debounceOutput;
}

/*-----------------------------------------------------------------------------
 Have a signal enabled for one clock cycle (rising edge pulser)
-----------------------------------------------------------------------------*/
bool digitalPin::ReadPulsedPin(bool signal) {
	// Output is rising edge detection
	bool pulse = !lastDebounceOutput && signal;

	// Update the previous state
	lastDebounceOutput = signal;

	return pulse;
}
