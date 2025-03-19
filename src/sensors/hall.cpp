#include "sensors/hall.h"

/*-----------------------------------------------------------------------------
 Hall effect sensor pedal constructor
-----------------------------------------------------------------------------*/
hall::hall(const uint8_t pinValue)
	// Initialize a circular buffer and analog pin
	: buffer( (size_t) ARRAY_SIZE ), pin(pinValue, INPUT) {}

/*-----------------------------------------------------------------------------
 Obtain the analog value of pedal
-----------------------------------------------------------------------------*/
uint16_t hall::ReadPedal() {
    // Read outputted voltage signal - 10-bit resolution
    return pin.ReadRawPinAnalog();
}

/*-----------------------------------------------------------------------------
 Obtain the percent of pedal pressed
-----------------------------------------------------------------------------*/
float hall::GetPercentRequest() {
    // Interpolate the analog value through a percent request
    return (float) (cookedOutput - lower) / (upper - lower);
}
