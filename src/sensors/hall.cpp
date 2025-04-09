#include "sensors/hall.h"

/*-----------------------------------------------------------------------------
 Hall effect sensor pedal constructor
-----------------------------------------------------------------------------*/
hall::hall(const uint8_t pinValue) : 
    // Initialize a circular buffer and analog pin
    buffer( (size_t) BUFFER_SIZE ), 
    pin(pinValue, INPUT) 
{
    // Intialize all data to zero
    rawOutput = 0;
    normalizedRawOutput = 0;
    cookedOutput = 0;
    torqueRequest = 0;
    upper = 0;
    lower = 0;
}

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

/*-----------------------------------------------------------------------------
 Average the signal incoming from hall sensor (low pass filter)
-----------------------------------------------------------------------------*/
void hall::AverageSignal() {
    // Modify signal buffer by adding newest reading
    buffer.PushBuffer(rawOutput);

    // Calculate new average and up scale
    uint16_t average = buffer.GetTotal() / buffer.GetCount();
    uint16_t cookedSignal = (TWO_BYTES * average) / ADC_RESOLUTION;

    // Set hall sensor cooked output to the new averaged signal
    cookedOutput = cookedSignal;
}

/*----------------------------------------------------------------------------- 
 Update a hall sensor's data
-----------------------------------------------------------------------------*/
void hall::UpdatePedalData() {
    // Read the raw signal from input analog pin
    rawOutput = ReadPedal();

    // Average the raw signal
    AverageSignal();

    // Calculate the pedal percent request to set torque request
    torqueRequest = GetPercentRequest() * FIFTEEN_BITS;

    // Cap the torque request to maximum if over 100% pedal percent request
    if (torqueRequest > FIFTEEN_BITS) {
		torqueRequest = FIFTEEN_BITS;
	}
}

/*-----------------------------------------------------------------------------
 Check if a pedal is OOR - Returns true if it is OOR
-----------------------------------------------------------------------------*/
bool hall::CheckPedalOOR() {
    float request = GetPercentRequest();

    // Check the raw analog pin reading has not been shorted/opened
	// Check the percent request did not go beneath -10% or above 110% due to slipping
    bool bRawValueOOR = rawOutput < OOR_LOWER_BOUND || rawOutput >= OOR_UPPER_BOUND;
    bool bPercentOOR = request <= OOR_LOWER_PERCENT || request >= OOR_UPPER_PERCENT;

	return bRawValueOOR || bPercentOOR; 
}
