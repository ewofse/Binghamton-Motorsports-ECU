// Safe guards
#ifndef HALL_H
#define HALL_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "core/general.h"
#include "core/pin.h"
#include "daq/DAQ.h"
#include "buffer.h"

/*------------------------------------------
 Macros - Hall Effect Sensor Percentages
------------------------------------------*/
#define PLAUSIBILITY_CHECK   5
#define PERCENT_THRESHOLD    10
#define PERCENT_BRAKE        20
#define PERCENT_ACCEL        25
#define APPS_AGREEMENT       10

/*------------------------------------------
 Macros - Other
------------------------------------------*/
#define NUM_SENSORS          3
#define OOR_LOWER_BOUND		 5
#define OOR_UPPER_BOUND		 1023
#define OOR_LOWER_PERCENT    -0.10
#define OOR_UPPER_PERCENT    1.10

/*-------------------------------------------------------------------------------------------------
 Hall Effect Processing
-------------------------------------------------------------------------------------------------*/
// Hall effect pedal sensor object
class hall {    
    public:
        // Constructor
        hall(const uint8_t pinValue);
        
        // Getters
        circularBuffer GetBuffer() { return buffer; }
        analogPin GetPin() { return pin; }

        uint16_t GetRawOutput() { return rawOutput; }
        uint16_t GetNormalizedRawOutput() { return normalizedRawOutput; }
        uint16_t GetCookedOutput() { return cookedOutput; }
        uint16_t GetTorqueRequest() { return torqueRequest; }

        uint16_t GetPercentRequestLowerBound() { return lower; }
        uint16_t GetPercentRequestUpperBound() { return upper; }

        // Setters
        void SetRawOutput(uint16_t value) { rawOutput = value; }
        void SetNormalizedRawOutput(uint16_t value) { normalizedRawOutput = value; }
        void SetCookedOutput(uint16_t value) { cookedOutput = value; }
        void SetTorqueRequest(uint16_t value) { torqueRequest = value; }

        void SetPercentRequestLowerBound(uint16_t value) { lower = value; }
        void SetPercentRequestUpperBound(uint16_t value) { upper = value; }

        // Data methods
        uint16_t ReadPedal();

        float GetPercentRequest();

        void AverageSignal();

        void UpdatePedalData();
        
        bool CheckPedalOOR();

    private:
        circularBuffer buffer; // Circular buffer for signal averaging
        analogPin pin; // Pedal sensor pin

        // Pedal supply, signal, and processed signal data
        uint16_t rawOutput;
        uint16_t normalizedRawOutput;
        uint16_t cookedOutput;
        uint16_t torqueRequest;

        // Pedal signal percent request bounds
        uint16_t lower;
        uint16_t upper;
};

// End safe guards
#endif /* HALL_H */
