// Safe guards
#ifndef HALL_H
#define HALL_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include <stdint.h>

#include "core/general.h"
#include "core/pin.h"
#include "sensors/buffer.h"

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
#define OOR_LOWER_BOUND		 320
#define OOR_UPPER_BOUND		 TWO_BYTES
#define OOR_LOWER_PERCENT    -0.10
#define OOR_UPPER_PERCENT    1.10
#define MAX_TORQUE_REQUEST   6100
#define HALL_BUFFER_SIZE     5000

/*-------------------------------------------------------------------------------------------------
 Hall Effect Processing
-------------------------------------------------------------------------------------------------*/
// Hall effect pedal sensor object
class hall {    
    public:
        // Constructor
        hall(const uint8_t pinValue, const bool bInverted);
        
        // Getters
        circularBuffer GetBuffer(void) { return buffer; }
        analogPin GetPin(void) { return pin; }

        uint16_t GetRawOutput(void) { return rawOutput; }
        uint16_t GetNormalizedRawOutput(void) { return normalizedRawOutput; }
        uint16_t GetCookedOutput(void) { return cookedOutput; }
        uint16_t GetTorqueRequest(void) { return torqueRequest; }

        uint16_t GetPercentRequestLowerBound(void) { return lower; }
        uint16_t GetPercentRequestUpperBound(void) { return upper; }

        bool GetVoltageInverted(void) { return bVoltageInverted; }

        // Setters
        void SetRawOutput(uint16_t value) { rawOutput = value; }
        void SetNormalizedRawOutput(uint16_t value) { normalizedRawOutput = value; }
        void SetCookedOutput(uint16_t value) { cookedOutput = value; }
        void SetTorqueRequest(uint16_t value) { torqueRequest = value; }

        void SetPercentRequestLowerBound(uint16_t value) { lower = value; }
        void SetPercentRequestUpperBound(uint16_t value) { upper = value; }

        // Data methods
        uint16_t ReadPedal(void);

        float GetPercentRequest(void);

        void AverageSignal(void);

        void UpdatePedalData(void);
        
        bool CheckPedalOOR(void);

    private:
        circularBuffer buffer; // Circular buffer for signal averaging
        analogPin pin; // Pedal sensor pin

        // Pedal supply, signal, and processed signal data
        uint16_t rawOutput;
        uint16_t normalizedRawOutput;
        uint16_t cookedOutput;
        uint16_t torqueRequest;

        // Pedal signal percent request bounds (raw values)
        uint16_t lower;
        uint16_t upper;

        bool bVoltageInverted;
};

// End safe guards
#endif /* HALL_H */
