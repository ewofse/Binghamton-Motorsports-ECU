// Safe guards
#ifndef HALL_H
#define HALL_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "general.h"
#include "pin.h"
#include "DAQ.h"

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
#define ARRAY_SIZE           6000
#define NUM_SENSORS          3
#define OOR_LOWER_BOUND		 5
#define OOR_UPPER_BOUND		 1023
#define OOR_LOWER_PERCENT    -0.10
#define OOR_UPPER_PERCENT    1.10
// #define NORMALIZED_OUTPUT

#ifdef NORMALIZED_OUTPUT
	#define UPDATE_APPS_BUFFER(APPS) APPS->buffer.PushBuffer( APPS->GetNormalizedRawOutput() );
#else
	#define UPDATE_APPS_BUFFER(APPS) APPS->buffer.PushBuffer( APPS->GetRawOutput() );
#endif


/*-------------------------------------------------------------------------------------------------
 Hall Effect Processing
-------------------------------------------------------------------------------------------------*/
// Circular buffer with a total sum
class circularBuffer {
    public:
        // Constructor
        circularBuffer(const size_t elements);

        // Getters
        size_t GetCapacity() { return capacity; }
        size_t GetCount() { return count; }
        uint32_t GetTotal() { return total; }

        // Data methods
        void FreeBuffer();
        void PushBuffer(uint16_t value);
        uint16_t PullBuffer();

    private:
        // Data buffer and pointers to current start and end of buffer
        uint16_t * pBuffer;
        uint16_t * pHead;
        uint16_t * pTail;
        uint32_t total;

        // Number of elements inside the buffer and maximum number of elements
        size_t count;
        size_t capacity;
};

// Hall effect pedal sensor object
class hall {    
    public:
        // Constructor
        circularBuffer buffer; // Circular buffer for signal averaging
		analogPin pin; // Pedal sensor pin

        hall(const uint8_t pinValue);
        
        // Getters
        uint16_t GetRawSupply() { return rawSupply; }
        uint16_t GetRawOutput() { return rawOutput; }
        uint16_t GetNormalizedRawOutput() { return normalizedRawOutput; }
        uint16_t GetCookedOutput() { return cookedOutput; }
        uint16_t GetTorqueRequest() { return torqueRequest; }

        uint16_t GetPercentRequestLowerBound() { return lower; }
        uint16_t GetPercentRequestUpperBound() { return upper; }

        // Setters
        void SetRawSupply(uint16_t value) { rawSupply = value; }
        void SetRawOutput(uint16_t value) { rawOutput = value; }
        void SetNormalizedRawOutput(uint16_t value) { normalizedRawOutput = value; }
        void SetCookedOutput(uint16_t value) { cookedOutput = value; }
        void SetTorqueRequest(uint16_t value) { torqueRequest = value; }

        void SetPercentRequestLowerBound(uint16_t value) { lower = value; }
        void SetPercentRequestUpperBound(uint16_t value) { upper = value; }

        // Data methods
        uint16_t ReadPedal();
        uint16_t ReadSupply();

        float GetPercentRequest();

    private:
        // Pedal supply, signal, and processed signal data
        static uint16_t rawSupply; // Static for shared data
        uint16_t rawOutput;
        uint16_t normalizedRawOutput;
        uint16_t cookedOutput;
        uint16_t torqueRequest;

        // Pedal signal percent request bounds
        uint16_t lower;
        uint16_t upper;
};

/*-------------------------------------------------------------------------------------------------
 Prototypes
-------------------------------------------------------------------------------------------------*/
bool ReadyToDrive(hall * pBSE, digitalPin pinBSE);

void ActivateBrakeLight(hall * pBSE, digitalPin pinBrakeLight);

void AverageSignal(hall * pAPPS);

void ProcessAPPS(hall * pAPPS1, hall * pAPPS2, uint8_t * pTorqueBuf);

void UpdatePedalData(hall * pSensor);

void UpdatePedalStructures(hall * pAPPS1, hall * pAPPS2, hall * pBSE);

float GetLowerPercentAPPS(hall * pAPPS1, hall * pAPPS2);

bool CheckAPPS(hall * pAPPS1, hall * pAPPS2);

bool CheckPedalOOR(hall * pSensor);

bool CheckPedalsOOR(hall * pAPPS1, hall * pAPPS2, hall * pBSE);

bool CheckPedalPlausibility(hall * pAPPS1, hall * pAPPS2, hall * pBSE);

bool CheckPedalImplausibility(hall * pAPPS1, hall * pAPPS2, hall * pBSE);

bool CheckAllErrors(hall * pAPPS1, hall * pAPPS2, hall * pBSE, bool b100msPassed);

bool ShutdownCircuitOpen();

bool PedalsDisagree();

bool BothPedalsPressed();

bool PedalsOOR();

bool SetPedalBounds(hall * pAPPS1, hall * pAPPS2, hall * pBSE);

void CalibratePedals(hall * pAPPS1, hall * pAPPS2, hall * pBSE, digitalPin pin);

bool DetectCalibrationStartup();

// End safe guards
#endif /* HALL_H */
