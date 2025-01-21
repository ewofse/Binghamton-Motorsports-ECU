#include "main.h"

// Intialize raw supply for all sensors
// uint16_t hall::rawSupply = 0;

/*-----------------------------------------------------------------------------
 Circular buffer construtor
-----------------------------------------------------------------------------*/
circularBuffer::circularBuffer(const size_t elements) {
    // Declare memory for buffer and set head and tail start and end
    pBuffer = (uint16_t *) calloc( elements, sizeof(uint16_t) );
    pHead = pBuffer + elements - 1;
    pTail = pBuffer;

    // Initialize sum of buffer & number of elements
    total = 0;
    count = 0;
    capacity = elements;
}

/*-----------------------------------------------------------------------------
 Add element to circular buffer
-----------------------------------------------------------------------------*/
void circularBuffer::PushBuffer(uint16_t value) {
    // Subtract old value from total sum
    total -= *pHead;

    // Store inputted data into current buffer head
    *pHead = value;

    // Add new value to total sum
    total += *pHead;

    // Wrap around when reaching the end of buffer
    if (pHead == pBuffer + capacity) {
        pHead = pBuffer;
    }

    // Advance the head to the next element
    ++pHead;

    // Update current element counter and tail
    if (count == capacity) {
        // Advance the tail to the next element
        ++pTail;

        // Wrap around when reaching the end of buffer
        if (pTail == pBuffer + capacity) {
            pTail = pBuffer;
        }
    } else {
        ++count;
    }
}

/*-----------------------------------------------------------------------------
 Obtain an element from circular buffer
-----------------------------------------------------------------------------*/
uint16_t circularBuffer::PullBuffer() {
    // Return the value at the head of buffer if the head is valid
    return (pHead >= pBuffer) && (pHead < pBuffer + count - 1) ? *pHead : 0;
}

/*-----------------------------------------------------------------------------
 Free all allocated memory from object
-----------------------------------------------------------------------------*/
void circularBuffer::FreeBuffer() {
    // Free memory (this includes head & tail)
    free(pBuffer);

    // Set pointer to null to prevent dangling pointer
    pBuffer = nullptr;
}

/*-----------------------------------------------------------------------------
 Obtain the analog value of pedal
-----------------------------------------------------------------------------*/
uint16_t hall::ReadPedal() {
    // Read outputted voltage signal - 10-bit resolution
    return analogRead(pin);
}

/*-----------------------------------------------------------------------------
 Read the voltage of the 12 LV battery (for a normalized spply)
-----------------------------------------------------------------------------*/
// uint16_t hall::ReadSupply() {
//     // Read outputted voltage signal - 10-bit resolution
//     return analogRead(PIN_VREF);
// }

/*-----------------------------------------------------------------------------
 Obtain the percent of pedal pressed
-----------------------------------------------------------------------------*/
float hall::GetPercentRequest() {
    // Interpolate the analog value through a percent request
    return (float) (cookedOutput - lower) / (upper - lower);
}

/*-----------------------------------------------------------------------------
 Average the signal
-----------------------------------------------------------------------------*/
void AverageSignal(hall * pAPPS) {
    // Use normalized output if functionality exists on ECU
    #ifdef NORMALIZED_OUTPUT
        pAPPS->buffer.PushBuffer( pAPPS->GetNormalizedRawOutput() );
    #else
        pAPPS->buffer.PushBuffer( pAPPS->GetRawOutput() );
    #endif

    // Calculate new average and up scale
    uint16_t average = pAPPS->buffer.GetTotal() / pAPPS->buffer.GetCount();
    uint16_t cookedSignal = (TWO_BYTES * average) / ADC_RESOLUTION;

    // Set hall sensor cooked output to new averaged signal
    pAPPS->SetCookedOutput(cookedSignal);
}

/*----------------------------------------------------------------------------- 
 Use the previous functions to process incoming APPS data                                                        
-----------------------------------------------------------------------------*/
void ProcessAPPS(hall * pAPPS1, hall * pAPPS2, uint8_t * pTorqueBuf) {
    // Fetch and average analog data
    uint16_t torqueAPPS1 = pAPPS1->GetTorqueRequest();
    uint16_t torqueAPPS2 = pAPPS2->GetTorqueRequest();

    // Obtain the lower signal
    uint16_t signal = (torqueAPPS1 < torqueAPPS2) ? torqueAPPS1: torqueAPPS2;

    // Set the data buffer equal to the processed signal
    if ( CheckAPPS(pAPPS1, pAPPS2) ) {
        pTorqueBuf[2] = signal & BYTE_ONE;
        pTorqueBuf[1] = (signal & BYTE_TWO) >> 8;
    }
}

/*----------------------------------------------------------------------------- 
 Update a hall sensor's data
-----------------------------------------------------------------------------*/
void UpdatePedalData(hall * pSensor) {
    // Read the raw signal from input analog pin
    pSensor->SetRawOutput( pSensor->ReadPedal() );

    // Calculate the normalized analog pin voltage
    // float ratio = (float) ( pSensor->GetRawOutput() / pSensor->GetRawSupply() );
    // uint16_t normalized = (uint16_t) ( ratio * ADC_RESOLUTION );

    // Set the normalized raw output
    // pSensor->SetNormalizedRawOutput(normalized);

    // Average normalized signal
    AverageSignal(pSensor);

    // Calculate the pedal percent request
    uint16_t torque = pSensor->GetPercentRequest() * FIFTEEN_BITS;

    // Set the torque percent request
    pSensor->SetTorqueRequest(torque);
}

/*----------------------------------------------------------------------------- 
 Update pedal data for all hall sensors
-----------------------------------------------------------------------------*/
void UpdatePedalStructures(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
    // Update reference voltage for all sensors
    // pAPPS1->SetRawSupply( pAPPS1->ReadSupply() );

    // Update pedal data for each hall sensor
    UpdatePedalData(pAPPS1);
    UpdatePedalData(pAPPS2);
    UpdatePedalData(pBSE);
}

/*----------------------------------------------------------------------------- 
 Get the lower percent request
-----------------------------------------------------------------------------*/
float GetLowerPercentAPPS(hall * pAPPS1, hall * pAPPS2) {
    float APPS1 = pAPPS1->GetPercentRequest();
    float APPS2 = pAPPS2->GetPercentRequest();

    // Return the lower percent request
    return (APPS1 < APPS2) ? APPS1 : APPS2;
}

/*-----------------------------------------------------------------------------
 Compare accelerator pedal positions - Returns true if signals agree
-----------------------------------------------------------------------------*/
bool CheckAPPS(hall * pAPPS1, hall * pAPPS2) {
    float APPS1 = pAPPS1->GetPercentRequest() * 100;
    float APPS2 = pAPPS2->GetPercentRequest() * 100;

    // Check if pedals disagree
    return abs(APPS1 - APPS2) <= APPS_AGREEMENT;
}

/*-----------------------------------------------------------------------------
 Check for pedals OOR - Returns true if any signals are out of range
-----------------------------------------------------------------------------*/
bool CheckPedalsOOR(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
    // Check if pedals are out of range
    return ( 
        ( pAPPS1->GetCookedOutput() < pAPPS1->GetPercentRequestLowerBound() ) || \
        ( pAPPS1->GetCookedOutput() > pAPPS1->GetPercentRequestUpperBound() ) || \
        ( pAPPS2->GetCookedOutput() < pAPPS2->GetPercentRequestLowerBound() ) || \
        ( pAPPS2->GetCookedOutput() > pAPPS2->GetPercentRequestUpperBound() ) || \
        ( pBSE->GetCookedOutput()   < pBSE->GetPercentRequestLowerBound() )   || \
        ( pBSE->GetCookedOutput()   > pBSE->GetPercentRequestUpperBound() ) 
    );
}

/*-----------------------------------------------------------------------------
 APPS BSE error check - Returns true if APPS & BSE are pressed (25% & 10%)
-----------------------------------------------------------------------------*/
bool CheckPedalPlausibility(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
    float APPS = GetLowerPercentAPPS(pAPPS1, pAPPS2) * 100;
    float BSE  = pBSE->GetPercentRequest() * 100;
    
    // Check if both pedals are pressed
    return APPS > PERCENT_ACCEL && BSE > PERCENT_BRAKE;
}

/*-----------------------------------------------------------------------------
 Check OOR & signal agreement - Returns true if error lasts for over 100 ms
-----------------------------------------------------------------------------*/
bool CheckPedalImplausibility(hall * pAPPS1, hall * pAPPS2, hall * pBSE, elapsedMillis * pPedalErrorTimer) {
    static bool bPedalError = false;
    bool bResult = false;

    // Check if APPS signals disagree or any signals are out of range
    if ( !CheckAPPS(pAPPS1, pAPPS2) || CheckPedalsOOR(pAPPS1, pAPPS2, pBSE) ) {
        // Check the error duration
        if (!bPedalError) {
            // Start millisecond timer
            *pPedalErrorTimer = 0;
            bPedalError = true;
        } else if (*pPedalErrorTimer > IMPLAUSIBILITY_TIME) {
            DebugPrint("PEDAL ERROR TIMER: "); DebugPrintln(*pPedalErrorTimer);

            bResult = true;
            bPedalError = false;
        }
    } else {
        // Return false if either/both errors resolve in less than 100 ms
        bPedalError = false;
        bResult = false;
    }

    return bResult;
}

/*-----------------------------------------------------------------------------
 Check All Errors - Returns true if any errors occur
-----------------------------------------------------------------------------*/
bool CheckAllErrors(hall * pAPPS1, hall * pAPPS2, hall * pBSE, bool b100msPassed) {
    bool bResult = false;

    // Check if pedal sensors disagree
    if ( b100msPassed && !CheckAPPS(pAPPS1, pAPPS2) ) {
        errorBuf |= (1 << ERROR_CODE_DISAGREE);
        bResult = true;

        DebugPrintln("ERROR: APPS DISAGREE");
    }

    // Check if accelerator and brake are both pressed
    if ( CheckPedalPlausibility(pAPPS1, pAPPS2, pBSE) ) {
        errorBuf |= (1 << ERROR_CODE_APPS_BSE);
        bResult = true;

        DebugPrintln("ERROR: APPS & BSE PRESSED");
    }

    // Check if any of the three sensors are out of range
    if ( b100msPassed && CheckPedalsOOR(pAPPS1, pAPPS2, pBSE) ) {
        errorBuf|= (1 << ERROR_CODE_OOR);
        bResult = true;

        DebugPrintln("ERROR: SENSOR(S) OUT OF RANGE");
    }

    return bResult;
}

/*-----------------------------------------------------------------------------
 Evalues bit zero of error buffer (Shutdown circuit error)
-----------------------------------------------------------------------------*/
bool ShutdownCircuitOpen() {
    // Check if shutdown circuit open
    return errorBuf & (1 << ERROR_CODE_SHUTDOWN);
}

/*-----------------------------------------------------------------------------
 Evalues bit one of error buffer (APPS signal agreement error)
-----------------------------------------------------------------------------*/
bool PedalsDisagree() {
    // Check if pedals disagree
    return errorBuf & (1 << ERROR_CODE_DISAGREE);
}

/*-----------------------------------------------------------------------------
 Evalues bit three of error buffer (Both pedals pressed error)
-----------------------------------------------------------------------------*/
bool BothPedalsPressed() {
    // Check if both pedals are pressed
    return errorBuf & (1 << ERROR_CODE_APPS_BSE);
}

/*-----------------------------------------------------------------------------
 Evalues bit four of error buffer (Pedals out of range error)
-----------------------------------------------------------------------------*/
bool PedalsOOR() {
    // Check if pedals are out of range
    return errorBuf & (1 << ERROR_CODE_OOR);
}
