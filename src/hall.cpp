#include "main.h"

/*-----------------------------------------------------------------------------
 Hall construtor
-----------------------------------------------------------------------------*/
hall::hall(const uint8_t pinValue) {
    total = 0;
    counter = 0;
    pin = pinValue;

    InitializeArray(array, ARRAY_SIZE);
}

/*-----------------------------------------------------------------------------
 Obtain the percent of pedal pressed
-----------------------------------------------------------------------------*/
float hall::GetPercentRequest(const uint16_t sensor[][2]) {
    // Interpolate the analog value through a percent request
    return (float) ( cookedOutput - sensor[MIN][VOLTAGE] ) / ( sensor[MAX][VOLTAGE] - sensor[MIN][VOLTAGE] );
}

/*-----------------------------------------------------------------------------
 Obtain the analog value of pedal
-----------------------------------------------------------------------------*/
uint16_t hall::ReadPedal() {
    // Read outputted voltage signal - 10-bit resolution
    return analogRead(pin);
}

/*-----------------------------------------------------------------------------
 Set Array elements to zero
-----------------------------------------------------------------------------*/
void hall::InitializeArray(uint16_t * pArray, const int size) {
    for (int i = 0; i < size; ++i) {
        pArray[i] = 0;
    }
}

/*-----------------------------------------------------------------------------
 Average the signal
-----------------------------------------------------------------------------*/
void AverageSignal(hall * pAPPS, const uint16_t sensor[][2]) {
    // Average the signal values to reduce noise
    pAPPS->ModifyTotal( pAPPS->GetArrayValue( pAPPS->GetCounter() ) * -1 );
    pAPPS->SetArrayValue( pAPPS->GetCounter(), pAPPS->GetRawOutput() );
    pAPPS->ModifyTotal( pAPPS->GetArrayValue( pAPPS->GetCounter() ) );
    pAPPS->SetCounter( pAPPS->GetCounter() + 1 );

    // Reset counter
    if ( pAPPS->GetCounter() == ARRAY_SIZE ) {
        pAPPS->SetCounter(0);
    }

    // Calculate new average and up convert
    uint16_t average = ( pAPPS->GetTotal() ) / ARRAY_SIZE;
    uint16_t cookedSignal = (TWO_BYTES * average) / TEN_BITS;

    // Set hall sensor cooked output to new averaged signal
    pAPPS->SetCookedOutput(cookedSignal);
}

/*----------------------------------------------------------------------------- 
 Use the previous functions to process incoming APPS data                                                        
-----------------------------------------------------------------------------*/
void ProcessAPPS(hall * pAPPS1, hall * pAPPS2, int8_t * pSpeedBuf) {
    // Fetch and average analog data
    uint16_t torqueAPPS1 = pAPPS1->GetTorqueRequest();
    uint16_t torqueAPPS2 = pAPPS2->GetTorqueRequest();

    // Obtain the lower signal
    uint16_t signal = (torqueAPPS1 < torqueAPPS2) ? torqueAPPS1: torqueAPPS2;

    // Set the data buffer equal to the processed signal
    if ( CheckAPPS(pAPPS1, pAPPS2) ) {
        *(pSpeedBuf + 2) = (signal & BYTE_ONE);
        *(pSpeedBuf + 1) = ( (signal & BYTE_TWO) >> 8 );
    }
}

/*----------------------------------------------------------------------------- 
 Update pedal data                                                   
-----------------------------------------------------------------------------*/
void UpdatePedalStructures(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
    // Update signal readings
    pAPPS1->SetRawOutput( pAPPS1->ReadPedal() );
    pAPPS2->SetRawOutput( pAPPS2->ReadPedal() );
    pBSE->SetRawOutput( pBSE->ReadPedal() );

    AverageSignal(pAPPS1, hallAPPS1);
    AverageSignal(pAPPS2, hallAPPS2);
    AverageSignal(pBSE, hallBSE);
    
    // Fetch torque request through pedal position
    uint16_t torqueAPPS1 = pAPPS1->GetPercentRequest(hallAPPS1) * FIFTEEN_BITS;
    uint16_t torqueAPPS2 = pAPPS2->GetPercentRequest(hallAPPS2) * FIFTEEN_BITS;
    uint16_t torqueBSE   = pBSE->GetPercentRequest(hallBSE) * FIFTEEN_BITS;

    pAPPS1->SetTorqueRequest(torqueAPPS1);
    pAPPS2->SetTorqueRequest(torqueAPPS2);
    pBSE->SetTorqueRequest(torqueBSE);
}

/*----------------------------------------------------------------------------- 
 Get the lower percent request
-----------------------------------------------------------------------------*/
float GetLowerPercentAPPS(hall * pAPPS1, hall * pAPPS2) {
    float APPS1 = pAPPS1->GetPercentRequest(hallAPPS1);
    float APPS2 = pAPPS2->GetPercentRequest(hallAPPS2);

    return (APPS1 < APPS2) ? APPS1 : APPS2;
}

/*-----------------------------------------------------------------------------
 Compare accelerator pedal positions - Returns true if signals agree
-----------------------------------------------------------------------------*/
bool CheckAPPS(hall * pAPPS1, hall * pAPPS2) {
    float APPS1 = pAPPS1->GetPercentRequest(hallAPPS1) * 100;
    float APPS2 = pAPPS2->GetPercentRequest(hallAPPS2) * 100;

    return ( abs(APPS1 - APPS2) <= APPS_AGREEMENT );
}

/*-----------------------------------------------------------------------------
 Check for pedals OOR - Returns true if signals are out of range
-----------------------------------------------------------------------------*/
bool CheckPedalsOOR(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
    return ( 
        ( pAPPS1->GetCookedOutput() < hallAPPS1[MIN][VOLTAGE] ) || \
        ( pAPPS1->GetCookedOutput() > hallAPPS1[MAX][VOLTAGE] ) || \
        ( pAPPS2->GetCookedOutput() < hallAPPS2[MIN][VOLTAGE] ) || \
        ( pAPPS2->GetCookedOutput() > hallAPPS2[MAX][VOLTAGE] ) || \
        ( pBSE->GetCookedOutput()   < hallBSE[MIN][VOLTAGE] )   || \
        ( pBSE->GetCookedOutput()   > hallBSE[MAX][VOLTAGE] ) 
    );
}

/*-----------------------------------------------------------------------------
 APPS BSE error check - Returns true if APPS & BSE are pressed
-----------------------------------------------------------------------------*/
bool AccelAndBrakePressed(hall * pAPPS1, hall * pAPPS2, hall * pBSE) {
    float APPS = GetLowerPercentAPPS(pAPPS1, pAPPS2) * 100;
    float BSE  = pBSE->GetPercentRequest(hallBSE) * 100;
    
    return (APPS > PERCENT_ACCEL && BSE > PERCENT_BRAKE);
}

/*-----------------------------------------------------------------------------
 Check All Errors - Returns true if any errors occur
-----------------------------------------------------------------------------*/
bool CheckAllErrors(hall * pAPPS1, hall * pAPPS2, hall * pBSE, int8_t * errorBuf, bool b100msPassed, int8_t * tempCode) {
    bool result = false;

    // Check if shutdown circuit is open
    if ( !digitalRead(PIN_SHUTDOWN_TAP) ) {
        *errorBuf |= (1 << 0);
        *tempCode = 1;
        result = true;

        PrintDebugMessage("ERROR 1");
    }
    
    // Check if pedal sensors disagree
    if ( !CheckAPPS(pAPPS1, pAPPS2) && b100msPassed ) {
        *errorBuf |= (1 << 1);
        *tempCode = 2;
        result = true;

        PrintDebugMessage("ERROR 2");
    }

    // Check if accelerator and brake are both pressed
    if ( AccelAndBrakePressed(pAPPS1, pAPPS2, pBSE) ) {
        *errorBuf |= (1 << 2);
        *tempCode = 3;
        result = true;

        PrintDebugMessage("ERROR 3");
    }

    // Check if any of the three sensors are out of range
    if ( CheckPedalsOOR(pAPPS1, pAPPS2, pBSE) && b100msPassed ) {
        *errorBuf |= (1 << 3);
        *tempCode = 4;
        result = true;

        PrintDebugMessage("ERROR 4");
    }

    return result;
}