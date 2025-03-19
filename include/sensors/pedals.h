// Safe guards
#ifndef PEDALS_H
#define PEDALS_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "core/general.h"
#include "hall.h"

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
#endif /* PEDALS_H */
