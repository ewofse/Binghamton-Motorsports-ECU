// Safe guards
#ifndef DAQ_H
#define DAQ_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "core/general.h"
#include "interrupts/interrupts.h"
#include "comms/CAN.h"

/*------------------------------------------
 Macros - Files
------------------------------------------*/
#define OVERWRITE            	 1
#define FILE_PEDAL_BOUNDS    	 "pedal_bounds.csv"
#define FILE_ECU_FAULTS          "fault.txt"
#define FILE_GENERAL_DATA        "general_data.txt"

/*-------------------------------------------------------------------------------------------------
 Prototypes
-------------------------------------------------------------------------------------------------*/
size_t CountCharacter(const char * pString, const char character);

size_t * CharacterIndex(const char * pString, const char character);

char * Substring(const char * pString, const size_t start, const size_t end);

uint16_t * SplitIntegerString(const char * pString, const char delimiter, size_t &length);

void WriteDataToFile(const char * pFileName, const char * pString, bool bOverwrite);

void ErrorToSD();

void CANDataToSD(const CAN_message_t &message);

// End safe gaurds
#endif /* DAQ_H */
