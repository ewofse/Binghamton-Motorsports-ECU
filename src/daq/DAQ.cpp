#include "daq/DAQ.h"

/*-----------------------------------------------------------------------------
 Count the number of instances of a character in a string
-----------------------------------------------------------------------------*/
size_t CountCharacter(const char * pString, const char character) {
	const char * pStringCopy = pString;
	size_t characterCount = 0;
	
	// Iterate through each character
	while (*pStringCopy) {
		// Check if current character matches
		if (*pStringCopy == character) {
			++characterCount;
		}

		// Increment string pointer
		++pStringCopy;
	}

	return characterCount;
}

/*-----------------------------------------------------------------------------
 Get each index of a character in a string
-----------------------------------------------------------------------------*/
size_t * CharacterIndex(const char * pString, const char character) {
	const char * pStringCopy = pString;

	// Count the number of characters
	size_t numCharacters = CountCharacter(pStringCopy, character);

	uint8_t stringIndex = 0;
	uint8_t bufferIndex = 0;

	size_t * pIndexBuffer = NULL;
	
	// Check number of characters is non-zero and input string is valid
	if (numCharacters) {
		// Initialize buffer in memory
		pIndexBuffer = static_cast<size_t *>( malloc( numCharacters * sizeof(size_t) ) );

		// Check buffer initialized correctly
		if (pIndexBuffer) {
			// Iterate through string until end of string or character count reached
			while (*pStringCopy && bufferIndex < numCharacters) {
				// Check if current character matches target
				if (*pStringCopy == character) {
					// Store index of target character into result
					pIndexBuffer[bufferIndex] = stringIndex;

					// Increment buffer index
					++bufferIndex;
				}
				
				// Increment the string pointer & index
				++pStringCopy;
				++stringIndex;
			}
		}
	}

	return pIndexBuffer;
}

/*-----------------------------------------------------------------------------
 Get a substring of a string
-----------------------------------------------------------------------------*/
char * Substring(const char * pString, const size_t start, const size_t end) {
	// Get the length of the strings
	size_t stringlength = strlen(pString);
	size_t substringLength = end - start + 1;

	char * pSubstring = NULL;
	
	// Check for valid string and start and end indicies
	if (start < stringlength && end < stringlength && start <= end) {
		// Initialize substring in memory
		pSubstring = static_cast<char *>( malloc(substringLength + 1) );

		// Check string initizlied correctly
		if (pSubstring) {
			// Copy substring into another memory location
			strncpy(pSubstring, pString + start, substringLength);
			pSubstring[substringLength] = '\0';	
		}
	}

	return pSubstring;
}

/*-----------------------------------------------------------------------------
 Split string data values to form an array of integers
-----------------------------------------------------------------------------*/
uint16_t * SplitIntegerString(const char * pString, const char delimiter, size_t &length) {
    // Get the number of integers
    size_t numIntegers = CountCharacter(pString, delimiter) + 1;

    // Get the indices of the delimiters to create substrings
    size_t * pDelimiters = CharacterIndex(pString, delimiter);

    // Initialize substring indices
    size_t start = 0;
    size_t end = (numIntegers - 1) ? pDelimiters[0] - 1 : strlen(pString) - 1;

    char * pSubstring = NULL;
    uint16_t * pBuffer = NULL;

	// Get length of buffer
	length = numIntegers;

    // Check for non-zero number of integers and a valid delimiter index buffer
    if (numIntegers && pDelimiters) {
        // Create buffer in memory
        pBuffer = static_cast<uint16_t *>( malloc( numIntegers * sizeof(uint16_t) ) );

        // Check for valid buffer
        if (pBuffer) {
            // Repeat for each integer in string
            for (size_t index = 0; index < numIntegers; ++index) {
                // Get the next substring
                pSubstring = Substring(pString, start, end);
                
                // Check for a valid substring
                if (pSubstring) {
                    // Convert number to integer (base 10)
                    pBuffer[index] = static_cast<uint16_t>( strtol(pSubstring, NULL, 10) );

                    // Free substring from memory after covnersion
                    free(pSubstring);
                } else {
                    // Set value to zero if read incorrectly
                    pBuffer[index] = 0;
                }

                // Update start and end substring indices
                start = end + 2;
                end = (index < numIntegers - 2) ? pDelimiters[index + 1] - 1 : strlen(pString) - 1;
            }
        }
    } else if (numIntegers == 1) {
        // Create buffer in memory
        pBuffer = static_cast<uint16_t *>( malloc( numIntegers * sizeof(uint16_t) ) );

        // Convert string to integer
        pBuffer[0] = static_cast<uint16_t>( strtol(pString, NULL, 10) );
    }

    // Free delimiter index buffer
    free(pDelimiters);

    return pBuffer;
}

/*-----------------------------------------------------------------------------
 Write a string of data to a data file
-----------------------------------------------------------------------------*/
void WriteDataToFile(const char * pFileName, const char * pString, bool bOverwrite) {
	File fDataFile;
	
	// Delete old data to overwrite file
	if ( bOverwrite && SD.exists(pFileName) ) {
		SD.remove(pFileName);
		DebugPrint("OLD FILE DELETED: "); DebugPrintln(pFileName);
	}

	// Open file for writing
	fDataFile = SD.open(pFileName, FILE_WRITE);

	// Check file exists and add data
	if (fDataFile) {
		fDataFile.println(pString);
		fDataFile.close();

		DebugPrint("DATA WRITTEN TO: "); DebugPrintln(pFileName);
	} else {
		DebugPrint("ERROR: OPENING "); DebugPrintln(pFileName);
	}
}

/*-----------------------------------------------------------------------------
 Write which errors occured to a data file
-----------------------------------------------------------------------------*/
void ErrorToSD(void) {
	constexpr uint8_t errorLength = 4;
	bool bBitHigh = false;

	char pBinary[errorLength + 1] = "";
	char pErrors[75] = "";

	// Iterate through each error bit from MSB to LSB
	for (int8_t currentBit = errorLength - 1; currentBit >= 0; --currentBit) {
		// Read the current bit
        bBitHigh = bitRead(IRQHandler::GetErrorBuffer(), currentBit);

		// Check if the current bit is high
		if (bBitHigh) {
			// Read which error occured
            switch (currentBit) {
				// Shutdown circuit opened
                case (ERROR_CODE_SHUTDOWN): 
                    strncat(pErrors, "SHUTDOWN ERROR ", 75 - strlen(pErrors) - 1);
                    break;
				
				// APPS sensors disagree
                case (ERROR_CODE_DISAGREE):
					strncat(pErrors, "APPS DISAGREE ERROR ", 75 - strlen(pErrors) - 1);
                    break;

				// APPS & BSE pressed
                case (ERROR_CODE_APPS_BSE):
					strncat(pErrors, "APPS BSE ERROR ", 75 - strlen(pErrors) - 1);
                    break;

				// Sensor(s) out of range
                case (ERROR_CODE_OOR):
					strncat(pErrors, "SENSOR OOR ERROR ", 75 - strlen(pErrors) - 1);
                    break;

                default: 
                    break;
            }
		}

		// Add each error bit to a string of data
		pBinary[currentBit] = (bBitHigh) ? '1' : '0';
	}

	// Add null terminator and new line characters
	pBinary[errorLength] = '\0';
	strncat(pErrors, "\n", 75 - strlen(pErrors) - 1);

	// Write errors to data file
    WriteDataToFile(FILE_ECU_FAULTS, pBinary, !OVERWRITE); 
    WriteDataToFile(FILE_ECU_FAULTS, pErrors, !OVERWRITE);
}

/*-----------------------------------------------------------------------------
 Initialize the SD card and handle any errors
-----------------------------------------------------------------------------*/
void SetupSD(void) {
	// Setup SD memory storage for pedal calibration & telemetry
    if ( !SD.begin(BUILTIN_SDCARD) ) {
		DebugErrorPrint("ERROR: SD CARD FAILED");

        // Enable fault LED to indicate error
        IRQHandler::EnableFaultLEDTimer();
        
        // Wait for 5 seconds to show error state
        delay(5000);
        
        // Trigger immediate system reset
        IRQHandler::ResetWDT();
    }
    
    DebugPrintln("SD MEMORY INITIALIZED");
}
