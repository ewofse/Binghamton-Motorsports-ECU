// Safe guards
#ifndef PIN_H
#define PIN_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "general.h"

/*------------------------------------------
 Macros - Pins
------------------------------------------*/
#ifdef EV1_5
    #define PIN_BSE              A16
    #define PIN_APPS_ONE         A15
    #define PIN_APPS_TWO         A14 

    #define PIN_RTD_BUTTON       9
    #define PIN_SHUTDOWN_TAP     8
    #define PIN_RESET            35
    #define PIN_RUN              13
    #define PIN_GO               17
    #define PIN_BRAKE_LIGHT      33
#elif defined(EV1)
    #define PIN_BSE              A10 // Left
    #define PIN_APPS_ONE         A13 // Second Right
    #define PIN_APPS_TWO         A12 // Right

    #define PIN_RTD_BUTTON       25  // Second Left
    #define PIN_SHUTDOWN_TAP     12
    #define PIN_RESET            18
    #define PIN_RUN              21
    #define PIN_GO               22
    #define PIN_BRAKE_LIGHT      19
#endif

// Model specific pins
#define PIN_LED_FAULT            0
#define PIN_PUMP			     34
#define PIN_AIR_PLUS             41

// General / shared pins
#define PIN_RTD_SOUND            20
#define PIN_GPIO0                0
#define PIN_GPIO1                1
#define PIN_GPIO2                2
#define PIN_GPIO3                3
#define PIN_GPIO4                4
#define PIN_GPIO5                5
#define PIN_GPIO6                6
#define PIN_GPIO7                7
#define PIN_LED                  13

/*------------------------------------------
 Macros - Vehicle-Specific Pin Functions
------------------------------------------*/
#ifdef EV1
	#define EV1_AIR_PLUS_HIGH() system.GetAIRPlusPin().WriteOutput(HIGH)
#else
	#define EV1_AIR_PLUS_HIGH()
#endif

#ifdef EV1_5
	#define EV1_5_TOGGLE_FAULT_LED() digitalWriteFast( PIN_LED_FAULT, !digitalReadFast(PIN_LED_FAULT) )
    #define EV1_5_PUMP_CONTROL()
#else
	#define EV1_5_TOGGLE_FAULT_LED()
    #define EV1_5_PUMP_CONTROL()
#endif

/*-------------------------------------------------------------------------------------------------
 Data Structures
-------------------------------------------------------------------------------------------------*/
// General GPIO pin object
class GPIO {
    public:
        // Constructor
        GPIO(const uint8_t pinValue, bool bPinMode);

        // Getters
        uint8_t GetPin() { return pin; }

        // Setters
        void SetPinMode(bool value) { pinMode(pin, value); }

    protected:
        uint8_t pin;
};

// Digital GPIO pin object
class digitalPin : public GPIO {
    public:
        // Constructors
        digitalPin(const uint8_t pinValue, uint16_t debounceTimeValue, bool bPinMode);
        digitalPin(const uint8_t pinValue, bool bPinMode);

        // Setters
        void SetDebounceTime(uint16_t value) { debounceTime = value; }

        // Data methods
        void WriteOutput(bool bValue) { digitalWriteFast(pin, bValue); }
        bool ReadRawPinDigital() { return digitalReadFast(pin); }
        bool ReadDebouncedPin();
        bool ReadPulsedPin(bool signal);
    
    private:
        uint32_t debounceTimer;
        uint16_t debounceTime;

        bool lastRawState;
        bool currentRawState;
        bool debounceOutput;
        bool lastDebounceOutput;
};

// Analog GPIO pin object
class analogPin : public GPIO {
    public:
        // Constructor
        analogPin(const uint8_t pinValue, bool bPinMode);

        // Data methods
        void SetOutput(uint16_t value) { analogWrite(pin, value); }
        uint16_t ReadRawPinAnalog() { return analogRead(pin); }
};

// End safe guards
#endif /* PIN_H */
