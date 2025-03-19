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
    #define PIN_LED_FAULT        13
    #define PIN_REGEN            14
    #define PIN_CHARGE_ENABLE    11
	#define PIN_PUMP			 34
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
    #define PIN_AIR_PLUS         41
#endif

// #define PIN_VREF
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
    #define PIN_AIR_PLUS_INSTANTIATION digitalPin pinAIRPlus(PIN_AIR_PLUS, OUTPUT);
	#define EV1_AIR_PLUS_HIGH() pinAIRPlus.WriteOutput(HIGH);
#else
    #define PIN_AIR_PLUS_INSTANTIATION
	#define EV1_AIR_PLUS_HIGH()
#endif

#ifdef EV1_5
    #define PIN_PUMP_INSTANTIATION digitalPin pinPump(PIN_PUMP, OUTPUT);
    #define PIN_REGEN_INSTANTIATION digitalPin pinRegen(PIN_REGEN, OUTPUT);
    #define PIN_CHARGE_ENABLE_INSTANTIATION digitalPin pinChargeEnable(PIN_CHARGE_ENABLE, OUTPUT);
    #define PIN_LED_FAULT_INSTANTIATION digitalPin pinFaultLED(PIN_LED_FAULT, OUTPUT);

	#define EV1_5_TOGGLE_FAULT_LED() pinFaultLED.WriteOutput( !pinFaultLED.ReadRawPinDigital() );
	#define EV1_5_ACTIVATE_FAULT_LED() ActivateFaultLED();
	
    #define EV1_5_PUMP_CONTROL()
#else
    #define PIN_PUMP_INSTANTIATION
    #define PIN_REGEN_INSTANTIATION
    #define PIN_CHARGE_ENABLE_INSTANTIATION
    #define PIN_LED_FAULT_INSTANTIATION

	#define EV1_5_TOGGLE_FAULT_LED()
	#define EV1_5_ACTIVATE_FAULT_LED()

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
        uint16_t debounceTime;
        uint32_t debounceTimer;

        bool lastRawState;
        bool currentRawState;
        bool currentStableState;
        bool lastStableState;
};

// Analog GPIO pin object
class analogPin : public GPIO {
    public:
        // Constructor
        analogPin(const uint8_t pinValue, bool pPinMode);

        // Data methods
        void SetOutput(uint16_t value) { analogWrite(pin, value); }
        uint16_t ReadRawPinAnalog() { return analogRead(pin); }
};


/*-------------------------------------------------------------------------------------------------
 Prototypes
-------------------------------------------------------------------------------------------------*/
void ActivateBamocar(digitalPin pinRUN, digitalPin pinGO);

void DeactivateBamocar(digitalPin pinRUN, digitalPin pinGO);

void ActivateFaultLED();

// End safe guards
#endif /* PIN_H */
