// Safe guards
#ifndef ECU_H
#define ECU_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>
#include <FlexCAN_T4.h>
#include <Arduino.h>

/*-------------------------------------------------------------------------------------------------
 Initializations for CAN Communication
-------------------------------------------------------------------------------------------------*/
extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;

/*------------------------------------------
 Macros - Pins
------------------------------------------*/
#define PIN_BRAKE            A11 // Left
#define PIN_RTD_BUTTON       28  // Second Left
#define PIN_ACCELERATOR_ONE  A13 // Second Right
#define PIN_ACCELERATOR_TWO  A12 // Right
#define PIN_SHUTDOWN_TAP     12

#define PIN_RESET            18
#define PIN_RTD_SOUND        20
#define PIN_RUN              21
#define PIN_GO               22
#define PIN_BRAKE_LIGHT      19
#define PIN_DISCHARGE        40
#define PIN_AIR_PLUS         41

#define PIN_GPIO0            0
#define PIN_GPIO1            1
#define PIN_GPIO2            2
#define PIN_GPIO3            3
#define PIN_GPIO4            4
#define PIN_GPIO5            5
#define PIN_GPIO6            6
#define PIN_GPIO7            7

/*------------------------------------------
 Macros - Baud & Serial Rates
------------------------------------------*/
#define BAUD_RATE            500000
#define SERIAL_RATE          9600

/*------------------------------------------
 Macros - CAN Frame Parameters
------------------------------------------*/
#define PAR_EXTENDED         0
#define PAR_REMOTE           0
#define PAR_OVERRUN          0
#define PAR_RESERVED         0
#define PAR_SEND_DLC         3
#define PAR_RECEIVE_DLC      4

/*------------------------------------------
 Macros - Bamocar CAN Message IDs
------------------------------------------*/
#define ID_CAN_MESSAGE_TX    0x181
#define ID_CAN_MESSAGE_RX    0x201

/*------------------------------------------
 Macros - Bamocar Registers
------------------------------------------*/
#define REG_DIG_SPEED_SET    0x31
#define REG_DIG_TORQUE_SET   0x90
#define REG_SPEED_ACTUAL     0x30 
#define REG_MOTOR_TEMP       0x49
#define REG_READ             0x3D
#define REG_RPM              0xA8
#define REG_VOLTAGE          0x8A
#define REG_CURRENT          0x5F

/*------------------------------------------
 Macros - Bit Manipulation
------------------------------------------*/
#define BYTE_ONE             0x00FF
#define BYTE_TWO             0xFF00
#define TWO_BYTES            65535
#define FIFTEEN_BITS         32767
#define TEN_BITS             1023

/*------------------------------------------
 Macros - Hall Effect Sensors
------------------------------------------*/
#define MAX                  0
#define MIN                  1
#define VOLTAGE              0
#define PERCENT_REQ          1
#define BSPD_CHECK           5
#define PERCENT_BRAKE        10
#define PERCENT_ACCEL        25
#define PERCENT_THRESHOLD    5
#define APPS_AGREEMENT       10

/*------------------------------------------
 Macros - Custom CAN protocol
------------------------------------------*/
#define ID_ERROR_CODE        0x681
#define ID_CURRENT_STATE     0x682
#define ID_MOTOR_TEMP        0x001
#define ID_RPM               0x002
#define ID_CURRENT           0x003
#define ID_VOLTAGE           0x004
#define PAR_ERROR_DLC        1
#define PAR_STATE_DLC        1

/*------------------------------------------
 Macros - Other
------------------------------------------*/
#define ARRAY_SIZE           40
#define DELAY                500
#define RESET                1000

/*-------------------------------------------------------------------------------------------------
 Hall Effect Processing
-------------------------------------------------------------------------------------------------*/
const uint16_t hallAPPS1[][2] {
    /* Cooked Value */  /* Percent Request */
    /*-------------------------------------*/
    {      51500      ,          100        },
    {      41000      ,            0        }
};

const uint16_t hallAPPS2[][2] {
    /* Cooked Value */  /* Percent Request */
    /*-------------------------------------*/
    {      27500      ,          100        },
    {      7200       ,            0        }
};

const uint16_t hallBSE[][2] {
    /* Cooked Value */  /* Percent Request */
    /*-------------------------------------*/
    {      33700      ,          100        },
    {      26000      ,            0        }
};

/*
    Implement soon: the percent request arrays should be a function of the supply voltage to the teensy as the battery
    drains over time.
*/

class hall {    
    public:
        // Constructor
        hall(const uint8_t pin);

        // Getters
        uint16_t GetTotal() { return total; }
        uint8_t GetCounter() { return counter; }
        uint16_t GetArrayValue(int index) { return array[index]; }

        uint16_t GetRawSupply() { return rawSupply; }
        uint16_t GetRawOutput() { return rawOutput; }
        uint16_t GetCookedOutput() { return cookedOutput; }
        uint16_t GetTorqueRequest() { return torqueRequest; }

        // Setters
        void ModifyTotal(int value) { total += value; }
        void SetCounter(int value) { counter = value; }
        void SetArrayValue(int index, int value) { array[index] = value; }

        void SetRawOutput(uint16_t value) { rawOutput = value; }
        void SetCookedOutput(uint16_t value) { cookedOutput = value; }
        void SetTorqueRequest(uint16_t value) { torqueRequest = value; }

        // Data methods
        float GetPercentRequest(const uint16_t sensor[][2]);
        uint16_t ReadPedal();
        void InitializeArray(uint16_t * pArray, const int size);
    
    private:
        // Used for averaging the raw hall effect output signal
        int16_t total;
        uint8_t counter;
        uint16_t array[ARRAY_SIZE];

        // Pedal supply, signal, and processed signal data
        uint16_t rawSupply;
        uint16_t rawOutput;
        uint16_t cookedOutput;
        uint16_t torqueRequest;

        uint8_t pin;
};

/*-------------------------------------------------------------------------------------------------
 States
-------------------------------------------------------------------------------------------------*/
typedef enum state {
    INIT,
    PRECHARGE,
    WAIT_FOR_RTD,
    IDLE,
    DRIVE,
    BRAKE,
    FAULT
} state_t;

/*-------------------------------------------------------------------------------------------------
 Prototypes
-------------------------------------------------------------------------------------------------*/
void SetPinModes();

bool ReadyToDrive(bool * bReady, hall * pBSE);

void ActivateBrakeLight(hall * pBSE);

void ActivateBamocar();

void DeactivateBamocar();

void DischargeHighVoltage();

void AverageSignal(hall * pAPPS, const uint16_t sensor[][2]);

void ProcessAPPS(hall * pAPPS1, hall * pAPPS2, int8_t * pSpeedBuf);

void UpdatePedalStructures(hall * APPS1, hall * APPS2, hall * BSE);

float GetLowerPercentAPPS(hall * pAPPS1, hall * pAPPS2);

bool CheckAPPS(hall * APPS1, hall * APPS2);

bool CheckPedalsOOR(hall * pAPPS1, hall * pAPPS2, hall * pBSE);

bool AccelAndBrakePressed(hall * pAPPS1, hall * pAPPS2, hall * pBSE);

bool CheckAllErrors(hall * pAPPS1, hall * pAPPS2, hall * pBSE, int8_t * errorBuf, bool b100msPassed, int8_t * tempCode);

void PopulateCANMessage(CAN_message_t * pMessage, uint8_t DLC, uint8_t bamocarDestReg, int8_t messageBuf[], uint16_t ID);

void PopulatePiCANMessage(CAN_message_t * pMessage, uint8_t DLC, int8_t * messageBuf, uint16_t ID);

void SendCANMessage(CAN_message_t * pMessage);

void ReadCANMessage(CAN_message_t * pMessage);

void PrintDebugMessage(const char message[]);

// End safe guards
#endif /* ECU_H */