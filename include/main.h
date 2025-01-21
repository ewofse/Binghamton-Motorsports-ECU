// Safe guards
#ifndef MAIN_H
#define MAIN_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include <FlexCAN_T4.h>
#include <Watchdog_T4.h>

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <SPI.h>
#include <SD.h>

#include "hall.h"
#include "ISR.h"

/*-------------------------------------------------------------------------------------------------
 Initializations for CAN Communication
-------------------------------------------------------------------------------------------------*/
extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;

/*------------------------------------------
 Macros - Pins
------------------------------------------*/
// #define EV1
#define EV1_5

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
// #define PIN_VREF

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
#define PAR_RX_DLC           3
#define PAR_TX_DLC           4

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
#define REG_TEMP             0x49
#define REG_READ             0x3D
#define REG_RPM              0xA8
#define REG_VOLTAGE          0x8A
#define REG_CURRENT          0x5F
#define REG_CURRENT_PHASE_1  0x54
#define REG_CURRENT_PHASE_2  0x55
#define REG_CURRENT_PHASE_3  0x56
#define TRANSMIT_ONCE        0x00

/*------------------------------------------
 Macros - Custom CAN protocol
------------------------------------------*/
#define ID_ERROR_CODE        0x681
#define ID_CURRENT_STATE     0x682
#define ID_TEMP              0x001
#define ID_SPEED             0x002
#define ID_CURRENT           0x003
#define ID_VOLTAGE           0x004
#define PAR_ERROR_DLC        1
#define PAR_STATE_DLC        1

/*------------------------------------------
 Macros - Bit Manipulation
------------------------------------------*/
#define BYTE_ONE             0x00FF
#define BYTE_TWO             0xFF00
#define TWO_BYTES            65535
#define FIFTEEN_BITS         32767
#define TEN_BITS             1023

/*------------------------------------------
 Macros - Timing
------------------------------------------*/
#define IMPLAUSIBILITY_TIME  100
#define CHARGE_TIME          500  
#define BUZZER_TIME          1000
#define RESET_TIME           1000
#define DISCHARGE_TIME       1500

/*------------------------------------------
 Macros - Files
------------------------------------------*/
#define OVERWRITE            1
#define FILE_PEDAL_BOUNDS    "pedal_bounds.csv"
#define FILE_CURRENT_DRAW    "current_draw.csv"

/*------------------------------------------
 Macros - Other
------------------------------------------*/
#define DELIMITER            ','
#define ADC_RESOLUTION       TEN_BITS
#define ERROR_CODE_SHUTDOWN  0
#define ERROR_CODE_DISAGREE  1
#define ERROR_CODE_APPS_BSE  2
#define ERROR_CODE_OOR       3

/*------------------------------------------
 Macros - Debugging
------------------------------------------*/
#define EXIT while (1) {}
#define DEBUG

#ifdef DEBUG
    #define DebugBegin(baudRate)     Serial.begin(baudRate)
    #define DebugPrint(message)      Serial.print(message)
    #define DebugPrintln(message)    Serial.println(message)
    #define DebugPrintHEX(message)   Serial.println(message, HEX)
    #define DebugErrorPrint(message) { for (int i = 0; i < 100; ++i) DebugPrintln(message); }
#else
    // Empty defines cause all prints to be ignored during acutal operation
    #define DebugBegin(baudRate)
    #define DebugPrint(message)
    #define DebugPrintln(message)
    #define DebugPrintHEX(message)
    #define DebugErrorPrint(message)
#endif

/*-------------------------------------------------------------------------------------------------
 States / Data Structures
-------------------------------------------------------------------------------------------------*/
typedef enum state {
    PEDALS,
    INIT,
    PRECHARGE,
    WAIT_FOR_RTD,
    IDLE,
    DRIVE,
    BRAKE,
    FAULT
} state_t;

typedef enum calibrate {
    PERCENT_REQ_LOWER,
    PERECENT_REQ_UPPER,
    DONE
} calibrate_t;

typedef struct timers {
    elapsedMillis chargeTimer;
    elapsedMillis buzzerTimer;
    elapsedMillis pedalErrorTimer;
    bool bChargeTimerStarted;
    bool bBuzzerActive;
    bool b100msPassed;
} timers_t;

/*-------------------------------------------------------------------------------------------------
 Prototypes
-------------------------------------------------------------------------------------------------*/
void SetPinModes();

void ConfigureCANBus();

bool ButtonDebouncer(uint8_t pin);

bool ButtonPulser(bool signal);

bool ReadyToDrive(hall * pBSE);

void ActivateBrakeLight(hall * pBSE);

void ActivateBamocar();

void DeactivateBamocar();

void ActivateFaultLED();

void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, 
    uint8_t * pMessageBuf, uint8_t bamocarDestReg);
void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, uint8_t bamocarDestReg);
void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, uint8_t * pMessageBuf);

bool MapCANMessage(CAN_message_t * pMessage1, CAN_message_t * pMessage2);

void SendCANMessage(CAN_message_t * pMessage);

void ReadCANMessage(CAN_message_t * pMessage);

void SendVehicleState(uint8_t state);

uint16_t * SplitIntegerString(String strValue, const char delimiter);

void WriteDataToFile(const char * strFileName, const String & strData, bool overwrite);

void UpdateCurrentData(CAN_message_t * pMessage, String & strData, uint16_t pDataBuf[]);

// End safe guards
#endif /* MAIN_H */
