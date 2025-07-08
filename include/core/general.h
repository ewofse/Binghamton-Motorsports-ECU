// Safe guards
#ifndef GENERAL_H
#define GENERAL_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include <FlexCAN_T4.h>
#include <Watchdog_t4.h>
#include <Teensy_PWM.h>
#include <PID_v2.h>

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <SPI.h>
#include <SD.h>

/*------------------------------------------
 Macros - Vehicle Model
------------------------------------------*/
// #define EV1
#define EV1_5

/*------------------------------------------
 Macros - Baud & Serial Rates
------------------------------------------*/
#define BAUD_RATE            	 500000
#define SERIAL_RATE          	 9600

/*------------------------------------------
 Macros - Bit Manipulation
------------------------------------------*/
#define BYTE_ONE             	 0x00FF
#define BYTE_TWO             	 0xFF00
#define TWO_BYTES            	 65535
#define FIFTEEN_BITS         	 32767
#define TEN_BITS             	 1023

/*------------------------------------------
 Macros - Timing
------------------------------------------*/
#define IMPLAUSIBILITY_TIME  	 100
#define CHARGE_TIME          	 1000  
#define BUZZER_TIME          	 1000
#define RESET_TIME           	 5000
#define DISCHARGE_TIME       	 1500
#define SHUTDOWN_STABLE_TIME 	 500
#define BUTTON_DEBOUNCE_TIME 	 5
#define PEDAL_CALIBRATION_TIME	 2000

/*------------------------------------------
 Macros - Other
------------------------------------------*/
#define ANALOG_PIN_BUFFER_SIZE   500
#define SDC_TAP_HIGH             512
#define DELIMITER            	 ','
#define ADC_RESOLUTION       	 TEN_BITS
#define ERROR_CODE_SHUTDOWN  	 0
#define ERROR_CODE_DISAGREE  	 1
#define ERROR_CODE_APPS_BSE  	 2
#define ERROR_CODE_OOR       	 3

/*------------------------------------------
 Macros - Debugging
------------------------------------------*/
#define EXIT while (1) {}
#define DEBUG

#ifdef DEBUG
    #define DebugBegin(baudRate)      Serial.begin(baudRate)
    #define DebugPrint(message)       Serial.print(message)
    #define DebugPrintln(message)     Serial.println(message)
    #define DebugPrintHEX(message)    Serial.print(message, HEX)
    #define DebugErrorPrint(message)  { for (int i = 0; i < 100; ++i) DebugPrintln(message); }
    #define DebugPrintVehicleErrors(systemObject) systemObject.DebugPrintErrors()
    #define DebugPrintCANMessage(message) PrintCANMessage(message);
#else
    // Empty defines cause all prints to be ignored during acutal operation
    #define DebugBegin(baudRate)
    #define DebugPrint(message)
    #define DebugPrintln(message)
    #define DebugPrintHEX(message)
    #define DebugErrorPrint(message)
    #define DebugPrintVehicleErrors(system)
    #define DebugPrintCANMessage(message)
#endif

/*-------------------------------------------------------------------------------------------------
 States / Data Structures
-------------------------------------------------------------------------------------------------*/
enum class systemState : uint8_t {
    PRECHARGE = 0,
    RTD,
    IDLE,
    DRIVE,
    BRAKE,
    FAULT,
    INIT,
    CALIBRATE,
    PEDALS
};

enum class pedalCalibrate {
	UPDATE_PEDALS,
    PERCENT_REQ_UPPER,
    PERCENT_REQ_LOWER,
    DONE
};

typedef struct timers {
    elapsedMillis chargeTimer;
    elapsedMillis buzzerTimer;
    elapsedMillis resetTimer;
    bool bChargeTimerStarted;
    bool bBuzzerActive;
    bool b100msPassed;
    bool bResetTimerStarted;
} timers_t;

// End safe guards
#endif /* GENERAL_H */
