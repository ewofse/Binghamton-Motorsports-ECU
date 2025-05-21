// Safe guards
#ifndef CAN_H
#define CAN_H

/*-------------------------------------------------------------------------------------------------
 Libraries
-------------------------------------------------------------------------------------------------*/
#include "core/general.h"
#include "interrupts/interrupts.h"

/*-------------------------------------------------------------------------------------------------
 Initializations for CAN Communication
-------------------------------------------------------------------------------------------------*/
extern FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> myCan;

/*------------------------------------------
 Macros - CAN Frame Parameters
------------------------------------------*/
#define PAR_EXTENDED         	 0
#define PAR_REMOTE           	 0
#define PAR_OVERRUN          	 0
#define PAR_RESERVED         	 0
#define PAR_RX_DLC           	 3
#define PAR_TX_DLC           	 4

/*------------------------------------------
 Macros - Bamocar CAN Message IDs
------------------------------------------*/
#define ID_CAN_MESSAGE_TX    	 0x181
#define ID_CAN_MESSAGE_RX    	 0x201

/*------------------------------------------
 Macros - Bamocar Registers
------------------------------------------*/
#define REG_DIG_SPEED_SET    	 0x31
#define REG_DIG_TORQUE_SET   	 0x90
#define REG_SPEED_ACTUAL     	 0x30
#define REG_SPEED_FILTERED       0xA8
#define REG_SPEED_NOMINAL        0x59
#define REG_MOTOR_TEMP           0x49
#define REG_BAMOCAR_TEMP         0x4A
#define REG_READ             	 0x3D
#define REG_VOLTAGE          	 0x8A
#define REG_CURRENT          	 0x5F
#define REG_CURRENT_PHASE_1  	 0x54
#define REG_CURRENT_PHASE_2  	 0x55
#define REG_CURRENT_PHASE_3  	 0x56
#define TRANSMIT_ONCE        	 0x00
#define TRANSMIT_100MS           0x64

/*------------------------------------------
 Macros - Custom CAN protocol
------------------------------------------*/
#define ID_BATTERY_TEMP          0x6B1
#define ID_ERROR_CODE        	 0x681
#define ID_CURRENT_STATE     	 0x682
#define ID_TEMP              	 0x001
#define ID_SPEED              	 0x002
#define ID_CURRENT           	 0x003
#define ID_VOLTAGE            	 0x004
#define PAR_ERROR_DLC         	 1
#define PAR_STATE_DLC        	 1

#define NUM_TX_MAILBOXES     	 10
#define NUM_RX_MAILBOXES	 	 6 // FlexCAN FIFO queue holds 6
#define NUM_MAILBOXES        	 NUM_TX_MAILBOXES + NUM_RX_MAILBOXES
#define MAILBOX_TORQUE		 	 MB6
#define MAILBOX_FAULT		 	 MB7
#define MAILBOX_STATE		 	 MB8
#define MAILBOX_TEMP		 	 MB9
#define MAILBOX_SPEED		 	 MB10
#define MAILBOX_CURRENT		 	 MB11
#define MAILBOX_VOLTAGE		 	 MB12
#define MAILBOX_PHASE_ONE	 	 MB13
#define MAILBOX_PHASE_TWO	 	 MB14
#define MAILBOX_PHASE_THREE	 	 MB15

#define STATUS_MESSAGE_INTERVAL	 50
#define NUM_MESSAGES_TX		 	 6

/*-------------------------------------------------------------------------------------------------
 Prototypes
-------------------------------------------------------------------------------------------------*/
void ConfigureCANBus();

void PrintCANMessage(const CAN_message_t & message);

void ProcessCANMessage(const CAN_message_t & message);

void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, 
    uint8_t * pMessageBuf, uint8_t bamocarDestReg);
void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, uint8_t bamocarDestReg);
void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, uint8_t bamocarDestReg,
    uint8_t transmissionInterval);
void PopulateCANMessage(CAN_message_t * pMessage, uint16_t ID, uint8_t DLC, uint8_t * pMessageBuf);

bool MapCANMessage(CAN_message_t & message);

void SendCANMessage(const CAN_message_t & message);
void SendCANMessage(const CAN_message_t & message, const FLEXCAN_MAILBOX MB);

void SendCANStatusMessages(uint8_t * errors, uint8_t * state);

void RequestBamocarData();

#endif /* CAN_H */
