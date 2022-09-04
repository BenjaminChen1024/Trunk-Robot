/**
  ******************************************************************************
  * @file   : System_Datapool.h
  * @brief  : System_Datapool.cppÍ·ÎÄ¼þ
  ****************************************************************************** 
**/
#ifndef _SERVICE_COMMUNICATION_H_
#define _SERVICE_COMMUNICATION_H_
/* Includes ------------------------------------------------------------------*/
#include "System_Datapool.h"
/* Private variables ---------------------------------------------------------*/

#pragma pack(1)
struct TrunkMotor_DataPack_Typedef 
{
	uint8_t Speed_Model;
	uint8_t TrunkMotor1_Angle = 90;
	uint8_t TrunkMotor2_Angle = 90;
	uint8_t TrunkMotor3_Angle = 90;
	uint8_t TrunkMotor4_Angle = 90;
	uint8_t TrunkMotor5_Angle = 90;
	uint8_t TrunkMotor6_Angle = 90;
	uint8_t TrunkMotor7_Angle = 90;
};
#pragma pack()

extern TrunkMotor_DataPack_Typedef TrunkMotor_DataPack;

HAL_StatusTypeDef CANMsgSend(CAN_HandleTypeDef *, int , TrunkMotor_DataPack_Typedef *);
#endif
