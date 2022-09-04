/**
  ******************************************************************************
  * @file   : Service_Communication.cpp
  * @brief  : 服务通信文件
  ******************************************************************************
  * @note
  *  - 
**/

/* Includes ------------------------------------------------------------------*/
#include "Service_Communication.h"
/* Private variables --------------、-------------------------------------------*/

TrunkMotor_DataPack_Typedef TrunkMotor_DataPack;

/* Private function prototypes -------------------------------------------------------*/

HAL_StatusTypeDef CANMsgSend(CAN_HandleTypeDef *hcan, int StdID, TrunkMotor_DataPack_Typedef *msg_rece)
{
	uint32_t mailbox;
	uint8_t Motor_Tx_Data[8];
	CAN_TxHeaderTypeDef CAN_TxHeader;	
	CAN_TxHeader.IDE=CAN_ID_STD;
	CAN_TxHeader.StdId=StdID;
	CAN_TxHeader.RTR=CAN_RTR_DATA;
	CAN_TxHeader.DLC=8;

	Motor_Tx_Data[0] = msg_rece->Speed_Model;
	Motor_Tx_Data[1] = myConstrain(180 - msg_rece->TrunkMotor1_Angle,0,180);
	Motor_Tx_Data[2] = myConstrain(180 - msg_rece->TrunkMotor2_Angle,0,180);
	Motor_Tx_Data[3] = myConstrain(180 - msg_rece->TrunkMotor3_Angle,0,180);
	Motor_Tx_Data[4] = myConstrain(180 - msg_rece->TrunkMotor4_Angle,0,180);
	Motor_Tx_Data[5] = myConstrain(180 - msg_rece->TrunkMotor5_Angle,0,180);
	Motor_Tx_Data[6] = myConstrain(180 - msg_rece->TrunkMotor6_Angle,0,180);
	Motor_Tx_Data[7] = myConstrain(180 - msg_rece->TrunkMotor7_Angle,0,180);

	return HAL_CAN_AddTxMessage(hcan,&CAN_TxHeader,(uint8_t *)Motor_Tx_Data,&mailbox);	
}
