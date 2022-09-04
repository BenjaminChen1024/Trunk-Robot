/**
  ******************************************************************************
  * @file   : Service_Chassis.h
  * @brief  : Service_Chassis.cppͷ�ļ�
  ****************************************************************************** 
  */
	
#ifndef _SERVICE_CHASSIS_H_
#define _SERVICE_CHASSIS_H_
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#include <FreeRTOS.h>

//CAN2���
#define ChassisMotor1_ID	0x205
#define ChassisMotor2_ID	0x206
#define ChassisMotor3_ID	0x207
#define ChassisMotor4_ID	0x208

/* Private variables ---------------------------------------------------------*/

/* ����Ŀ���ٶ� --------------*/
extern float  TargetVelocity_X , TargetVelocity_Y , TargetVelocity_Z;
/* ���̵�����ʼ�� --------------*/
extern CChassis Trunk_Chassis;
/* ���PID���� --------------*/
extern myPID Chassis_PID[4]; 

/* Private function declarations ---------------------------------------------*/
void Controller_Init();
int* SpeedController(const int16_t* current, const int16_t* target);
void Chassis_ExtInit();
void Set_TargetVelocity();
void Update_CAN2_Motor(uint8_t *msg_rece,uint32_t Motor_ID);
void Motor_Send();

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
