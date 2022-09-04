/**
  ******************************************************************************
  * @file   : Service_Trunk.h
  * @brief  : Service_Trunk.cpp头文件
  ****************************************************************************** 
**/
#ifndef _SERVICE_TRUNK_H_
#define _SERVICE_TRUNK_H_
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#include <FreeRTOS.h>
/* Private defines ------------------------------------------------------------*/

/* 电机ID --------------*/
//CAN1电机
#define TrunkMotor1_ID		0x201
#define TrunkMotor2_ID		0x202
#define TrunkMotor3_ID		0x203
#define TrunkMotor4_ID		0x204
#define YawMotor_ID				0x205

/* 电机PID --------------*/
#define TrunkMotor1_PosPID 			Trunk_PID[0]	
#define TrunkMotor1_SpdPID 			Trunk_PID[1]	
#define TrunkMotor2_PosPID 			Trunk_PID[2]	
#define TrunkMotor2_SpdPID 			Trunk_PID[3]
#define TrunkMotor3_PosPID			Trunk_PID[4]
#define TrunkMotor3_SpdPID 			Trunk_PID[5]
#define TrunkMotor4_PosPID 			Trunk_PID[6]
#define TrunkMotor4_SpdPID 			Trunk_PID[7]
#define YawMotor_PosPID 				Trunk_PID[8]	
#define YawMotor_SpdPID 				Trunk_PID[9]
/* 死区 --------------*/
#define DeathZone_Limit 					2500.0f

/* Private variables ---------------------------------------------------------*/

/* 电机类 --------------*/
extern Motor_C620 TrunkMotor1,TrunkMotor2;
extern Motor_C620 TrunkMotor3,TrunkMotor4;
extern Motor_GM6020 YawMotor;

/* 电机PID数组 --------------*/
extern myPID Trunk_PID[10];



/* Private function declarations ---------------------------------------------*/

/*---------------------------------------------- Universal Motor Class --------------------------------------------*/
class Motor_Classdef
{
	private:		
		void My_MotorMsgSend(CAN_HandleTypeDef *hcan,int16_t Motor_Msg[],int StdID);				//电机数据发送函数（CAN口，数据包，电机ID）
		void MotorPID_Control(void);				//电机PID控制函数（）


	public:
		/* 电机函数 --------------*/
		void Update_CAN1_Motor(uint8_t *,uint32_t );				//电机数据更新函数（数据包，电机ID）
		void Update_CAN2_Motor(uint8_t *,uint32_t );				//电机数据更新函数（数据包，电机ID）
		void MotorPIDInit();				//电机PID初始化函数（）
		void MotorSiteInit(float);				//电机位置初始化函数（）
		void MotorSiteAdjust();
		void MotorAdjust();
		void My_Motor_Control();				//电机控制函数（）
	
		void Yaw_Ctrl(float);
		void Yaw(float);
		void Trunk_Ctrl_X(float);
		void Trunk_Ctrl_Y(float);
		void Trunk_Ctrl_L(float);
		void Trunk_Ctrl_R(float);
		void Trunk_Ctrl_M(float,float);
		void Trunk_Ctrl(float,float);
		void Trunk_Protect();
		void Trunk_Move_XY(float, float, float);
		bool Trunk_Judge_XY(float, float);
		void Trunk_Auto_XY(float,float,float);
		void Trunk_Move_LR(float, float, float);
		bool Trunk_Judge_LR(float, float);
		void Trunk_Auto_LR(float,float,float);
};

#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
