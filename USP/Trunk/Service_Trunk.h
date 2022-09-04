/**
  ******************************************************************************
  * @file   : Service_Trunk.h
  * @brief  : Service_Trunk.cppͷ�ļ�
  ****************************************************************************** 
**/
#ifndef _SERVICE_TRUNK_H_
#define _SERVICE_TRUNK_H_
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#include <FreeRTOS.h>
/* Private defines ------------------------------------------------------------*/

/* ���ID --------------*/
//CAN1���
#define TrunkMotor1_ID		0x201
#define TrunkMotor2_ID		0x202
#define TrunkMotor3_ID		0x203
#define TrunkMotor4_ID		0x204
#define YawMotor_ID				0x205

/* ���PID --------------*/
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
/* ���� --------------*/
#define DeathZone_Limit 					2500.0f

/* Private variables ---------------------------------------------------------*/

/* ����� --------------*/
extern Motor_C620 TrunkMotor1,TrunkMotor2;
extern Motor_C620 TrunkMotor3,TrunkMotor4;
extern Motor_GM6020 YawMotor;

/* ���PID���� --------------*/
extern myPID Trunk_PID[10];



/* Private function declarations ---------------------------------------------*/

/*---------------------------------------------- Universal Motor Class --------------------------------------------*/
class Motor_Classdef
{
	private:		
		void My_MotorMsgSend(CAN_HandleTypeDef *hcan,int16_t Motor_Msg[],int StdID);				//������ݷ��ͺ�����CAN�ڣ����ݰ������ID��
		void MotorPID_Control(void);				//���PID���ƺ�������


	public:
		/* ������� --------------*/
		void Update_CAN1_Motor(uint8_t *,uint32_t );				//������ݸ��º��������ݰ������ID��
		void Update_CAN2_Motor(uint8_t *,uint32_t );				//������ݸ��º��������ݰ������ID��
		void MotorPIDInit();				//���PID��ʼ����������
		void MotorSiteInit(float);				//���λ�ó�ʼ����������
		void MotorSiteAdjust();
		void MotorAdjust();
		void My_Motor_Control();				//������ƺ�������
	
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
