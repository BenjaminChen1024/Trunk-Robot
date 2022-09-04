/**
  ******************************************************************************
  * @file   : Service_Motor.cpp
  * @brief  : 服务电机文件
  ******************************************************************************
  * @note
  *  - 
**/

/* Includes ------------------------------------------------------------------*/
#include "Service_Trunk.h"
#include "Motor.h"
/* Private variables ---------------------------------------------------------*/

/* 电机类 --------------*/
Motor_C620 TrunkMotor1(1),TrunkMotor2(2);
Motor_C620 TrunkMotor3(3),TrunkMotor4(4);
Motor_GM6020 YawMotor(5);
/* 电机PID数组 --------------*/
myPID Trunk_PID[10];


/* HAL Handlers --------------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/* Private function prototypes -------------------------------------------------------*/

/*---------------------------------------------- Universal Motor Function --------------------------------------------*/
uint32_t test_num;
void Motor_Classdef::My_MotorMsgSend(CAN_HandleTypeDef *hcan,int16_t Motor_Msg[],int StdID)
{
	uint8_t Tx_Data[8];
	Tx_Data[0]=Motor_Msg[0] >> 8;
	Tx_Data[1]=Motor_Msg[0];
	Tx_Data[2]=Motor_Msg[1] >> 8;
	Tx_Data[3]=Motor_Msg[1];
	Tx_Data[4]=Motor_Msg[2] >> 8;
	Tx_Data[5]=Motor_Msg[2];
	Tx_Data[6]=Motor_Msg[3] >> 8;
	Tx_Data[7]=Motor_Msg[3];
	if(CANx_SendData(hcan,StdID,Tx_Data,8) != CAN_SUCCESS)
	{
		test_num++;
	}
}

void Motor_Classdef::Update_CAN1_Motor(uint8_t *msg_rece,uint32_t Motor_ID)
{
	switch(Motor_ID)
	{
		case TrunkMotor1_ID:
			TrunkMotor1.update(msg_rece);
			TrunkMotor1_PosPID.Current=TrunkMotor1.getAngle();
			TrunkMotor1_SpdPID.Current=TrunkMotor1.getSpeed();
			break;
		case TrunkMotor2_ID:
			TrunkMotor2.update(msg_rece);
			TrunkMotor2_PosPID.Current=TrunkMotor2.getAngle();
			TrunkMotor2_SpdPID.Current=TrunkMotor2.getSpeed();
			break;
		case TrunkMotor3_ID:
			TrunkMotor3.update(msg_rece);
			TrunkMotor3_PosPID.Current=TrunkMotor3.getAngle();
			TrunkMotor3_SpdPID.Current=TrunkMotor3.getSpeed();
			break;
		case TrunkMotor4_ID:
			TrunkMotor4.update(msg_rece);
			TrunkMotor4_PosPID.Current=TrunkMotor4.getAngle();
			TrunkMotor4_SpdPID.Current=TrunkMotor4.getSpeed();
			break;
		case YawMotor_ID:
			YawMotor.update(msg_rece);
			YawMotor_PosPID.Current=YawMotor.getAngle();
			YawMotor_SpdPID.Current=YawMotor.getSpeed();
			break;
		default:
			break;
	}
}



void Motor_Classdef::MotorPIDInit()
{
	/* PID计时器初始化 --------------*/
	myPIDTimer::getMicroTick_regist(Get_SystemTimer);
	/* Motor PID Config */
	TrunkMotor1_PosPID.SetPIDParam(8.0f,0.0f,0.0f,1000.0f,10000.0f);
	TrunkMotor1_SpdPID.SetPIDParam(20.0f,0.0f,0.0f,10000.0f,10000.0f);
	TrunkMotor2_PosPID.SetPIDParam(8.0f,0.0f,0.0f,1000.0f,10000.0f);
	TrunkMotor2_SpdPID.SetPIDParam(20.0f,0.0f,0.0f,10000.0f,10000.0f);
	TrunkMotor3_PosPID.SetPIDParam(8.0f,0.0f,0.0f,1000.0f,10000.0f);
	TrunkMotor3_SpdPID.SetPIDParam(20.0f,0.0f,0.0f,10000.0f,10000.0f);
	TrunkMotor4_PosPID.SetPIDParam(8.0f,0.0f,0.0f,1000.0f,10000.0f);
	TrunkMotor4_SpdPID.SetPIDParam(20.0f,0.0f,0.0f,10000.0f,10000.0f);
	YawMotor_PosPID.SetPIDParam(80.0f,0.0f,0.0f,0.0f,20000.0f);
	YawMotor_SpdPID.SetPIDParam(120.0f,0.0,0.0f,20000.0f,30000.0f);

}

void Motor_Classdef::MotorPID_Control(void)
{
	/* PID Control */
	TrunkMotor1_PosPID.Adjust();
	TrunkMotor1_SpdPID.Target = TrunkMotor1_PosPID.Out;
	TrunkMotor1_SpdPID.Adjust();
	TrunkMotor2_PosPID.Adjust();
	TrunkMotor2_SpdPID.Target = TrunkMotor2_PosPID.Out;
	TrunkMotor2_SpdPID.Adjust();
	TrunkMotor3_PosPID.Adjust();
	TrunkMotor3_SpdPID.Target = TrunkMotor3_PosPID.Out;
	TrunkMotor3_SpdPID.Adjust();
	TrunkMotor4_PosPID.Adjust();
	TrunkMotor4_SpdPID.Target = TrunkMotor4_PosPID.Out;
	TrunkMotor4_SpdPID.Adjust();
	YawMotor_PosPID.Adjust();
	YawMotor_SpdPID.Target = YawMotor_PosPID.Out;
	YawMotor_SpdPID.Adjust();

}

void Motor_Classdef::My_Motor_Control()
{
		int16_t MotorMsg[4];
	/* 电机PID控制 --------------*/
		MotorPID_Control();
	
	/* CAN1 --------------*/
	/* 发送0x1ff电机数据 --------------*/
		MotorMsg[0] = YawMotor_SpdPID.Out;
		MotorMsg[1] = 0;
		MotorMsg[2] = 0;
		MotorMsg[3] = 0;		
		My_MotorMsgSend(&hcan1,MotorMsg,0x1ff);
	/* 发送0x200电机数据 --------------*/
		MotorMsg[0] = TrunkMotor1_SpdPID.Out;
		MotorMsg[1] = TrunkMotor2_SpdPID.Out;
		MotorMsg[2] = TrunkMotor3_SpdPID.Out;
		MotorMsg[3] = TrunkMotor4_SpdPID.Out;
		My_MotorMsgSend(&hcan1,MotorMsg,0x200); 

}

void Motor_Classdef::Yaw_Ctrl(float Norm)
{
	if(Norm != 0)
	{
		YawMotor_PosPID.Target = myConstrain(YawMotor_PosPID.Target - Norm*0.08f, -100.0f, 100.0f);
	}
}

void Motor_Classdef::Yaw(float Pos)
{
	if(YawMotor_PosPID.Current <= Pos)
	{
			YawMotor_PosPID.Target = myConstrain(YawMotor_PosPID.Target + 0.05f, -100.0f, Pos);
	}
	else
	{
			YawMotor_PosPID.Target = myConstrain(YawMotor_PosPID.Target - 0.05f, Pos, 100.0f);
	}
}

void Motor_Classdef::Trunk_Ctrl_X(float Norm)
{
	if(Norm != 0)
	{
		TrunkMotor1_PosPID.Target = myConstrain(TrunkMotor1_PosPID.Target + Norm*2.0f, -3000.0f, 3000.0f);
	}
}

void Motor_Classdef::Trunk_Ctrl_Y(float Norm)
{
	if(Norm != 0)
	{
		TrunkMotor2_PosPID.Target = myConstrain(TrunkMotor2_PosPID.Target + Norm*2.0f, -3000.0f, 3000.0f);
	}
}

void Motor_Classdef::Trunk_Ctrl_L(float Norm)
{
	if(Norm != 0)
	{
		TrunkMotor3_PosPID.Target = myConstrain(TrunkMotor3_PosPID.Target + Norm*2.0f, -1500.0f, 1500.0f);
	}
}

void Motor_Classdef::Trunk_Ctrl_R(float Norm)
{
	if(Norm != 0)
	{
		TrunkMotor4_PosPID.Target = myConstrain(TrunkMotor4_PosPID.Target + Norm*2.0f, -1500.0f, 1500.0f);
	}
}

void Motor_Classdef::Trunk_Ctrl_M(float X_Norm, float Y_Norm)
{
	if(X_Norm != 0 || Y_Norm != 0)
	{
		X_Norm = -X_Norm;
		TrunkMotor3_PosPID.Target = myConstrain(TrunkMotor3_PosPID.Target + (X_Norm + Y_Norm)*2.0f, -1500.0f, 1500.0f);
		TrunkMotor4_PosPID.Target = myConstrain(TrunkMotor4_PosPID.Target + (X_Norm - Y_Norm)*2.0f, -1500.0f, 1500.0f);
	}
}

void Motor_Classdef::Trunk_Ctrl(float X_Norm, float Y_Norm)
{
	if(X_Norm != 0 || Y_Norm != 0)
	{
		TrunkMotor1_PosPID.Target = myConstrain(TrunkMotor1_PosPID.Target + X_Norm*5.0f, -3000.0f, 3000.0f);
		TrunkMotor2_PosPID.Target = myConstrain(TrunkMotor2_PosPID.Target + Y_Norm*5.0f, -3000.0f, 3000.0f);

		X_Norm = -X_Norm;
		TrunkMotor3_PosPID.Target = myConstrain(TrunkMotor3_PosPID.Target + (X_Norm + Y_Norm)*2.0f, -1500.0f, 1500.0f);
		TrunkMotor4_PosPID.Target = myConstrain(TrunkMotor4_PosPID.Target + (X_Norm - Y_Norm)*2.0f, -1500.0f, 1500.0f);
	}
}

void Motor_Classdef::Trunk_Protect()
{
		TrunkMotor1_PosPID.Target = TrunkMotor1_PosPID.Current;
		TrunkMotor2_PosPID.Target = TrunkMotor2_PosPID.Current;
		TrunkMotor3_PosPID.Target = TrunkMotor3_PosPID.Current;
		TrunkMotor4_PosPID.Target = TrunkMotor4_PosPID.Current;
}

void Motor_Classdef::Trunk_Move_XY(float Auto_X_Pos, float Auto_Y_Pos, float Auto_XY_speed)
{
		if(TrunkMotor1_PosPID.Target<Auto_X_Pos){TrunkMotor1_PosPID.Target = myConstrain(TrunkMotor1_PosPID.Target + Auto_XY_speed, -3000.0f, Auto_X_Pos);}
		if(TrunkMotor1_PosPID.Target>Auto_X_Pos){TrunkMotor1_PosPID.Target = myConstrain(TrunkMotor1_PosPID.Target - Auto_XY_speed, Auto_X_Pos, 3000.0f);}
		if(TrunkMotor2_PosPID.Target<Auto_Y_Pos){TrunkMotor2_PosPID.Target = myConstrain(TrunkMotor2_PosPID.Target + Auto_XY_speed, -3000.0f, Auto_Y_Pos);}
		if(TrunkMotor2_PosPID.Target>Auto_Y_Pos){TrunkMotor2_PosPID.Target = myConstrain(TrunkMotor2_PosPID.Target - Auto_XY_speed, Auto_Y_Pos, 3000.0f);}
}

bool Motor_Classdef::Trunk_Judge_XY(float Auto_X_Pos, float Auto_Y_Pos)
{
	return TrunkMotor1_PosPID.Target == Auto_X_Pos && TrunkMotor2_PosPID.Target == Auto_Y_Pos;
}

uint8_t Auto_XY_flag = 0;
void Motor_Classdef::Trunk_Auto_XY(float Auto_X_Pos, float Auto_Y_Pos, float Auto_XY_speed)
{
	switch(Auto_XY_flag)
	{

		case 0:
			Trunk_Move_XY(0, Auto_Y_Pos, Auto_XY_speed);
			if(Trunk_Judge_XY(0,Auto_Y_Pos)){Auto_XY_flag=1;}
			break;
		case 1:
			Trunk_Move_XY(Auto_X_Pos, 0, Auto_XY_speed);
			if(Trunk_Judge_XY(Auto_X_Pos, 0)){Auto_XY_flag=2;}
			break;
		case 2:
			Trunk_Move_XY(0, -Auto_Y_Pos, Auto_XY_speed);
			if(Trunk_Judge_XY(0, -Auto_Y_Pos)){Auto_XY_flag=3;}
			break;
		case 3:
			Trunk_Move_XY(-Auto_X_Pos, 0, Auto_XY_speed);
			if(Trunk_Judge_XY(-Auto_X_Pos, 0)){Auto_XY_flag=0;}
			break;
		default:
			break;
	}
}


void Motor_Classdef::Trunk_Move_LR(float Auto_X_Pos, float Auto_Y_Pos, float Auto_LR_speed)
{
	float Auto_L_Pos,Auto_R_Pos;
	Auto_L_Pos = (Auto_X_Pos + Auto_Y_Pos)*0.707;
	Auto_R_Pos = (Auto_X_Pos - Auto_Y_Pos)*0.707;

	if(TrunkMotor3_PosPID.Target<Auto_L_Pos){TrunkMotor3_PosPID.Target = myConstrain(TrunkMotor3_PosPID.Target + Auto_LR_speed, -1500.0f, Auto_L_Pos);}
	if(TrunkMotor3_PosPID.Target>Auto_L_Pos){TrunkMotor3_PosPID.Target = myConstrain(TrunkMotor3_PosPID.Target - Auto_LR_speed, Auto_L_Pos, 1500.0f);}
	if(TrunkMotor4_PosPID.Target<Auto_R_Pos){TrunkMotor4_PosPID.Target = myConstrain(TrunkMotor4_PosPID.Target + Auto_LR_speed, -1500.0f, Auto_R_Pos);}
	if(TrunkMotor4_PosPID.Target>Auto_R_Pos){TrunkMotor4_PosPID.Target = myConstrain(TrunkMotor4_PosPID.Target - Auto_LR_speed, Auto_R_Pos, 1500.0f);}
}

bool Motor_Classdef::Trunk_Judge_LR(float Auto_X_Pos, float Auto_Y_Pos)
{
	float Auto_L_Pos,Auto_R_Pos;
	Auto_L_Pos = (Auto_X_Pos + Auto_Y_Pos)*0.707;
	Auto_R_Pos = (Auto_X_Pos - Auto_Y_Pos)*0.707;
	return TrunkMotor3_PosPID.Target == Auto_L_Pos && TrunkMotor4_PosPID.Target == Auto_R_Pos;
}

uint8_t Auto_LR_flag = 0;
void Motor_Classdef::Trunk_Auto_LR(float Auto_X_Pos, float Auto_Y_Pos, float Auto_LR_speed)
{
	switch(Auto_LR_flag)
	{

		case 0:
			Trunk_Move_LR(0, Auto_Y_Pos, Auto_LR_speed);
			if(Trunk_Judge_LR(0,Auto_Y_Pos)){Auto_LR_flag=2;}
			break;
		case 1:
			Trunk_Move_LR(Auto_X_Pos, 0, Auto_LR_speed);
			if(Trunk_Judge_LR(Auto_X_Pos, 0)){Auto_LR_flag=2;}
			break;
		case 2:
			Trunk_Move_LR(0, -Auto_Y_Pos, Auto_LR_speed);
			if(Trunk_Judge_LR(0, -Auto_Y_Pos)){Auto_LR_flag=0;}
			break;
		case 3:
			Trunk_Move_LR(-Auto_X_Pos, 0, Auto_LR_speed);
			if(Trunk_Judge_LR(-Auto_X_Pos, 0)){Auto_LR_flag=0;}
			break;
		default:
			break;
	}
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
