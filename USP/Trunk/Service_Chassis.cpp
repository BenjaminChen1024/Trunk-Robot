/**
  ******************************************************************************
  * @file   : Service_Chassis.cpp
  * @brief  : 底盘控制器文件
  ******************************************************************************
  * @note
  *  - 
**/

/* Includes ------------------------------------------------------------------*/
#include "Service_Chassis.h"
#include "drv_timer.h"

/* Private variables ---------------------------------------------------------*/
/* 设置目标速度 --------------*/
float  TargetVelocity_X , TargetVelocity_Y , TargetVelocity_Z;
/* 底盘电机类初始化 --------------*/
CChassis Trunk_Chassis(0,0,76,16000,9000);
/* 底盘电机PID数组 --------------*/
myPID Chassis_PID[4];

/**
* @brief  控制器初始化函数.
* @param  void.
* @return void.
* @note  	初始化底盘电机PID.
*/
void Controller_Init()
{
	for(uint8_t i = 0; i < 4; i++)
	{
		Chassis_PID[i].SetPIDParam(5.2f, 0, 0, 3000, 16000);
	}
	myPIDTimer::getMicroTick_regist(Get_SystemTimer);
}

/**
* @brief  底盘电机初始化函数.
* @param  void.
* @return void.
* @note  	初始化底盘电机与PID.
*/
 void Chassis_ExtInit()
{
	Trunk_Chassis.Set_SpeedGear(FAST_GEAR);
	Trunk_Chassis.Set_SpeedParam(0.35,0.5,1.0,1.0);
	Trunk_Chassis.Set_AccelerationParam(9000, 28000, 55555);	/* 单位rpm  0-35000 */
	Trunk_Chassis.Set_TorqueOptimizeFlag(1);  
	Trunk_Chassis.Set_AttitudeOptimizeFlag(0);			/* 不进行姿态优化 */
	Trunk_Chassis.Switch_Mode(Normal_Speed);
	Trunk_Chassis.Load_SpeedController(SpeedController);
	/* PID初始化 --------------*/
	Controller_Init();/*PID*/
}

/**
* @brief  速度控制函数.
* @param  current				当前速度.
* @param  target				目标速度.
* @return Chassis_out		底盘电机发送数据.
* @note  	设置速度环目标值、计算PID.
*/
int* SpeedController(const int16_t* current, const int16_t* target)
{
	static int Chassis_out[4];	
	for(uint8_t motor = 0; motor < 4; motor++)		
	{
		Chassis_PID[motor].Target = (float)target[motor];
		Chassis_PID[motor].Current = (float)current[motor];		
		Chassis_PID[motor].Adjust();
		Chassis_out[motor] = Chassis_PID[motor].Out;
	}		
	return Chassis_out;
}

/**
* @brief  设置目标速度函数.
* @param  void.
* @return void.
* @note  	None.
*/
void Set_TargetVelocity()
{
	Trunk_Chassis.Set_Target(TargetVelocity_X,TargetVelocity_Y,TargetVelocity_Z);				
}

/**
* @brief  更新底盘电机数据函数.
* @param  msg_rece	数据包.
* @param  motor			电机ID.
* @return void.
* @note  	None.
*/
void Update_CAN2_Motor(uint8_t *msg_rece,uint32_t Motor_ID)
{
	switch(Motor_ID)
	{
		case ChassisMotor1_ID:
			Trunk_Chassis.wheel_rpm[0] = (short)((unsigned char)msg_rece[2]<<8|(unsigned char)msg_rece[3]);	
			Trunk_Chassis.wheel_torque[0] = (short)((unsigned char)msg_rece[4]<<8|(unsigned char)msg_rece[5]);	
			break;
		case ChassisMotor2_ID:
			Trunk_Chassis.wheel_rpm[3] = (short)((unsigned char)msg_rece[2]<<8|(unsigned char)msg_rece[3]);	
			Trunk_Chassis.wheel_torque[3] = (short)((unsigned char)msg_rece[4]<<8|(unsigned char)msg_rece[5]);	
			break;
		case ChassisMotor3_ID:
			Trunk_Chassis.wheel_rpm[2] = (short)((unsigned char)msg_rece[2]<<8|(unsigned char)msg_rece[3]);	
			Trunk_Chassis.wheel_torque[2] = (short)((unsigned char)msg_rece[4]<<8|(unsigned char)msg_rece[5]);	
			break;
		case ChassisMotor4_ID:
			Trunk_Chassis.wheel_rpm[1] = (short)((unsigned char)msg_rece[2]<<8|(unsigned char)msg_rece[3]);	
			Trunk_Chassis.wheel_torque[1] = (short)((unsigned char)msg_rece[4]<<8|(unsigned char)msg_rece[5]);	
			break;
		default:
			break;
	}
}

/**
* @brief  电机数据发送函数.
* @param  void.
* @return void.
* @note  	发送数据控制底盘电机.
*/
void Motor_Send()
{
  Trunk_Chassis.wheel_Out[1]= -Trunk_Chassis.wheel_Out[1];
	Trunk_Chassis.wheel_Out[2]= -Trunk_Chassis.wheel_Out[2];
	
	uint8_t msg_send[8] = {0};	
	
	msg_send[0] = (unsigned char)((short)Trunk_Chassis.wheel_Out[0] >> 8);//低8位
	msg_send[1] = (unsigned char)(short)Trunk_Chassis.wheel_Out[0];
	msg_send[2] = (unsigned char)((short)Trunk_Chassis.wheel_Out[3]>> 8);
	msg_send[3] = (unsigned char)(short)Trunk_Chassis.wheel_Out[3];
	msg_send[4] = (unsigned char)((short)Trunk_Chassis.wheel_Out[2] >> 8);
	msg_send[5] = (unsigned char)(short)Trunk_Chassis.wheel_Out[2];
	msg_send[6] = (unsigned char)((short)Trunk_Chassis.wheel_Out[1] >> 8);
	msg_send[7] = (unsigned char)(short)Trunk_Chassis.wheel_Out[1];	
	
	CANx_SendData(&hcan2,0x1ff,msg_send,8);	
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
