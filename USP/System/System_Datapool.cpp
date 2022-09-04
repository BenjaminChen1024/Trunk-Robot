/**
  ******************************************************************************
  * @file   : System_Datapool.cpp
  * @brief  : 包含所有用户资源（变量、函数）
  ******************************************************************************
  * @note
  *  - 
**/
/* Includes ------------------------------------------------------------------*/
#include "System_Datapool.h"
#include "System_Config.h"
/* Private variables ---------------------------------------------------------*/

/* 类 --------------*/
DR16_Classdef Trunk_DR16;
Motor_Classdef Trunk_Motor;
KeyFlag_Typedef KeyFlag;
/* 串口接收数组 --------------*/
uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];
uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];
uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE];

uint32_t ECPU_ID = 0x100;
uint32_t CPU_ID = 1;
uint32_t MainCtrlCPU_ID;

uint8_t exit_flag = 0;
uint8_t rising_falling_flag;

FlashStatus_ECPU_ID_Typedef FlashStatus;
MCU_Model_Typedef MCU_Model = MCU_Normal_Model;
uint32_t Set_ID_Time;

bool SystemReset_flag;

/* HAL Handlers --------------------------------------------------------------*/
QueueHandle_t DR16_RX_QueueHandle;
QueueHandle_t PC_RX_QueueHandle;
/* Private function prototypes -------------------------------------------------------*/

void FlashWrite_Data()
{
		uint32_t writedata[8];	
		flash_erase_address(ADDR_FLASH_SECTOR_11, 8);
//		writedata[0] = ECPU_ID;
//		writedata[1] = TrunkAdjust_DataPack.TrunkMotor1_Angle;
//		writedata[2] = TrunkAdjust_DataPack.TrunkMotor2_Angle;
//		writedata[3] = TrunkAdjust_DataPack.TrunkMotor3_Angle;
//		writedata[4] = TrunkAdjust_DataPack.TrunkMotor4_Angle;
//		writedata[5] = TrunkAdjust_DataPack.TrunkMotor5_Angle;
//		writedata[6] = TrunkAdjust_DataPack.TrunkMotor6_Angle;
//		writedata[7] = TrunkAdjust_DataPack.TrunkMotor7_Angle;

		flash_write_single_address(ADDR_FLASH_SECTOR_11, (uint32_t *)writedata, 8);
	
		SystemReset_flag = 1;
}

void FlashRead_Data()
{
		uint32_t readdata[8];
		flash_read(ADDR_FLASH_SECTOR_11, (uint32_t *)readdata, 8);
		ECPU_ID = readdata[0];
//		TrunkAdjust_DataPack.TrunkMotor1_Angle = readdata[1];
//		TrunkAdjust_DataPack.TrunkMotor2_Angle = readdata[2];
//		TrunkAdjust_DataPack.TrunkMotor3_Angle = readdata[3];
//		TrunkAdjust_DataPack.TrunkMotor4_Angle = readdata[4];
//		TrunkAdjust_DataPack.TrunkMotor5_Angle = readdata[5];
//		TrunkAdjust_DataPack.TrunkMotor6_Angle = readdata[6];
//		TrunkAdjust_DataPack.TrunkMotor7_Angle = readdata[7];
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == KEY_Pin)
    {
			MCU_Model = MCU_SetID_Model;
			if(exit_flag == 0)
			{
				exit_flag = 1;
				rising_falling_flag = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
			}
    }
}

void Keyboard_Control()
{
	static float Speed_Gear=0.5;
	static float Mouse_Sensitivity_Gear=0.8;
	static float  Claw_Control_flag = 0;

	if(KeyFlag_judge(_W) == ABC){TargetVelocity_Y = -Speed_Gear;}												/*前*/
	if(KeyFlag_judge(_S) == ABC){TargetVelocity_Y = Speed_Gear;}                				/*后*/
	if(KeyFlag_judge(_W) != ABC && KeyFlag_judge(_S) != ABC){TargetVelocity_Y = 0;}
	if(KeyFlag_judge(_A) == ABC){TargetVelocity_X = Speed_Gear;}					              /*左*/
	if(KeyFlag_judge(_D) == ABC){TargetVelocity_X = -Speed_Gear;}												/*右*/
	if(KeyFlag_judge(_A) != ABC && KeyFlag_judge(_D) != ABC){TargetVelocity_X = 0;}
	TargetVelocity_Z = Trunk_DR16.Get_MouseX_Norm() * Mouse_Sensitivity_Gear;    /*旋转*/
		
	if(KeyFlag_judge(_W) == Mouse_L_ABC){Trunk_Motor.Trunk_Ctrl_Y(0.5);}
	if(KeyFlag_judge(_S) == Mouse_L_ABC){Trunk_Motor.Trunk_Ctrl_Y(-0.5);}
	if(KeyFlag_judge(_A) == Mouse_L_ABC){Trunk_Motor.Trunk_Ctrl_X(-0.5);}
	if(KeyFlag_judge(_D) == Mouse_L_ABC){Trunk_Motor.Trunk_Ctrl_X(0.5);}
	
	if(KeyFlag_judge(_W) == Mouse_R_ABC){Trunk_Motor.Trunk_Ctrl_M(0,0.5);}
	if(KeyFlag_judge(_S) == Mouse_R_ABC){Trunk_Motor.Trunk_Ctrl_M(0,-0.5);}
	if(KeyFlag_judge(_A) == Mouse_R_ABC){Trunk_Motor.Trunk_Ctrl_M(-0.5,0);}
	if(KeyFlag_judge(_D) == Mouse_R_ABC){Trunk_Motor.Trunk_Ctrl_M(0.5,0);}
	
	if(KeyFlag_judge(_Q) == ABC){Trunk_Motor.Yaw_Ctrl(-0.5);}
	if(KeyFlag_judge(_E) == ABC){Trunk_Motor.Yaw_Ctrl(0.5);}
	
	if(KeyFlag._Z_KeyFlag && !Trunk_DR16.IsKeyPress(_Z)){KeyFlag._Z_KeyFlag = 0;}
	if(KeyFlag._Z_Shift_KeyFlag && !Trunk_DR16.IsKeyPress(_Z)){KeyFlag._Z_Shift_KeyFlag = 0;}
	if(KeyFlag._Z_Ctrl_KeyFlag && !Trunk_DR16.IsKeyPress(_Z)){KeyFlag._Z_Ctrl_KeyFlag = 0;}

	if(KeyFlag._X_KeyFlag && !Trunk_DR16.IsKeyPress(_X)){KeyFlag._X_KeyFlag = 0;}
	if(KeyFlag._X_Shift_KeyFlag && !Trunk_DR16.IsKeyPress(_X)){KeyFlag._X_Shift_KeyFlag = 0;}
	if(KeyFlag._X_Ctrl_KeyFlag && !Trunk_DR16.IsKeyPress(_X)){KeyFlag._X_Ctrl_KeyFlag = 0;}

	if(KeyFlag._C_KeyFlag && !Trunk_DR16.IsKeyPress(_C)){KeyFlag._C_KeyFlag = 0;}
	if(KeyFlag._C_Shift_KeyFlag && !Trunk_DR16.IsKeyPress(_C)){KeyFlag._C_Shift_KeyFlag = 0;}
	if(KeyFlag._C_Ctrl_KeyFlag && !Trunk_DR16.IsKeyPress(_C)){KeyFlag._C_Ctrl_KeyFlag = 0;}
		
	if(KeyFlag._V_KeyFlag && !Trunk_DR16.IsKeyPress(_V)){KeyFlag._V_KeyFlag = 0;}
	if(KeyFlag._V_Shift_KeyFlag && !Trunk_DR16.IsKeyPress(_V)){KeyFlag._V_Shift_KeyFlag = 0;}
	if(KeyFlag._V_Ctrl_KeyFlag && !Trunk_DR16.IsKeyPress(_V)){KeyFlag._V_Ctrl_KeyFlag = 0;}
	
	if(KeyFlag._B_KeyFlag && !Trunk_DR16.IsKeyPress(_B)){KeyFlag._B_KeyFlag = 0;}
	if(KeyFlag._B_Shift_KeyFlag && !Trunk_DR16.IsKeyPress(_B)){KeyFlag._B_Shift_KeyFlag = 0;}
	if(KeyFlag._B_Ctrl_KeyFlag && !Trunk_DR16.IsKeyPress(_B)){KeyFlag._B_Ctrl_KeyFlag = 0;}
	
	if(KeyFlag._F_KeyFlag && !Trunk_DR16.IsKeyPress(_F)){Claw_Control_flag = !Claw_Control_flag; Claw_Control(Claw_Control_flag);KeyFlag._F_KeyFlag = 0;}
	if(KeyFlag_judge(_F) == SHIFT_ABC){}
	if(KeyFlag_judge(_F) == CTRL_ABC){}
			
	if(KeyFlag._E_KeyFlag && !Trunk_DR16.IsKeyPress(_E)){KeyFlag._E_KeyFlag = 0;}
	if(KeyFlag._E_Shift_KeyFlag && !Trunk_DR16.IsKeyPress(_E)){KeyFlag._E_Shift_KeyFlag = 0;}
	if(KeyFlag._E_Ctrl_KeyFlag && !Trunk_DR16.IsKeyPress(_E)){KeyFlag._E_Ctrl_KeyFlag = 0;}

	if(KeyFlag._R_KeyFlag && !Trunk_DR16.IsKeyPress(_R)){KeyFlag._R_KeyFlag = 0;}
	if(KeyFlag._R_Shift_KeyFlag && !Trunk_DR16.IsKeyPress(_R)){KeyFlag._R_Shift_KeyFlag = 0;}
	if(KeyFlag._R_Ctrl_KeyFlag && !Trunk_DR16.IsKeyPress(_R)){KeyFlag._R_Ctrl_KeyFlag = 0;}
	
	if(KeyFlag._Q_KeyFlag && !Trunk_DR16.IsKeyPress(_Q)){KeyFlag._Q_KeyFlag = 0;}
	if(KeyFlag._Q_Shift_KeyFlag && !Trunk_DR16.IsKeyPress(_Q)){KeyFlag._Q_Shift_KeyFlag = 0;}
	if(KeyFlag._Q_Ctrl_KeyFlag && !Trunk_DR16.IsKeyPress(_Q)){KeyFlag._Q_Ctrl_KeyFlag = 0;}
	
	if(KeyFlag._Mouse_L_KeyFlag && !Trunk_DR16.IsKeyPress(_Mouse_L)){KeyFlag._Mouse_L_KeyFlag = 0;}
	if(KeyFlag._Mouse_L_Shift_KeyFlag && !Trunk_DR16.IsKeyPress(_Mouse_L)){KeyFlag._Mouse_L_Shift_KeyFlag = 0;}
	if(KeyFlag._Mouse_L_Ctrl_KeyFlag && !Trunk_DR16.IsKeyPress(_Mouse_L)){KeyFlag._Mouse_L_Ctrl_KeyFlag = 0;}
	
	if(KeyFlag._Mouse_R_KeyFlag && !Trunk_DR16.IsKeyPress(_Mouse_R)){KeyFlag._Mouse_R_KeyFlag = 0;}
}

void Set_KeyFlag()
{
	if(KeyFlag_judge(_Z) == ABC){KeyFlag._Z_KeyFlag = 1;}
	if(KeyFlag_judge(_Z) == SHIFT_ABC){KeyFlag._Z_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_Z) == CTRL_ABC){KeyFlag._Z_Ctrl_KeyFlag = 1;}

	if(KeyFlag_judge(_X) == ABC){KeyFlag._X_KeyFlag = 1;}
	if(KeyFlag_judge(_X) == SHIFT_ABC){KeyFlag._X_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_X) == CTRL_ABC){KeyFlag._X_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_C) == ABC){KeyFlag._C_KeyFlag = 1;}
	if(KeyFlag_judge(_C) == SHIFT_ABC){KeyFlag._C_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_C) == CTRL_ABC){KeyFlag._C_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_V) == ABC){KeyFlag._V_KeyFlag = 1;}
	if(KeyFlag_judge(_V) == SHIFT_ABC){KeyFlag._V_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_V) == CTRL_ABC){KeyFlag._V_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_B) == ABC){KeyFlag._B_KeyFlag = 1;}
	if(KeyFlag_judge(_B) == SHIFT_ABC){KeyFlag._B_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_B) == CTRL_ABC){KeyFlag._B_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_A) == ABC){KeyFlag._A_KeyFlag = 1;}
	if(KeyFlag_judge(_A) == SHIFT_ABC){KeyFlag._A_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_A) == CTRL_ABC){KeyFlag._A_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_S) == ABC){KeyFlag._S_KeyFlag = 1;}
	if(KeyFlag_judge(_S) == SHIFT_ABC){KeyFlag._S_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_S) == CTRL_ABC){KeyFlag._S_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_D) == ABC){KeyFlag._D_KeyFlag = 1;}
	if(KeyFlag_judge(_D) == SHIFT_ABC){KeyFlag._D_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_D) == CTRL_ABC){KeyFlag._D_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_F) == ABC){KeyFlag._F_KeyFlag = 1;}
	if(KeyFlag_judge(_F) == SHIFT_ABC){KeyFlag._F_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_F) == CTRL_ABC){KeyFlag._F_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_G) == ABC){KeyFlag._G_KeyFlag = 1;}
	if(KeyFlag_judge(_G) == SHIFT_ABC){KeyFlag._G_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_G) == CTRL_ABC){KeyFlag._G_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_Q) == ABC){KeyFlag._Q_KeyFlag = 1;}
	if(KeyFlag_judge(_Q) == SHIFT_ABC){KeyFlag._Q_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_Q) == CTRL_ABC){KeyFlag._Q_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_W) == ABC){KeyFlag._W_KeyFlag = 1;}
	if(KeyFlag_judge(_W) == SHIFT_ABC){KeyFlag._W_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_W) == CTRL_ABC){KeyFlag._W_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_E) == ABC){KeyFlag._E_KeyFlag = 1;}
	if(KeyFlag_judge(_E) == SHIFT_ABC){KeyFlag._E_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_E) == CTRL_ABC){KeyFlag._E_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_R) == ABC){KeyFlag._R_KeyFlag = 1;}
	if(KeyFlag_judge(_R) == SHIFT_ABC){KeyFlag._R_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_R) == CTRL_ABC){KeyFlag._R_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_Mouse_L) == ABC){KeyFlag._Mouse_L_KeyFlag = 1;}
	if(KeyFlag_judge(_Mouse_L) == SHIFT_ABC){KeyFlag._Mouse_L_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_Mouse_L) == CTRL_ABC){KeyFlag._Mouse_L_Ctrl_KeyFlag = 1;}
	
	if(KeyFlag_judge(_Mouse_R) == ABC){KeyFlag._Mouse_R_KeyFlag = 1;}
	if(KeyFlag_judge(_Mouse_R) == SHIFT_ABC){KeyFlag._Mouse_R_Shift_KeyFlag = 1;}
	if(KeyFlag_judge(_Mouse_R) == CTRL_ABC){KeyFlag._Mouse_R_Ctrl_KeyFlag = 1;}
}

KeyTypedef KeyFlag_judge(int key)
{
	if(Trunk_DR16.IsKeyPress(key)&&!Trunk_DR16.IsKeyPress(_SHIFT)&&!Trunk_DR16.IsKeyPress(_CTRL)&&Trunk_DR16.IsKeyPress(_Mouse_L)&&!Trunk_DR16.IsKeyPress(_Mouse_R))
	{
		return Mouse_L_ABC;
	}
	if(Trunk_DR16.IsKeyPress(key)&&Trunk_DR16.IsKeyPress(_SHIFT)&&!Trunk_DR16.IsKeyPress(_CTRL)&&!Trunk_DR16.IsKeyPress(_Mouse_L)&&Trunk_DR16.IsKeyPress(_Mouse_R))
	{
		return Mouse_R_ABC;
	}
	if(Trunk_DR16.IsKeyPress(key)&&Trunk_DR16.IsKeyPress(_SHIFT)&&!Trunk_DR16.IsKeyPress(_CTRL)&&!Trunk_DR16.IsKeyPress(_Mouse_L)&&!Trunk_DR16.IsKeyPress(_Mouse_R))
	{
		return SHIFT_ABC;
	}
	else if(Trunk_DR16.IsKeyPress(key)&&!Trunk_DR16.IsKeyPress(_SHIFT)&&Trunk_DR16.IsKeyPress(_CTRL)&&!Trunk_DR16.IsKeyPress(_Mouse_L)&&!Trunk_DR16.IsKeyPress(_Mouse_R))
	{
		return CTRL_ABC;
	}
	else if(Trunk_DR16.IsKeyPress(key)&&!Trunk_DR16.IsKeyPress(_SHIFT)&&!Trunk_DR16.IsKeyPress(_CTRL)&&!Trunk_DR16.IsKeyPress(_Mouse_L)&&!Trunk_DR16.IsKeyPress(_Mouse_R))
	{
		return ABC;
	}
	else
	{
		return None;
	}
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
