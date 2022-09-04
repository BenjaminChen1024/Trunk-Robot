/**
  ******************************************************************************
  * @file   : System_Datapool.h
  * @brief  : System_Datapool.cpp头文件
  ****************************************************************************** 
**/
#ifndef _SYSTEM_DATAPOOL_H_
#define _SYSTEM_DATAPOOL_H_
/* Includes ------------------------------------------------------------------*/
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <stm32f4xx.h>
#include "stm32f4xx_hal.h"
#include <SRML.h>
#include "UpperMonitor.h"
#include "main.h"
#include "drv_flash.h"
#include "chassis.h"
#include "Service_Communication.h"
#include "Service_Trunk.h"
#include "Service_Chassis.h"
#include "Service_LED.h"
#include "Service_PWM.h"

/* Private define ------------------------------------------------------------*/

/* 任务堆栈深度 --------------*/
#define Tiny_Stack_Size 64
#define Small_Stack_Size 128
#define Normal_Stack_Size 256
#define Large_Stack_Size 512
#define Huge_Stack_Size 1024
/* 任务优先级 --------------*/
#define PriorityVeryLow 1
#define PriorityLow 2
#define PriorityBelowNormal 3
#define PriorityNormal 4
#define PriorityAboveNormal 5
#define PriorityHigh 6
#define PriorityVeryHigh 7
/* 串口接收数组大小 --------------*/
#define USART1_RX_BUFFER_SIZE 64
#define USART3_RX_BUFFER_SIZE 64
#define USART6_RX_BUFFER_SIZE 64


/* Private variables ---------------------------------------------------------*/

enum MCU_Model_Typedef
{
	MCU_Normal_Model = 1,
	MCU_SetID_Model,
	MCU_Failure_Model,
	MCU_Adjust_Model,
};

enum FlashStatus_ECPU_ID_Typedef
{
	Flash_Waiting = 0,
	Write_ECPU_ID,
	Read_ECPU_ID,
};

enum KeyTypedef
{
	None,
	ABC,
	SHIFT_ABC,
	CTRL_ABC,
	CTRL_SHIRT_ABC,
	Mouse_L_ABC,
	Mouse_R_ABC,
};

struct KeyFlag_Typedef
{
	bool _Z_KeyFlag;
	bool _Z_Shift_KeyFlag;
	bool _Z_Ctrl_KeyFlag;
	bool _X_KeyFlag;
	bool _X_Shift_KeyFlag;
	bool _X_Ctrl_KeyFlag;
	bool _C_KeyFlag;
	bool _C_Shift_KeyFlag;
	bool _C_Ctrl_KeyFlag;
	bool _V_KeyFlag;
	bool _V_Shift_KeyFlag;
	bool _V_Ctrl_KeyFlag;
	bool _B_KeyFlag;
	bool _B_Shift_KeyFlag;
	bool _B_Ctrl_KeyFlag;
	bool _A_KeyFlag;
	bool _A_Shift_KeyFlag;
	bool _A_Ctrl_KeyFlag;
	bool _S_KeyFlag;
	bool _S_Shift_KeyFlag;
	bool _S_Ctrl_KeyFlag;
	bool _D_KeyFlag;
	bool _D_Shift_KeyFlag;
	bool _D_Ctrl_KeyFlag;
	bool _F_KeyFlag;
	bool _F_Shift_KeyFlag;
	bool _F_Ctrl_KeyFlag;
	bool _G_KeyFlag;
	bool _G_Shift_KeyFlag;
	bool _G_Ctrl_KeyFlag;
	bool _Q_KeyFlag;
	bool _Q_Shift_KeyFlag;
	bool _Q_Ctrl_KeyFlag;
	bool _W_KeyFlag;
	bool _W_Shift_KeyFlag;
	bool _W_Ctrl_KeyFlag;
	bool _E_KeyFlag;
	bool _E_Shift_KeyFlag;
	bool _E_Ctrl_KeyFlag;
	bool _R_KeyFlag;
	bool _R_Shift_KeyFlag;
	bool _R_Ctrl_KeyFlag;

	bool _Mouse_L_KeyFlag;
	bool _Mouse_L_Shift_KeyFlag;
	bool _Mouse_L_Ctrl_KeyFlag;
	bool _Mouse_R_KeyFlag;
	bool _Mouse_R_Shift_KeyFlag;
	bool _Mouse_R_Ctrl_KeyFlag;
};

class Motor_Classdef;
/* 类 --------------*/
extern Motor_Classdef Trunk_Motor;
extern DR16_Classdef Trunk_DR16;
/* 串口接收数组 --------------*/
extern uint8_t Uart1_Rx_Buff[USART1_RX_BUFFER_SIZE];
extern uint8_t Uart3_Rx_Buff[USART3_RX_BUFFER_SIZE];
extern uint8_t Uart6_Rx_Buff[USART6_RX_BUFFER_SIZE];

extern uint32_t ECPU_ID;

extern uint8_t exit_flag;
extern uint8_t rising_falling_flag;

extern bool SystemReset_flag;

/* HAL Handlers --------------------------------------------------------------*/
/* 计时器句柄 --------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
/* 串口句柄 --------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* CAN句柄 --------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
/* 消息队列句柄 --------------*/
extern QueueHandle_t DR16_RX_QueueHandle;
extern QueueHandle_t PC_RX_QueueHandle;
/* Private class declarations ---------------------------------------------*/


/* Private function prototypes -------------------------------------------------------*/

/* 限位模板函数 --------------*/
template<typename Type>
Type myConstrain(Type input, Type min, Type max)
{
  if (input <= min)
    return min;
  else if (input >= max)
    return max;
  else
    return input;
}

void FlashWrite_Data();
void FlashRead_Data();

MCU_Model_Typedef Get_MCU_Model();
void Set_MCU_Model(MCU_Model_Typedef);
uint8_t Get_Board_ID();

bool Set_MCUID();

void Keyboard_Control();
KeyTypedef KeyFlag_judge(int key);

#endif
