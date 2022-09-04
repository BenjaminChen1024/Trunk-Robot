/**
  ******************************************************************************
  * @file   : System_Tasks.cpp
  * @brief  : 创建任务.
  ******************************************************************************
  * @note
  *  - 
**/

/* Includes ------------------------------------------------------------------*/
#include "System_Tasks.h"
#include "System_Datapool.h"
/* TaskHandle_t --------------------------------------------------------------*/
TaskHandle_t DR16Com_Handle;
TaskHandle_t ModeSelection_Handle;
TaskHandle_t OfflineCtrl_Handle;
TaskHandle_t PCCom_Handle;
TaskHandle_t MotorCtrl_Handle;
TaskHandle_t PWMCtrl_Handle;
TaskHandle_t DebugSent_Handle;
TaskHandle_t SystemReset_Handle;

/* Private function declarations ---------------------------------------------*/
void DR16_Com(void *arg);
void Mode_Selection(void*arg);
void Offline_Ctrl(void*arg);
void PC_Com(void *arg);
void Motor_Ctrl(void *arg);
void PWM_Ctrl(void *arg);
void Debug_Sent(void*arg);
void System_Reset(void*arg);

/* Task Function prototypes --------------------------------------------------*/

/**
* @brief  任务初始化函数.
* @param  void.
* @return void.
* @note  	注意不要任务重名（不报错，时序会乱）.

*/
void System_Tasks_Init(void)
{
  vTaskSuspendAll();
	
	xTaskCreate(DR16_Com, "DR16_Com", Normal_Stack_Size, NULL, PriorityHigh, &DR16Com_Handle);
//	xTaskCreate(PC_Com,"PC_Com",Small_Stack_Size,NULL,PriorityHigh,&PCCom_Handle);	

	xTaskCreate(Motor_Ctrl,"Motor_Control",Normal_Stack_Size,NULL,PriorityHigh,&MotorCtrl_Handle);
	xTaskCreate(PWM_Ctrl,"PWM_Control",Small_Stack_Size,NULL,PriorityHigh,&PWMCtrl_Handle);
	
	xTaskCreate(Mode_Selection,"Mode_Selection",Normal_Stack_Size,NULL,PriorityHigh,&ModeSelection_Handle);
	xTaskCreate(Offline_Ctrl,"Offline_Ctrl",Normal_Stack_Size,NULL,PriorityHigh,&OfflineCtrl_Handle);

	xTaskCreate(Debug_Sent,"Debug_Sent",Small_Stack_Size,NULL,PriorityHigh,&DebugSent_Handle);
	
//	xTaskCreate(System_Reset,"System_Reset",Small_Stack_Size,NULL,PriorityHigh,&SystemReset_Handle);
	
  if (!xTaskResumeAll())
    taskYIELD();
}

/**
* @brief  DR16通信函数.
* @param  void.
* @return void.
* @note  	None.
*/
void DR16_Com(void *arg)
{
  static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
  static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
  static DR16_DataPack_Typedef _buffer;
  for (;;)
  {
    if (xQueueReceive(DR16_RX_QueueHandle, &_buffer, _xTicksToWait) == pdTRUE)
		{
		  Trunk_DR16.DataCapture(&_buffer);
		}
		vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
  }
}

/**
* @brief  电脑通信函数.
* @param  void.
* @return void.
* @note  	与电脑进行通信.
*/
void PC_Com(void *arg)
{
  static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(5);
  static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
  for (;;)
  {		
//		if (xQueueReceive(PC_RX_QueueHandle, &_buffer, _xTicksToWait) == pdTRUE)
//		{
//			Update_USART_PC(&_buffer);
//		}
	}
}

/**
* @brief  电机控制函数.
* @param  void.
* @return void.
* @note  	.
*/

void Motor_Ctrl(void *arg)
{
  static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
  for (;;)
  {
		vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
		Set_TargetVelocity();
		Trunk_Chassis.Chassis_Control();
		Motor_Send();
		Trunk_Motor.My_Motor_Control();
  }
}

/**
* @brief  舵机PWM控制函数.
* @param  void.
* @return void.
* @note  	.
*/

void PWM_Ctrl(void *arg)
{
  static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
  for (;;)
  {
		vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);

		PWM_Control();

  }
}

/**
* @brief  模式切换函数.
* @param  void.
* @return void.
* @note  	根据DR16开关切换不同模式.
*/
void Mode_Selection(void*arg)
{
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
  for (;;)
  {
		vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
		Trunk_DR16.Check_Link(xTaskGetTickCount());
    if (Trunk_DR16.GetStatus() == Connection_Lost)
		{
			LED_R_SET;
			xTaskNotify(OfflineCtrl_Handle, (uint32_t)NULL, eNoAction);
		}
    else
    {
			LED_G_SET;
			
			float LX_Norm = Trunk_DR16.Get_LX_Norm();
			float LY_Norm = Trunk_DR16.Get_LY_Norm();
			float RX_Norm = Trunk_DR16.Get_RX_Norm();
			float RY_Norm = Trunk_DR16.Get_RY_Norm();
			
			switch(Trunk_DR16.GetS1())
			{
				case UP:				
					switch(Trunk_DR16.GetS2())
					{
						case UP:
							Trunk_Motor.Trunk_Ctrl_X(LX_Norm);
							Trunk_Motor.Trunk_Ctrl_Y(LY_Norm);
							Trunk_Motor.Trunk_Ctrl_M(RX_Norm,RY_Norm);
							break;
						
						case MID:
							Trunk_Motor.Trunk_Ctrl(LX_Norm, LY_Norm);
							Trunk_Motor.Yaw_Ctrl(RX_Norm);
							break;
						
						case DOWN:
							if(LX_Norm == 0 && LY_Norm == 0 && RX_Norm == 0)
							{
								TargetVelocity_X = 0;
								TargetVelocity_Y = 0;
								TargetVelocity_Z = 0;
							}
							else
							{
								TargetVelocity_X=LX_Norm*-0.6f;
								TargetVelocity_Y=LY_Norm*-0.6f;
								TargetVelocity_Z=RX_Norm*0.4f;
							}
							break;
						
						default:
							break;
					}	
					break;

				case MID:

				break;
				
				case DOWN:
					Keyboard_Control();
					break;
				
				default:
					break;
			}

		}
	}
}

float Yaw_flag=0;
/**
* @brief  掉线保护函数.
* @param  void.
* @return void.
* @note  	暂时没用，先保留接口.
*/
void Offline_Ctrl(void*arg)
{
  static TickType_t _xTicksToWait = pdMS_TO_TICKS(1);
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
	
  for (;;)
  {
    if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, NULL, _xTicksToWait) == pdTRUE)
		{
      TargetVelocity_X =0;
			TargetVelocity_Y =0;
			TargetVelocity_Z =0;
			
			Trunk_Motor.Trunk_Protect();
    }
		vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
  }
}

/**
* @brief  上位机发送函数.
* @param  void.
* @return void.
* @note  	None.
*/
void Debug_Sent(void*arg)
{
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(100);
	for (;;)
  {
		vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);

		Sent_Contorl(&huart6);
	}
}

void System_Reset(void*arg)
{
	static TickType_t _xPreviousWakeTime = xTaskGetTickCount();
  static TickType_t _xTimeIncrement = pdMS_TO_TICKS(1);
	static uint32_t delay_time;
	for (;;)
  {
		if(SystemReset_flag == 1)
		{
			delay_time++;
			if(delay_time == 1000)
			{
				__set_FAULTMASK(1);
				HAL_NVIC_SystemReset();
				delay_time = 0;
				SystemReset_flag = 0;
			}
		}
		vTaskDelayUntil(&_xPreviousWakeTime, _xTimeIncrement);
	}
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
