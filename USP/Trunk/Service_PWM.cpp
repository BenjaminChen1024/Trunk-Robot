/**
  ******************************************************************************
  * @file   : Service_Motor.cpp
  * @brief  : 服务电机文件
  ******************************************************************************
  * @note
  *  - 
**/

/* Includes ------------------------------------------------------------------*/
#include "Service_PWM.h"
/* Private variables ---------------------------------------------------------*/

uint32_t Claw_PWM = 2400;
uint32_t Finger_PWM = 2500;

/* HAL Handlers --------------------------------------------------------------*/


/* Private function prototypes -------------------------------------------------------*/

/*---------------------------------------------- Universal Motor Function --------------------------------------------*/

void PWM_Start()
{
  HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  HAL_TIM_Base_Start(&htim8);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);

	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, 500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 500);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 500);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 500);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 500);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 500);

}

void PWM_Control()
{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, Claw_PWM);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, Claw_PWM);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, Claw_PWM);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, Claw_PWM);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, Claw_PWM);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, Claw_PWM);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, Claw_PWM);
	vTaskDelay(500);
}

void Finger_Control(float Norm)
{

		Finger_PWM = myConstrain(uint32_t(Finger_PWM + Norm*100), (uint32_t)1000, (uint32_t)2500);
}

void Claw_Control(float Norm)
{
	Claw_PWM = myConstrain(uint32_t(2400 - Norm*400), (uint32_t)2000, (uint32_t)2400);
}

//void Claw_Control_Speed(float Norm, float Speed)
//{
//	if(Norm<0)
//	{}
//	if(Claw_PWM < uint32_t(500 + Norm*300))
//	{
//		Claw_PWM = myConstrain(uint32_t(Claw_PWM + Speed), (uint32_t)500, uint32_t(500 + Norm*300));
//	}
//	if(Claw_PWM > uint32_t(500 + Norm*300))
//	{
//		Claw_PWM = myConstrain(uint32_t(Claw_PWM - Speed), uint32_t(500 + Norm*300), (uint32_t)800);
//	}
//	else
//	{
//	
//	}
//}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
