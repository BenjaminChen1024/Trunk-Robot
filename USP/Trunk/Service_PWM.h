/**
  ******************************************************************************
  * @file   : Service_PWM.h
  * @brief  : Service_PWM.cppÍ·ÎÄ¼þ
  ****************************************************************************** 
**/
#ifndef _SERVICE_PWM_H_
#define _SERVICE_PWM_H_
/* Includes ------------------------------------------------------------------*/
#include "System_DataPool.h"
#include <FreeRTOS.h>
/* Private defines ------------------------------------------------------------*/


/* Private variables ---------------------------------------------------------*/

void PWM_Start();
void PWM_Control();
void Finger_Control(float);
void Claw_Control(float);
void Claw_Control_Speed(float, float);


/* Private function declarations ---------------------------------------------*/

/*---------------------------------------------- Universal Motor Class --------------------------------------------*/


#endif
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
