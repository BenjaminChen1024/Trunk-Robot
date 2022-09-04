/**
  ******************************************************************************
  * @file   : Service_LED.cpp
  * @brief  : 服务通信文件
  ******************************************************************************
  * @note
  *  - 
**/

/* Includes ------------------------------------------------------------------*/
#include "Service_LED.h"
/* Private variables --------------、-------------------------------------------*/

void LED(uint8_t rgb,uint8_t num)
{
//	if(rgb == 1){	LED_R_SET; }			//红
//	if(rgb == 2){	LED_G_SET; }			//绿
//	if(rgb == 3){	LED_B_SET; }			//蓝
//	if(rgb == 4){	LED_RG_SET; }			//黄
//	if(rgb == 5){	LED_RB_SET; }			//紫
//	if(rgb == 6){	LED_GB_SET; }			//青
//	if(rgb == 7){	LED_RGB_SET; }		//白
	
	for(uint8_t i=0;i<num;i++)
	{
		vTaskDelay(500);
		LED_R_SET;
		vTaskDelay(500);
		LED_R_RESET;
	}
	vTaskDelay(5000);
}

/* Private function prototypes -------------------------------------------------------*/


