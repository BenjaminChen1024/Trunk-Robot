/**
  ******************************************************************************
  * @file   : Service_LED.cpp
  * @brief  : ����ͨ���ļ�
  ******************************************************************************
  * @note
  *  - 
**/

/* Includes ------------------------------------------------------------------*/
#include "Service_LED.h"
/* Private variables --------------��-------------------------------------------*/

void LED(uint8_t rgb,uint8_t num)
{
//	if(rgb == 1){	LED_R_SET; }			//��
//	if(rgb == 2){	LED_G_SET; }			//��
//	if(rgb == 3){	LED_B_SET; }			//��
//	if(rgb == 4){	LED_RG_SET; }			//��
//	if(rgb == 5){	LED_RB_SET; }			//��
//	if(rgb == 6){	LED_GB_SET; }			//��
//	if(rgb == 7){	LED_RGB_SET; }		//��
	
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


