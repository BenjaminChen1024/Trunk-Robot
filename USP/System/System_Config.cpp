/**
  ******************************************************************************
  * @file   System_Config.cpp
  * @brief  创建初始化配置、通信回调函数
  ******************************************************************************
  * @note
  *  - 
**/
/* Includes ------------------------------------------------------------------*/
#include "System_Config.h"
#include "System_Datapool.h"
/* Private function declarations ---------------------------------------------*/
uint32_t User_UART1_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
uint32_t User_UART3_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
uint32_t User_UART6_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);

void User_CAN1_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage);
void User_CAN2_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage);

/* Private Function prototypes -------------------------------------------------------*/

/**
* @brief  系统资源初始化函数.
* @param  void.
* @return void.
* @note  	None.
*/
void System_Resource_Init(void)
{
	FlashRead_Data();
	/* 计时器初始化 --------------*/
	Timer_Init(&htim1,USE_HAL_DELAY);
	Timer_Init(&htim4,USE_HAL_DELAY);
	Timer_Init(&htim8,USE_HAL_DELAY);
	/* 串口初始化 --------------*/
  Uart_Init(&huart1, Uart1_Rx_Buff, USART1_RX_BUFFER_SIZE, User_UART1_RxCpltCallback);
  Uart_Init(&huart3, Uart3_Rx_Buff, USART3_RX_BUFFER_SIZE, User_UART3_RxCpltCallback);
  Uart_Init(&huart6, Uart6_Rx_Buff, USART6_RX_BUFFER_SIZE, User_UART6_RxCpltCallback);

	/* CAN初始化 --------------*/
  CAN_Init(&hcan1, User_CAN1_RxCpltCallback);
  CAN_Init(&hcan2, User_CAN2_RxCpltCallback);
	/* CAN过滤器初始化 --------------*/
	
	CAN_Filter_Mask_Config(&hcan1,CanFilter_0|CanFifo_0|Can_STDID|Can_DataType,TrunkMotor1_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_1|CanFifo_0|Can_STDID|Can_DataType,TrunkMotor2_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_2|CanFifo_0|Can_STDID|Can_DataType,TrunkMotor3_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_3|CanFifo_0|Can_STDID|Can_DataType,TrunkMotor4_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_4|CanFifo_0|Can_STDID|Can_DataType,YawMotor_ID,0x3ff);

	CAN_Filter_Mask_Config(&hcan2,CanFilter_15|CanFifo_1|Can_STDID|Can_DataType,ChassisMotor1_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan2,CanFilter_16|CanFifo_1|Can_STDID|Can_DataType,ChassisMotor2_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan2,CanFilter_17|CanFifo_1|Can_STDID|Can_DataType,ChassisMotor3_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan2,CanFilter_18|CanFifo_1|Can_STDID|Can_DataType,ChassisMotor4_ID,0x3ff);
	
		/* 创建消息队列 --------------*/
	DR16_RX_QueueHandle = xQueueCreate(4, sizeof(DR16_DataPack_Typedef));
  PC_RX_QueueHandle = xQueueCreate(4, sizeof(DR16_DataPack_Typedef));
	/* 电机初始化 --------------*/
	Trunk_Motor.MotorPIDInit();
	Chassis_ExtInit();
	PWM_Start();
}

/*---------------------------------------------- User RxCpltCallback Function --------------------------------------------*/
uint32_t User_UART1_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  if (PC_RX_QueueHandle != NULL && ReceiveLen == 56)
	{
    return xQueueSendFromISR(PC_RX_QueueHandle, Recv_Data, NULL);
	}
	else
	{
		return 0xFFFFFFFF;
	}

}

uint32_t User_UART3_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
	/* 接收DR16数据 --------------*/
  if (DR16_RX_QueueHandle != NULL && ReceiveLen == 18)
	{
    return xQueueSendFromISR(DR16_RX_QueueHandle, Recv_Data, NULL);
	}
  else
	{
    return 0xFFFFFFFF;
	}
}

uint32_t User_UART6_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{

	return 0xFFFFFFFF;
}

void User_CAN1_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
	Trunk_Motor.Update_CAN1_Motor(CAN_RxMessage->data,CAN_RxMessage->header.StdId);
}

void User_CAN2_RxCpltCallback(CAN_RxBuffer *CAN_RxMessage)
{
	Update_CAN2_Motor(CAN_RxMessage->data,CAN_RxMessage->header.StdId);
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
