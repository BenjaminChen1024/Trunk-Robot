/**
  ******************************************************************************
  * @file   System_Config.cpp
  * @brief  ������ʼ�����á�ͨ�Żص�����
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
* @brief  ϵͳ��Դ��ʼ������.
* @param  void.
* @return void.
* @note  	None.
*/
void System_Resource_Init(void)
{
	FlashRead_Data();
	/* ��ʱ����ʼ�� --------------*/
	Timer_Init(&htim1,USE_HAL_DELAY);
	Timer_Init(&htim4,USE_HAL_DELAY);
	Timer_Init(&htim8,USE_HAL_DELAY);
	/* ���ڳ�ʼ�� --------------*/
  Uart_Init(&huart1, Uart1_Rx_Buff, USART1_RX_BUFFER_SIZE, User_UART1_RxCpltCallback);
  Uart_Init(&huart3, Uart3_Rx_Buff, USART3_RX_BUFFER_SIZE, User_UART3_RxCpltCallback);
  Uart_Init(&huart6, Uart6_Rx_Buff, USART6_RX_BUFFER_SIZE, User_UART6_RxCpltCallback);

	/* CAN��ʼ�� --------------*/
  CAN_Init(&hcan1, User_CAN1_RxCpltCallback);
  CAN_Init(&hcan2, User_CAN2_RxCpltCallback);
	/* CAN��������ʼ�� --------------*/
	
	CAN_Filter_Mask_Config(&hcan1,CanFilter_0|CanFifo_0|Can_STDID|Can_DataType,TrunkMotor1_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_1|CanFifo_0|Can_STDID|Can_DataType,TrunkMotor2_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_2|CanFifo_0|Can_STDID|Can_DataType,TrunkMotor3_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_3|CanFifo_0|Can_STDID|Can_DataType,TrunkMotor4_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan1,CanFilter_4|CanFifo_0|Can_STDID|Can_DataType,YawMotor_ID,0x3ff);

	CAN_Filter_Mask_Config(&hcan2,CanFilter_15|CanFifo_1|Can_STDID|Can_DataType,ChassisMotor1_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan2,CanFilter_16|CanFifo_1|Can_STDID|Can_DataType,ChassisMotor2_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan2,CanFilter_17|CanFifo_1|Can_STDID|Can_DataType,ChassisMotor3_ID,0x3ff);
	CAN_Filter_Mask_Config(&hcan2,CanFilter_18|CanFifo_1|Can_STDID|Can_DataType,ChassisMotor4_ID,0x3ff);
	
		/* ������Ϣ���� --------------*/
	DR16_RX_QueueHandle = xQueueCreate(4, sizeof(DR16_DataPack_Typedef));
  PC_RX_QueueHandle = xQueueCreate(4, sizeof(DR16_DataPack_Typedef));
	/* �����ʼ�� --------------*/
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
	/* ����DR16���� --------------*/
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
