/**
  ******************************************************************************
  * @file   ： Module_BoardCOM.cpp
  * @brief  ： 多块板的通信协议(CANopen)
  * @author ： 陈佩奇
  * @date   ： 11,2019
  ******************************************************************************
  ==============================================================================
                     ##### How to use this module #####
  ==============================================================================
  1)引入CANopen，通过对象字典实现指令，支持SDO和PDO通信；
	2)SDO与PDO通信的区别：SDO为双向通信，数据段包含对象地址和数据，每次SDO通信包含发送
		和回传两个帧；PDO为单向通信，数据段完全用来传输数据，效率高；
	3)预定义的PDO共有4个；
	4)建立PDO通信的流程：
		- 设置主站PDO1参数；
		- 主站进入预处理阶段，通过SDO配置PDO1的通信参数；
		- 关闭从站的PDO2，PDO3，PDO4；
		- 发送START指令，从站开始运行，此时通信建立；
	5)有可靠传输需求的地方，使用SDO通信实现，利用定时发送的SDO帧实现离线监测；
	6)使用CANopen标准规定的对象字典地址段，所有数据均规定了对象字典地址；如有新增数据，
		请参考标准对象字典地址段进行预定义，并在对象字典表中进行更新并说明记录：
		(规定的对象字典地址段)
		---------------------------------
		| 地址        | 变量入口 				|
		---------------------------------
		| 0000h       | 保留   					|
		| 0001h-009Fh | 保留   					|
		| 00A0h-0FFFh | 保留   					|
		| 1000h-1FFFh | 通信参数	   		|
		| 2000h-5FFFh | 实验室指定参数	|
		| 6000h-9FFFh | 标准设备区域    |
		| A000h-FFFFh | 保留						|
		---------------------------------

  ******************************************************************************
  */

#include "Module_BoardCOM.h"
#include "FreeRTOS.h"
#include "stm32flash.h"
#include "drv_can.h"



void* Object_Dictionary[OD_MAX_INDEXS][OD_MAX_SUBS] = {NULL};

uint8_t Type_Memery[OD_MAX_INDEXS*OD_MAX_SUBS] = {0};

/*不同板下载这份代码的时候记得修改本地的node_id,范围在0-3*/
uint8_t Local_NodeID =0;



uint8_t SDO_Mailbox_Classdef::size()
{
	return sizes;  
}

/**
 * @brief  往邮箱队列后插入新帧
 * @param  input_Frame SDO帧
 * @retval void
 * @author 陈佩奇
 */
void SDO_Mailbox_Classdef::push_back(SDO_Frame_Typedef input_frame)
{
	if(sizes < MAX_MAILBOX_DEEPTH)
	{
		SDO_Mailbox[sizes] = input_frame;
		sizes++;
	}
}

/**
 * @brief  邮箱队列前移
 * @param  void
 * @retval void
 * @author 陈佩奇
 */

void SDO_Mailbox_Classdef::erase()
{
	static SDO_Frame_Typedef clear_frame = {0,0,0,0,0,0};

	if(sizes)
	{
		for(uint8_t i = 0; i < sizes - 1; i++)
		{
			SDO_Mailbox[i] = SDO_Mailbox[i+1];
		}
		SDO_Mailbox[sizes - 1] = clear_frame;
		sizes--;
	}
}

/**
 * @brief  重载[]操作符
 * @param  void
 * @retval SDO_Frame_Typedef
 * @author 陈佩奇
 */
SDO_Frame_Typedef SDO_Mailbox_Classdef::operator [](uint8_t num)
{
	return SDO_Mailbox[num];
}

void SDO_RequireList_Typedef::Register(void* ptr,uint16_t cob_id,uint16_t index,uint8_t sub_index,Object_Type_Enum type)
{
	if(sizes >= MAX_REQUIRE_LIST_DEEPTH)
	{
		return;
	}

	/* 注册表 储存注册的数据来源节点ID和地址 */
	SDO_Frame_Typedef data_config = {cob_id,SDO_ReadRecv,index,sub_index,4};

	/* 注册 */
	register_list[sizes] = ptr;
	require_list[sizes] = data_config;
	type_list[sizes] = type;
	sizes++;

}

uint8_t SDO_RequireList_Typedef::Get_Register_Index(uint16_t cob_id,uint16_t index,uint8_t sub_index)
{
	uint8_t register_index = 255;
	for(size_t i = 0; i < MAX_REQUIRE_LIST_DEEPTH; i++)
	{
		if(require_list[i].COB_ID == cob_id && require_list[i].Index == index && require_list[i].Sub_Index == sub_index)
		{
			register_index = i;
		}
	}
	return register_index;
}

template <typename T> T SDO_RequireList_Typedef::at(uint16_t cob_id,uint16_t index,uint8_t sub_index)
{
	T res;
	T* ptr;

	/* 获取对应数据索引 */
	uint8_t register_index = Get_Register_Index(cob_id,index,sub_index);

	if(register_index < MAX_REQUIRE_LIST_DEEPTH)
	{
		memcpy(&res,&require_list[register_index].Data[4],sizeof(T));

		/* 更新数据 */
		ptr = (T*)register_list[register_index];
		*ptr = res;
	}
	return res;
}

Object_Type_Enum SDO_RequireList_Typedef::Type(uint8_t register_index)
{
	/* 越界 */
	if(register_index >= MAX_REQUIRE_LIST_DEEPTH)
	{
		return FLOAT;
	}
	else
	{
		return type_list[register_index];
	}
}

void SDO_RequireList_Typedef::Update(SDO_Frame_Typedef frame,uint8_t register_index)
{
	require_list[register_index] = frame;
}

/**
 * @brief  往SDO发送邮箱里添加帧
 * @param  SDO_Frame_Typedef SDO_Frame SDO帧
 * @retval uint8_t
 * @author 陈佩奇
 */
uint8_t SDOManager_Classdef::SDO_Add_Message_To_Mailbox(SDO_Frame_Typedef SDO_Frame)
{

	if(SDO_Tx_Mailbox.size() >= MAX_Deepth)
	{
		return ERROR_MAILBOX_FULL;
	}

	SDO_Tx_Mailbox.push_back(SDO_Frame);
	return COM_OK;
}

/**
 * @brief  往SDO发送邮箱里添加帧
 * @param  SDO帧具体参数
 * @retval uint8_t
 * @author 陈佩奇
 */
uint8_t SDOManager_Classdef::SDO_Add_Message_To_Mailbox(
	uint16_t device_id,
	uint8_t ndoe_id,
	SDO_WorkType_Enum work_type,
    uint16_t index,
	uint8_t sub_index,
	uint8_t * data,
    SDO_DataLength_Enum data_length)
{
	if(SDO_Tx_Mailbox.size() >= MAX_Deepth)
	{
		return ERROR_MAILBOX_FULL;
	}

	uint8_t ctrl_byte = Set_Ctrl_Byte(work_type,data_length);

	SDO_Frame_Typedef SDO_Tx_Frame = {
		device_id+ndoe_id, ctrl_byte, index, sub_index, data_length
	};
	/* SDO的数据段只有4个字节，前4个字节为控制数据段 */
	memset(&SDO_Tx_Frame.Data[4],0,4);
	memcpy(&SDO_Tx_Frame.Data[4],&data[0],(uint8_t)data_length);

	/* 设置SDO的控制段 */
	SDO_Tx_Frame.Data[0] = SDO_Tx_Frame.Ctrl_Byte;
	SDO_Tx_Frame.Data[1] = SDO_Tx_Frame.Index & 0xff;//
	SDO_Tx_Frame.Data[2] = (SDO_Tx_Frame.Index >> 8) & 0xff;
	SDO_Tx_Frame.Data[3] = SDO_Tx_Frame.Sub_Index;

	SDO_Tx_Mailbox.push_back(SDO_Tx_Frame);
	return COM_OK;
}



SDOManager_Classdef::SDOManager_Classdef(CAN_HandleTypeDef* hcanx):MAX_Deepth(MAX_MAILBOX_DEEPTH),hcan(hcanx)
{}

/**
 * @brief  SDO接收回调函数
 * @param  CAN包
 * @retval uint8_t
 * @author 陈佩奇
 */

uint8_t SDOManager_Classdef::SDO_Recv_Callback(CAN_RxBuffer* recv_msg,TaskHandle_t TASK_Handle)
{


	SDO_Frame_Typedef SDO_Rx_Frame = {recv_msg->header.StdId,recv_msg->data[0],recv_msg->data[1]+recv_msg->data[2]*256,recv_msg->data[3],SDO_Data_4Byte};
	memcpy(&SDO_Rx_Frame.Data[0],recv_msg->data,8);

	/* 如果为回传信号 通过信号量发送出去 
	   提醒pull or push函数已经初始化成功*/
	if(SDO_Rx_Frame.Ctrl_Byte == SDO_WriteRecv)
	{

		BaseType_t xHigherPriorityTaskWoken;
		vTaskNotifyGiveFromISR(TASK_Handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	}

	SDO_Rx_Mailbox.push_back(SDO_Rx_Frame);
	return 0;
}

/**
 * @brief  SDO协议执行函数 任务调用
 * @param  void
 * @retval void
 * @author 陈佩奇
 */

void SDOManager_Classdef::SDO_Exce_Once()
{
	/* 初始化回传abort code */
	uint8_t abort_code[4] = {0};
	uint8_t read_data[4] = {0};



	/* 清理发送邮箱 */
	if(SDO_Tx_Mailbox.size())
	{
		if(CANx_SendData(hcan,SDO_Tx_Mailbox[0].COB_ID,&SDO_Tx_Mailbox[0].Data[0],8))
		{
			SDO_Tx_Mailbox.erase();
		}
	}

	/* 处理接收邮箱 */
	while(SDO_Rx_Mailbox.size())
	{
		switch(SDO_Rx_Mailbox[0].Ctrl_Byte & 0xf1)
		{
			case SDO_ReadSend:
				if(Object_Read(SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,read_data) == 0)
				{
					SDO_Add_Message_To_Mailbox(DEVICE_ID_RX,Local_NodeID,SDO_ReadRecv,SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,read_data,SDO_Data_4Byte);
				}
				/* 无法读取 */
				else
				{
					Set_Abort_Code(AC_OUT_OF_MEMERORY,abort_code);
					SDO_Add_Message_To_Mailbox(DEVICE_ID_RX,Local_NodeID,SDO_ReadRecv,SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,abort_code,SDO_Data_4Byte);
				}
				break;
			case SDO_ReadRecv:
				/* 更新数据 */
				Update(SDO_Rx_Mailbox[0]);
				break;
			case SDO_WriteSend:

				if(Object_Write(SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,SDO_Rx_Mailbox[0].Data) == 0)
				{
					Set_Abort_Code(AC_NO_ERROR,abort_code);
					SDO_Add_Message_To_Mailbox(DEVICE_ID_RX,Local_NodeID,SDO_WriteRecv,SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,abort_code,SDO_Data_4Byte);
					HAL_Delay(1);
				}
				/* 不可写入 */
				else
				{
					Set_Abort_Code(AC_OUT_OF_MEMERORY,abort_code);
					SDO_Add_Message_To_Mailbox(DEVICE_ID_RX,Local_NodeID,SDO_WriteRecv,SDO_Rx_Mailbox[0].Index,SDO_Rx_Mailbox[0].Sub_Index,abort_code,SDO_Data_4Byte);
				}
				break;
			case SDO_WriteRecv:
				break;

		}
		SDO_Rx_Mailbox.erase();
	}

}

/**
 * @brief  请求读取一个数据
 * @param  地址
 * @retval uint8_t
 * @author 陈佩奇
 */
uint8_t SDOManager_Classdef::Pull(void * ptr,uint8_t node_id,uint16_t index,uint8_t sub_index,Object_Type_Enum type)
{
	uint8_t temp_res[4];
	SDO_Add_Message_To_Mailbox(DEVICE_ID_TX,node_id,SDO_ReadSend,index,sub_index,temp_res,SDO_Data_0Byte);

	if(SDO_RequireList.Get_Register_Index(DEVICE_ID_RX+node_id,index,sub_index) >= MAX_REQUIRE_LIST_DEEPTH)
	{
		/* 注册请求数据 */
		SDO_RequireList.Register(ptr,DEVICE_ID_RX+node_id,index,sub_index,type);
	}
	return 0;
}

/**
 * @brief  请求写一个数据
 * @param  地址
 * @retval uint8_t
 * @author 陈佩奇
 */
uint8_t SDOManager_Classdef::Push
(void * Ptr,uint8_t node_id,uint16_t index,uint8_t sub_index,Object_Type_Enum type)
{
	uint8_t temp_res[4];
	memcpy(&temp_res[0],Ptr,int(type)/3+1);
	SDO_Add_Message_To_Mailbox(DEVICE_ID_TX,node_id,SDO_WriteSend,index,sub_index,temp_res,SDO_DataLength_Enum(int(type)/3+1));
	return 0;
}

/**
 * @brief  更新一个数据
 * @param  SDO数据帧
 * @retval void
 * @author 陈佩奇
 */
void SDOManager_Classdef::Update(SDO_Frame_Typedef frame)
{
	/* 如果该数据已注册 */
	uint8_t register_index = SDO_RequireList.Get_Register_Index(frame.COB_ID,frame.Index,frame.Sub_Index);
	if(register_index < MAX_REQUIRE_LIST_DEEPTH)
	{
		/* 更新数据到注册表 */
		SDO_RequireList.Update(frame,register_index);

		switch (SDO_RequireList.Type(register_index))
		{
			case UINT8:
			SDO_RequireList.at<uint8_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case INT8:
			SDO_RequireList.at<int8_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case UINT16:
			SDO_RequireList.at<uint16_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case INT16:
			SDO_RequireList.at<int16_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case UINT32:
			SDO_RequireList.at<uint32_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case INT32:
			SDO_RequireList.at<int32_t>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		case FLOAT:
			SDO_RequireList.at<float>(frame.COB_ID,frame.Index,frame.Sub_Index);
			break;
		}
	}
}

/**
 * @brief  将对象Map到PDO帧
 * @param  对象数和对象(变长参数)
*  @note   total size为位长度
 * @retval 错误码
 * @author 陈佩奇
 */

uint8_t PDO_Typedef::Map(uint8_t n,uint32_t objects[])
{
	uint8_t total_size = 0;

	uint32_t temp_object[4] = {0};

	for(size_t i = 0; i < n; i++)
	{
		temp_object[i] = objects[i];
		total_size += (temp_object[i]&0xff);
	}

	if(total_size <= 64)
	{
		memcpy(this->Mapped_Objects,temp_object,n*sizeof(uint32_t));
		this->Mapped_Nums = n;

		return COM_OK;
	}

	return ERROR_MAPERROR;

}

/**
 * @brief  获取该PDO Map的对象数
 * 				 等于0则未Map
 * @param  void包
 * @retval 对象数
 * @author 陈佩奇
 */

uint8_t PDO_Typedef::Get_Mapped_Nums()
{
	return this->Mapped_Nums;
}

/**
 * @brief  将RXPDO帧的数据更新到注册的地址中
 * @note   object的组成index(2byte)+subindex(1byte)+bit_size(1byte)
           右移16位的原因是只用到了第二个字节那两位index，第一个字节相同
 * @param  CAN接收数据
 * @retval 错误码
 * @author 陈佩奇
 */

//右移16位的原因是只用到了第二个字节那两位index，第一个字节一般相同
//所以有Addr_Read（）里面的transfer就是为了区分掉第一个字节的不同
uint8_t RXPDO_Typedef::Update(CAN_RxBuffer* recv_msg)
{
	uint8_t bias = 0;
	void* addr = NULL;
	for(size_t i = 0; i < this->Mapped_Nums; i++)
	{
		/* 通过是否注册数据暂存地址判断是主动请求还是写入 */
		if(Register_Buff[i] == NULL)
		{
			/* 写入 */
			addr = Addr_Read(Mapped_Objects[i]>>16,(Mapped_Objects[i]>>8)&0xff);
			memcpy(addr,&recv_msg->data[bias],(Mapped_Objects[i]&0xff)/8);

			if(addr == NULL)
			{
				return ERROR_UNREGISTED;
			}
		}
		else
		{
			/* 主动请求 */
			memcpy(Register_Buff[i],&recv_msg->data[bias],(Mapped_Objects[i]&0xff)/8);
		}
		bias += (Mapped_Objects[i]&0xff)/8;
	}
	return COM_OK;
}

/**
 * @brief  注册RXPDO帧的数据缓存地址
 * @param  地址
 * @retval 错误码
 * @author 陈佩奇
 */
uint8_t RXPDO_Typedef::Regster(uint8_t n,void* Ptr[])
{
	for(size_t i = 0; i < n ; i++)
	{
		Register_Buff[i] = Ptr[i];
	}
	return 0;
}

/**
 * @brief  把数据打包为TXPDO帧
 * @param  PDO帧的COB-ID
 * @retval PDO帧
 * @author 陈佩奇
 */
PDO_Frame_Typedef TXPDO_Typedef::Pack()
{
	PDO_Frame_Typedef res = {0};
	if(Mapped_Nums > 0)
	{
		res.COB_ID = this->COB_ID;
		uint8_t bias = 0;
		for(size_t i = 0; i < Mapped_Nums; i++)
		{
			Object_Read(Mapped_Objects[i]>>16,(Mapped_Objects[i]>>8)&0xff,&res.Data[bias]);//以后的结构体想要从第n位开始copy就这么干“&res.Data[bias]”，此步为读取
			bias += (Mapped_Objects[i]&0xff)/8;
		}
	}
	return res;

}

/**
 * @brief  PDO通信初始化 注册通信参数地址
 * @param  CAN包
 * @retval uint8_t
 * @author 陈佩奇
 */

PDOManager_Classdef::PDOManager_Classdef(CAN_HandleTypeDef* hcanx):hcan(hcanx)
{
	for(size_t i = 0; i < 4; i++)
	{
		/* 通信参数 */
		Object_Dict_Register(&this->RxPDO[i].COB_ID,0x1400+i,0x01,ReadNWrite,UINT32);
		Object_Dict_Register(&this->RxPDO[i].PDO_Type,0x1400+i,0x02,ReadNWrite,UINT8);
		Object_Dict_Register(&this->TxPDO[i].COB_ID,0x1800+i,0x01,ReadNWrite,UINT32);
		Object_Dict_Register(&this->TxPDO[i].PDO_Type,0x1800+i,0x02,ReadNWrite,UINT8);

		Object_Dict_Register(&this->RxPDO[i].Mapped_Nums,0x1600+i,0x00,ReadNWrite,UINT8);
		Object_Dict_Register(&this->RxPDO[i].Mapped_Objects[0],0x1600+i,0x01,ReadNWrite,UINT32);
		Object_Dict_Register(&this->RxPDO[i].Mapped_Objects[1],0x1600+i,0x02,ReadNWrite,UINT32);
		Object_Dict_Register(&this->RxPDO[i].Mapped_Objects[2],0x1600+i,0x03,ReadNWrite,UINT32);
		Object_Dict_Register(&this->RxPDO[i].Mapped_Objects[3],0x1600+i,0x04,ReadNWrite,UINT32);

		Object_Dict_Register(&this->TxPDO[i].Mapped_Nums,0x1A00+i,0x00,ReadNWrite,UINT8);
		Object_Dict_Register(&this->TxPDO[i].Mapped_Objects[0],0x1A00+i,0x01,ReadNWrite,UINT32);
		Object_Dict_Register(&this->TxPDO[i].Mapped_Objects[1],0x1A00+i,0x02,ReadNWrite,UINT32);
		Object_Dict_Register(&this->TxPDO[i].Mapped_Objects[2],0x1A00+i,0x03,ReadNWrite,UINT32);
		Object_Dict_Register(&this->TxPDO[i].Mapped_Objects[3],0x1A00+i,0x04,ReadNWrite,UINT32);
	}
}

/**
* @brief  读取接收到的PDO帧,区分到底是哪个pdo的
 * @param  CAN包
 * @retval uint8_t
 * @author 陈佩奇
 */
void PDOManager_Classdef::PDO_Recv_Callback(CAN_RxBuffer* recv_msg)
{
	switch(recv_msg->header.StdId & 0xf80)
	{
		case 0x180:
			RxPDO[0].Update(recv_msg);
			break;
		case 0x280:
			RxPDO[1].Update(recv_msg);
			break;
		case 0x380:
			RxPDO[2].Update(recv_msg);
			break;
		case 0x480:
			RxPDO[3].Update(recv_msg);
			break;
	}
}

/**
 * @brief  发送所有Map过的PDO帧(一次)
 * @param  void
 * @retval void
 * @author 陈佩奇
 */
void PDOManager_Classdef::PDO_Exce_Once()
{

	PDO_Frame_Typedef tx_frame;
	for(size_t i = 0; i < 4; i++)
	{
		if(TxPDO[i].Get_Mapped_Nums() > 0)
		{
			tx_frame = TxPDO[i].Pack();
			CANx_SendData(hcan,tx_frame.COB_ID,&tx_frame.Data[0],8);
		}
	}
}

/**
 * @brief  通过PDO Mapping请求数据
 * @param  节点ID，PDO索引，对象数，对象，数据缓存地址，触发类型
 * @retval 错误码
 * @author 陈佩奇
 */
//请求另一个client
uint8_t PDOManager_Classdef::Pull(uint8_t node_id,uint8_t PDO_Index,uint8_t n,uint32_t objects[],void* reg_buff[],PDO_TriggerType_Enum trigger_type)
{
	uint8_t temp_data[4] = {0x80+Local_NodeID,0x01,0x00,0x40};//???zhe
	uint8_t res =1;

	if(PDO_Index < 4)
	{
		RxPDO[PDO_Index].Map(n,objects);//把需要映射的对象字典数据拷贝
		RxPDO[PDO_Index].Regster(n,reg_buff);//把用来存储数据段的地址拷贝进去

//		PDO_WatchDog[PDO_Index].Start();

		/* 发送SDO帧 请求建立PDO通信 */
		/* 写入TxPDO参数 */
		/* Map映射对象数 */
		if(Transmit_SDO(node_id,0x1A00+PDO_Index,0x00,&n,SDO_Data_1Byte,5) == ERROR_TIMEOUT)
		{
			res |= ERROR_TIMEOUT;
		}

		/* COB-ID */
		if(Transmit_SDO(node_id,0x1800+PDO_Index,0x01,temp_data,SDO_Data_4Byte,5) == ERROR_TIMEOUT)
		{
			res |= ERROR_TIMEOUT;
		}
		for(size_t i = 0; i < n; i++)
		{
			memcpy(temp_data,&objects[i],4);
			/* 对象地址 */
			if(Transmit_SDO(node_id,0x1A00+PDO_Index,0x01+i,temp_data,SDO_Data_4Byte,5) == ERROR_TIMEOUT)
			{
				res |= ERROR_TIMEOUT;
			}
		}
		return res;
	}
	else
	{
		return ERROR_PDOMAPPED;
	}

}
//请求另一个client
uint8_t PDOManager_Classdef::Push(uint8_t node_id,uint8_t PDO_Index,uint8_t n,uint32_t objects[],PDO_TriggerType_Enum trigger_type)
{
	uint8_t temp_data[4] = {0x00+Local_NodeID,0x02,0x00,0x00};
	uint8_t res =1;

	if(PDO_Index < 4)
	{
		TxPDO[PDO_Index].Map(n,objects);
		TxPDO[PDO_Index].COB_ID = 0x180+node_id+(PDO_Index<<8);

		/* 发送SDO帧 请求建立PDO通信 */
		/* 写入RxPDO参数 */
		/* Map对象数 */
		if(Transmit_SDO(node_id,0x1600+PDO_Index,0x00,&n,SDO_Data_1Byte,5) == ERROR_TIMEOUT)
		{
			res |= ERROR_TIMEOUT;
		}
		/* COB-ID */
		if(Transmit_SDO(node_id,0x1400+PDO_Index,0x01,temp_data,SDO_Data_4Byte,5) == ERROR_TIMEOUT)
		{
			res |= ERROR_TIMEOUT;
		}
		for(size_t i = 0; i < n; i++)
		{
			memcpy(temp_data,&objects[i],4);
			/* 对象地址 */
			if(Transmit_SDO(node_id,0x1600+PDO_Index,0x01+i,temp_data,SDO_Data_4Byte,5) == ERROR_TIMEOUT)
			{
				res |= ERROR_TIMEOUT;
			}
		}
		return res;
	}
	else
	{
		return ERROR_PDOMAPPED;
	}

}

	/* PDO看门狗计时 */
//void PDOManager_Classdef::PDOWDog_Bark()
//{
//	for(size_t i = 0;i < 4; i++)
//	{
//		PDO_WatchDog[i].Bark();
//	}
//}
//	/* PDO状态 */
//uint8_t PDOManager_Classdef::PDO_Status(uint8_t PDO_Index)
//{
//	if(PDO_Index >= 4)
//	{
//		for(size_t i = 0;i < 4; i++)
//		{
//			if(PDO_WatchDog[i].Status() == ERROR_TIMEOUT)
//			{
//				return ERROR_TIMEOUT;
//			}
//		}
//		return COM_OK;
//	}
//	else
//	{
//		return PDO_WatchDog[PDO_Index].Status();
//	}
//}

uint8_t PDOManager_Classdef::Transmit_SDO(
		                                 uint8_t ndoe_id,
										  uint16_t index, uint8_t sub_index,
										  uint8_t * data,
										  SDO_DataLength_Enum data_length,
										  uint8_t max_retran_times
										  )
{
	uint8_t ctrl_byte;
	uint8_t wait_ms = 0;
	uint8_t retran_times = 0;

	if(data_length == SDO_Data_1Byte)
		ctrl_byte = 0x2f;
	else if(data_length == SDO_Data_2Byte)
		ctrl_byte = 0x2b;
	else if(data_length == SDO_Data_4Byte)
		ctrl_byte = 0x23;

	SDO_Frame_Typedef SDO_Tx_Frame = {
		DEVICE_ID_TX+ndoe_id, ctrl_byte, index, sub_index, data_length
	};

	/* SDO的数据段只有4个字节，前4个字节为控制数据段 */
	memset(&SDO_Tx_Frame.Data[4],0,4);
	memcpy(&SDO_Tx_Frame.Data[4],&data[0],(uint8_t)data_length);

	/* 设置SDO的控制段 */
	SDO_Tx_Frame.Data[0] = SDO_Tx_Frame.Ctrl_Byte;
	SDO_Tx_Frame.Data[1] = SDO_Tx_Frame.Index & 0xff;
	SDO_Tx_Frame.Data[2] = (SDO_Tx_Frame.Index >> 8) & 0xff;
	SDO_Tx_Frame.Data[3] = SDO_Tx_Frame.Sub_Index;

	vTaskDelay(3);

	/* 发送SDO帧 */
	do{
		CANx_SendData(hcan,SDO_Tx_Frame.COB_ID,SDO_Tx_Frame.Data,8);

		wait_ms = 0;
		/* 等待直到收到回传 */
		uint32_t NotifyValue=0;
		while(wait_ms++ <60)
		{
			vTaskDelay(1);
			NotifyValue=ulTaskNotifyTake(pdTRUE, 9);
			if(NotifyValue)
				{
			      NotifyValue=0;
				  return COM_OK;
				}
		}
		if(retran_times++ >= max_retran_times)
	  {
		    return ERROR_TIMEOUT;
	  }
	}while(true);
	
	
}

//void WatchDog_Classdef::Start()
//{
//	isStarted = 1;
//}


//void WatchDog_Classdef::Bark()
//{
//	if(isStarted)
//	{
//		if(Waitms < MaxWaitms)
//		{
//				Waitms++;
//		}
//	}
//	else
//	{
//		Waitms = 0;
//	}
//}

//void WatchDog_Classdef::Feed()
//{
//	Waitms = 0;
//}

//uint8_t WatchDog_Classdef::Status()
//{
//	if(Waitms >= MaxWaitms && isStarted)
//	{
//		return ERROR_TIMEOUT;
//	}
//	return COM_OK;
//}

/**
 * @brief  将Abort Code写入数据段
 * @param  Abort Code，数据段地址
 * @retval void
 * @author 苏锌雨
 */
void Set_Abort_Code(uint32_t abort_code, uint8_t* data)
{
	data[0] = abort_code&0xff;
	data[1] = (abort_code>>8)&0xff;
	data[2] = (abort_code>>16)&0xff;
	data[3] = (abort_code>>24)&0xff;
}

/**
 * @brief  通过SDO类型和数据长度设置Ctrl Byte
 * @param  工作类型，数据长度
 * @retval Ctrl Byte
 * @author 苏锌雨
 */
uint8_t Set_Ctrl_Byte(SDO_WorkType_Enum work_type,SDO_DataLength_Enum data_length)
{
	uint8_t ctrl_byte;
	switch(work_type)
	{
		case SDO_ReadSend:
			ctrl_byte = 0x40;
			break;
		case SDO_ReadRecv:
			if(data_length == SDO_Data_1Byte)
				ctrl_byte = 0x4f;
			else if(data_length == SDO_Data_2Byte)
				ctrl_byte = 0x4b;
			else if(data_length == SDO_Data_4Byte)
				ctrl_byte = 0x43;
			break;
		case SDO_WriteSend:
			if(data_length == SDO_Data_1Byte)
				ctrl_byte = 0x2f;
			else if(data_length == SDO_Data_2Byte)
				ctrl_byte = 0x2b;
			else if(data_length == SDO_Data_4Byte)
				ctrl_byte = 0x23;
			break;
		case SDO_WriteRecv:
			ctrl_byte = 0x60;
			break;
	}
	return ctrl_byte;
}

/**
 * @brief  对象字典注册
 * @param  对象信息，注册地址
 * @retval void
 * @author 苏锌雨
 */
void Object_Dict_Register(void* param_ptr, uint16_t index, uint8_t sub_index, bool RW, Object_Type_Enum type)
{
	/* 地址转换 */
	index = Addr_Transfer(index);

	if(index >= OD_MAX_INDEXS || sub_index > OD_MAX_SUBS)
	{
		return;
	}

	/* 注册 */
	Object_Dictionary[index][sub_index] = param_ptr;
	Type_Memery[index*OD_MAX_SUBS+sub_index] = type << 1;
	Type_Memery[index*OD_MAX_SUBS+sub_index] |= RW;
}

/**
 * @brief  查询对象R/W
 * @param  对象地址
 * @retval bool
 * @author 苏锌雨
 */
bool Object_RW(uint16_t index, uint8_t sub_index)
{
	/* 地址转换 */
	index = Addr_Transfer(index);

	if(index*OD_MAX_SUBS+sub_index < OD_MAX_INDEXS*OD_MAX_SUBS)
	{
		return Type_Memery[index*OD_MAX_SUBS+sub_index] & 0x01;
	}

	return ReadOnly;
}

/**
 * @brief  查询对象类型
 * @param  对象地址
 * @retval Object_Type_Enum 对象类型枚举
 * @author 苏锌雨
 */
Object_Type_Enum Object_Type(uint16_t index, uint8_t sub_index)
{
	/* 地址转换 */
	index = Addr_Transfer(index);

	if(index*OD_MAX_SUBS+sub_index < OD_MAX_INDEXS*OD_MAX_SUBS)
	{
		return Object_Type_Enum(Type_Memery[index*OD_MAX_SUBS+sub_index] >> 1);
	}

	return UINT8;
}

/**
 * @brief  写入对象
 * @param  对象地址，数据
 * @retval uint8_t(错误类型)
 * @author 苏锌雨
 */
uint8_t Object_Write(uint16_t index, uint8_t sub_index, uint8_t * Data)
{
	static uint8_t * data_uint8;
	static int8_t * data_int8;
	static uint16_t * data_uint16;
	static int16_t * data_int16;
	static uint32_t * data_uint32;
	static int32_t * data_int32;
	static float* data_float;

	void* addr = Addr_Read(index,sub_index);

	if(addr == NULL)
	{
		return ERROR_INDEXERROR;
	}


	switch(Object_Type(index,sub_index))
	{
		case UINT8:
			data_uint8 = (uint8_t*)addr;
			*data_uint8 = uint8_t(Data[4]);
			break;
		case INT8:
			data_int8 = (int8_t*)addr;
			*data_int8 = int8_t(Data[4]);
			break;
		case UINT16:
			data_uint16 = (uint16_t*)addr;
			*data_uint16 = uint16_t(Data[4]|(Data[5]<<8));
			break;
		case INT16:
			data_int16 = (int16_t*)addr;
			*data_int16 = int16_t(Data[4]|(Data[5]<<8));
			break;
		case UINT32:
			data_uint32 = (uint32_t*)addr;
			*data_uint32 = uint32_t(Data[4]|(Data[5]<<8)|(Data[6]<<16)|(Data[7]<<24));
			break;
		case INT32:
			data_int32 = (int32_t*)addr;
			*data_int32 = int32_t(Data[4]|(Data[5]<<8)|(Data[6]<<16)|(Data[7]<<24));
			break;
		case FLOAT:
			data_float = (float*)addr;
			memcpy(&data_float,&Data[4],4);
			break;
	}

	return COM_OK;
}

/**
 * @brief  读出对象
 * @param  对象地址，存放数据的内存
 * @retval uint8_t(错误类型)
 * @author 苏锌雨
 */
//读出对象的内容
uint8_t Object_Read(uint16_t index, uint8_t sub_index, uint8_t * Data)
{
	static uint8_t * data_uint8;
	static int8_t * data_int8;
	static uint16_t * data_uint16;
	static int16_t * data_int16;
	static uint32_t * data_uint32;
	static int32_t * data_int32;
	static float* data_float;

	void* addr = Addr_Read(index,sub_index);

	if(addr == NULL)
	{
		return ERROR_INDEXERROR;
	}

	switch(Object_Type(index,sub_index))
	{
		case UINT8:
			data_uint8 = (uint8_t*)addr;//取对象地址
			memcpy(&Data[0],data_uint8,1);
			break;
		case INT8:
			data_int8 = (int8_t*)addr;
			memcpy(&Data[0],data_int8,1);
			break;
		case UINT16:
			data_uint16 = (uint16_t*)addr;
			memcpy(&Data[0],data_uint16,2);
			break;
		case INT16:
			data_int16 = (int16_t*)addr;
			memcpy(&Data[0],data_int16,2);
			break;
		case UINT32:
			data_uint32 = (uint32_t*)addr;
			memcpy(&Data[0],data_uint32,4);
			break;
		case INT32:
			data_int32 = (int32_t*)addr;
			memcpy(&Data[0],data_int32,4);
			break;
		case FLOAT:
			data_float = (float*)addr;
			memcpy(&Data[0],data_float,4);
			break;
	}

	return COM_OK;
}

void* Addr_Read(uint16_t index, uint8_t sub_index)
{
	index = Addr_Transfer(index);
	//下面是检查对象是否有效，有效则返回地址
	if(index >= OD_MAX_INDEXS || sub_index >= OD_MAX_SUBS)
	{
		return NULL;
	}

	if(Object_Dictionary[index][sub_index] == NULL)
	{
		return NULL;
	}

	return Object_Dictionary[index][sub_index];
}

uint16_t Addr_Transfer(uint16_t addr)
{
	/* 实验室指定参数 */
	if(addr >= 0x2000)
	{
		addr -= 0x2000;
	}

	/* 通信参数 */
	//PDO的通信参数
	switch (addr & 0xff00)
	{
	    case 0x1000:
	    	addr= addr - 0x1015 + 20 -4;
	    	break;
		case 0x1400:
			addr = addr - 0x1400 + 20;
			break;
		case 0x1800:
			addr = addr - 0x1800 + 20 + 4;
			break;
		case 0x1600:
			addr = addr - 0x1600 + 20 + 8;
			break;
		case 0x1A00:
			addr = addr - 0x1A00 + 20 + 12;
			break;
	}

	return addr;
}

/*NMT----------------------------------------------------------*/

/**
 * @brief  心跳报文时间的注册
 * @note    生产者的消费时间不能低于20ms,消费者的时间不能低于40ms，
            且必须要大于生产者的消费时间的1.8倍
 * @author 陈佩奇
 */

/*心跳报文时间的注册*/
/*生产者的消费时间不能低于20ms,消费者的时间不能低于40ms，且必须要大于生产者的消费时间的1.8倍*/
NMT_Classdef::NMT_Classdef(CAN_HandleTypeDef* hcanx,uint16_t pro_time,uint16_t con_time):hcan(hcanx),\
		                                   producer_time(pro_time),\
										   start_byte(0),\
										   reset_byte(0),\
										   pre_byte(0)

{

		Object_Dict_Register(&this->producer_time,0x1017,0x00,ReadNWrite,UINT16);
		Object_Dict_Register(&this->Consumer[0].consumer_time,0x1016,0x00,ReadNWrite,UINT32);
		Object_Dict_Register(&this->Consumer[1].consumer_time,0x1016,0x01,ReadNWrite,UINT32);
		Object_Dict_Register(&this->Consumer[2].consumer_time,0x1016,0x02,ReadNWrite,UINT32);
		Object_Dict_Register(&this->Consumer[3].consumer_time,0x1016,0x03,ReadNWrite,UINT32);
		Object_Dict_Register(&this->Consumer[4].consumer_time,0x1016,0x04,ReadNWrite,UINT32);
	  addr1=Addr_Transfer(0x1600);
	  addr2=Addr_Transfer(0x1400);
    addr3=Addr_Transfer(0x1A00);
	  addr4=Addr_Transfer(0x1800);
	  for(uint8_t i=0;i<5;i++)
	 {
	  this->Consumer[i]=con_time;
	 }

}

/**
 * @brief  NMT心跳报文发送函数
 * @note   需定时发送
 * @param  节点序号
 * @retval  void
 * @author  陈佩奇
 */

void NMT_Classdef::NMT_HeartBeat_Transmit(uint8_t node_id)
{
	NMT_Frame_Typedef NMT_Tx_Frame;
	NMT_Tx_Frame.COB_ID=0x700+node_id;
	NMT_Tx_Frame.Data[0]=WorkingState;
	CANx_SendData(hcan, NMT_Tx_Frame.COB_ID, NMT_Tx_Frame.Data, 1);

}
/**
 * @brief  NMT发送指令函数
 * @param  控制码
 * @retval void
 * @author 陈佩奇
 */

void NMT_Classdef::NMT_Contrl_Transmit(uint8_t controldata[])
{
	NMT_Frame_Typedef NMTcontrol={0x00,0};
	memcpy(NMTcontrol.Data,controldata,2);
	CANx_SendData(hcan, NMTcontrol.COB_ID,NMTcontrol.Data,2);

}

/**
 * @brief  NMT中断处理函数
 * @note   在中断中调用
 * @param  can接收到的信息
 * @retval void
 * @author 陈佩奇
 */

void NMT_Classdef::NMT_Callback(CAN_RxBuffer* recv_msg)
{
	  NMT_Frame_Typedef NMT_Rx_Frame = {recv_msg->header.StdId,recv_msg->header.RTR};
		memcpy(&NMT_Rx_Frame.Data[0],recv_msg->data,2);

		if((NMT_Rx_Frame.COB_ID&NMT_HEARTBEAT_ID)==NMT_HEARTBEAT_ID)
		{
            uint8_t id=NMT_Rx_Frame.COB_ID&0x00f;
            Consumer[id].rx_num++;
		}
		if(((NMT_Rx_Frame.COB_ID&NMT_CONTROL_ID)==NMT_CONTROL_ID)&&(NMT_Rx_Frame.Data[1]==Local_NodeID))
		{
			switch (NMT_Rx_Frame.Data[0])
			{
			case Start:
				start_byte++;
				break;
			case ResetCommunication:
				reset_byte++;
				break;
			case PreOperation:
				pre_byte++;
				break;
			}
		}
}

/**
 * @brief  监控生产者(发送心跳报文的板)的工作状态
 * @note   需定时检查
 * @param  生产者的节点序号
 * @retval void
 * @author 陈佩奇
 */
/*检查是否接收到心跳心跳报文*/
uint8_t NMT_Classdef::NMT_HeartBeat_Monitor(uint8_t node_id)
{
	
	if(Consumer[node_id].rx_num==0)
	{
		for(uint8_t q=0;q<2;q++)
		{
		 uint8_t rsdata[2]={ResetCommunication,node_id};
		 NMT_Contrl_Transmit(rsdata);
		 HAL_Delay(30);
	  }
		Consumer[node_id].rx_num=0;
	  return 0;
	}
	Consumer[node_id].rx_num=0;
  return 1;
}

//  uint32_t MAXTIME=Consumer[node_id].consumer_time;
//	uint32_t wait_ms=0;
//	while(wait_ms<MAXTIME)
//	{
//		vTaskDelay(1);
//		wait_ms ++;
//		if(Consumer[node_id].rx_num)
//			break;
//	}
//	if(wait_ms>=MAXTIME)
//	{
//		//因为NMT无返回，所以发送多几次确保复位通信，延时测试过再改
//		for(uint8_t q=0;q<3;q++)
//		{
//		 uint8_t rsdata[2]={ResetCommunication,node_id};
//		 NMT_Contrl_Transmit(rsdata);
//		 HAL_Delay(50);
//	  }
//	}


/**
 * @brief  保留通信参数在Flash和读取通信参数
 * @note   num的范围为0-7
 *         其中 0-3为TDPO所用参数
 *         4-7为RPOO所用参数
 *         每个TPDO和RPDO所对应的地址是不一样的
 * @param  TPDO&RPDO的序号
 * @retval 是否成功,1为成功，0为错误
 * @author 陈佩奇
 */

uint8_t NMT_Classdef::Copy_PDO_Message(uint8_t num)
{
	    uint8_t write_state=1;
	    if(num<=7)
	    {
	       if(num>=4&&num<=7)
	       {
	          write_state&=Write_Message(ADDR_FLASH_SECTOR_6+num*22, (uint16_t*)Object_Dictionary[addr1+num][0x00], 2);//因为stm32flash只支持字或半字的写入，所以Mapped_Nums要用两位地址存储
	          write_state&=Write_Message(ADDR_FLASH_SECTOR_6+2+num*22, (uint16_t*)Object_Dictionary[addr2+num][0x01], 4);
					  write_state&=Write_Message(ADDR_FLASH_SECTOR_6+2+4+num*22,(uint16_t*)Object_Dictionary[addr1+num][0x01],16);
	          
	       }
	       else if(num<=3)
	       {
	    	  write_state&=Write_Message(ADDR_FLASH_SECTOR_6+num*22, (uint16_t*)Object_Dictionary[addr3+num][0x00], 2);//因为stm32flash只支持字或半字的写入，所以Mapped_Nums要用两位地址存储
	    	  write_state&=Write_Message(ADDR_FLASH_SECTOR_6+2+num*22, (uint16_t*)Object_Dictionary[addr4+num][0x01], 4);
	  	     write_state&=Write_Message(ADDR_FLASH_SECTOR_6+2+4+num*22,(uint16_t*)Object_Dictionary[addr3+num][0x01],16);
	       }
	       return write_state;
	    }
	    return 0;

}


uint8_t NMT_Classdef::Reset_PDO_Message(uint8_t num)
{
	if(num<=7)
	{
		if(num>=4&&num<=7)
		{
			
			memcpy(Object_Dictionary[addr1+num][0x00],(uint16_t*)(ADDR_FLASH_SECTOR_6+num*22),1);
		  memcpy(Object_Dictionary[addr2+num][0x01],(uint16_t*)(ADDR_FLASH_SECTOR_6+2+num*22),4);
		  memcpy(Object_Dictionary[addr1+num][0x01],(uint16_t*)(ADDR_FLASH_SECTOR_6+2+4+num*22),16);
		    return 1;
		}
		else if(num<=3)
	    {
				
			memcpy(Object_Dictionary[addr3+num][0x00],(uint32_t*)(ADDR_FLASH_SECTOR_6+num*22),1);
			memcpy(Object_Dictionary[addr4+num][0x01],(uint32_t*)(ADDR_FLASH_SECTOR_6+2+num*22),4);
			memcpy(Object_Dictionary[addr3+num][0x01],(uint32_t*)(ADDR_FLASH_SECTOR_6+2+4+num*22),16);
			return 1;
	    }
	}
     return 0;

}

/**
 * @brief  NMT控制处理函数
 * @param  void
 * @retval void
 * @author 陈佩奇
 */

void NMT_Classdef::NMT_Exce_Once(TickType_t sys_time,uint8_t heart_node_id,uint8_t monitor_node_id,uint8_t flag)
{
	static TickType_t HeartBeat_time=0, NMT_Monitor_time=0;
	if(start_byte&&flag)
	{
		for(uint8_t i=0;i<8;i++)
		{
		 Copy_PDO_Message(i);
		}
		start_byte=0;
	}
	if(reset_byte&&flag)
	{
		for(uint8_t i=0;i<8;i++)
		{
		 Reset_PDO_Message(i);
		}
		reset_byte=0;
	}
	if(pre_byte)
	{
		Erase(FLASH_SECTOR_6);
		pre_byte=0;
	}
	if(sys_time - HeartBeat_time >= producer_time)
	{
	  NMT_HeartBeat_Transmit(heart_node_id);
		HeartBeat_time=sys_time;
  }
		
	if((int64_t)sys_time -(int64_t)NMT_Monitor_time >Consumer[0].consumer_time)
 {
	  if(NMT_HeartBeat_Monitor(monitor_node_id))
		{
			NMT_Monitor_time=sys_time;
		}
		else 
		{
			NMT_Monitor_time=sys_time+1000;
		}
	}

}

	


/*保留的NMT邮箱函数以及其他控制报文的接口-----------------------------------------------*/

///*NMT邮箱处理*/
//uint8_t NMT_Mailbox_Classdef::size()
//{
//	return sizes;
//}
//void NMT_Mailbox_Classdef::push_back(NMT_Frame_Typedef input_frame)
//{
//	if(sizes < MAX_MAILBOX_DEEPTH)
//	{
//		NMT_Mailbox[sizes] = input_frame;
//		sizes++;
//	}
//}
//void NMT_Mailbox_Classdef:: erase()
//{
//	static NMT_Frame_Typedef clear_frame = {0,0,0};
//
//		if(sizes)
//		{
//			for(uint8_t i = 0; i < sizes - 1; i++)
//			{
//				NMT_Mailbox[i] = NMT_Mailbox[i+1];
//			}
//			NMT_Mailbox[sizes - 1] = clear_frame;
//			sizes--;
//		}
//}
//
//NMT_Frame_Typedef NMT_Mailbox_Classdef::operator [](uint8_t num)
//{
//	return NMT_Mailbox[num];
//}
//
//void NMT_Classdef::NMT_Exce_Once()
//{
//	 if(NMT_Rx_Mailbox.size())
//	 {
//		 switch(NMT_Rx_Mailbox[0].Data[0])
//		 	{
//		 	case Start:
//		 		if(nowstate==PreOperationState||nowstate==StopState)
//		 			//disuse
//		 		break;
//		 	case Stop:
//		 		if(nowstate==PreOperationState||nowstate==WorkingState)
//		 			//disuse
//		 		break;
//		 	case PreOperation:
//		 		if(nowstate==StopState||nowstate==WorkingState)
//		 			//disuse
//		 		break;
//		 	case ResetNode:
//		 		//disuse
//		 		break;
//		 	case ResetCommunication:
//
//		 		break;
//		 	}
//	 }
//}






