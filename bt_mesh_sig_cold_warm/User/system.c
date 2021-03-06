/****************************************Copyright (c)*************************
**                               版权所有 (C), 2015-2017, 涂鸦科技
**
**                                 http://www.tuya.com
**
**--------------文件信息-------------------------------------------------------
**文   件   名: system.c
**描        述: bluetooth数据处理函数
**使 用 说 明 : 用户无需关心该文件实现内容
**
**
**--------------当前版本修订---------------------------------------------------
** 版  本: v1.0
** 日　期: 2017年5月3日
** 描　述: 1:创建涂鸦bluetooth对接MCU_SDK
**
**-----------------------------------------------------------------------------
******************************************************************************/
#define SYSTEM_GLOBAL
#include "HC89S003F4.h"
#include "bluetooth.h"
//
extern ulong xdata SUM0;	   //
extern ulong xdata SUM2;		//
extern uint xdata average;	//
extern u8 xdata light_ad;		//
extern ulong xdata TH;		//
extern unsigned char upload_disable;

extern u16 idata groupaddr[8];
extern u8 idata Exit_network_controlflag;

void reset_bt_module(void);
void savevar(void);
extern const DOWNLOAD_CMD_S xdata download_cmd[];
////////////////////////////////////////////////////////
// pc send: 55 AA 00 C0 00 00 BF
// mcu send:
// HEADER  version   com0    length  avg  light  SUM0    SUM2    TH     CRC
// 55 AA   00        C0      00 08   4E   FF     03 ED   07 69   00 9C  10
// 55 AA   00        C0      00 08   4E   FF     05 B9   07 75   00 9C  EA
// 55 AA   00        C0      00 08   4E   FF     06 9F   07 8B   00 9C  E7
// 55 AA   00        C0      00 08   4E   FF     07 68   07 BA   00 9C  E0
// 55 AA   00        C0      00 08   4E   FF     07 85   07 88   00 9C  CB
////////////////////////////////////////////////////////
static void cmd0(void)
{
    unsigned char length = 0;

		length = set_bt_uart_byte(length, average >> 4);
		length = set_bt_uart_byte(length, light_ad);
		length = set_bt_uart_byte(length, SUM0 >> 16);
		length = set_bt_uart_byte(length, SUM0 >> 8);
		length = set_bt_uart_byte(length, SUM2 >> 16);
		length = set_bt_uart_byte(length, SUM2 >> 8);
		length = set_bt_uart_byte(length, TH >> 16);
		length = set_bt_uart_byte(length, TH >> 8);
		
    bt_uart_write_frame(USER_DEFINE_CMD0, length);
}

static void cmd1(void)
{
    unsigned char length = 0;

    //length = set_zigbee_uart_buffer(length,(unsigned char *)"{\"p\":\"",my_strlen((unsigned char *)"{\"p\":\""));

    bt_uart_write_frame(USER_DEFINE_CMD1, length);
}
/*****************************************************************************
函数名称 : set_bt_uart_byte
功能描述 : 写bt_uart字节
输入参数 : dest:缓存区其实地址;
           byte:写入字节值
返回参数 : 写入完成后的总长度
*****************************************************************************/
unsigned short set_bt_uart_byte(unsigned short dest, unsigned char byte)
{
  unsigned char *obj = (unsigned char *)bt_uart_tx_buf + DATA_START + dest;
  
  *obj = byte;
  dest += 1;
  
  return dest;
}
/*****************************************************************************
函数名称 : set_bt_uart_buffer
功能描述 : 写bt_uart_buffer
输入参数 : dest:目标地址
           src:源地址
           len:数据长度
返回参数 : 无
*****************************************************************************/
unsigned short set_bt_uart_buffer(unsigned short dest, unsigned char *src, unsigned short len)
{
  unsigned char *obj = (unsigned char *)bt_uart_tx_buf + DATA_START + dest;
  
  my_memcpy(obj,src,len);
  
  dest += len;
  return dest;
}
/*****************************************************************************
函数名称 : bt_uart_write_data
功能描述 : 向bt uart写入连续数据
输入参数 : in:发送缓存指针
           len:数据发送长度
返回参数 : 无
*****************************************************************************/
static void bt_uart_write_data(unsigned char *in, unsigned short len)
{
  if((NULL == in) || (0 == len))
  {
    return;
  }
  
  while(len --)
  {
    uart_transmit_output(*in);
    in ++;
  }
}
/*****************************************************************************
函数名称 : get_check_sum
功能描述 : 计算校验和
输入参数 : pack:数据源指针
           pack_len:计算校验和长度
返回参数 : 校验和
*****************************************************************************/
unsigned char get_check_sum(unsigned char *pack, unsigned short pack_len)
{
  unsigned short i;
  unsigned char check_sum = 0;
  
  for(i = 0; i < pack_len; i ++)
  {
    check_sum += *pack ++;
  }
  
  return check_sum;
}
/*****************************************************************************
函数名称 : bt_uart_write_frame
功能描述 : 向bt串口发送一帧数据
输入参数 : fr_type:帧类型
           len:数据长度
返回参数 : 无
*****************************************************************************/
void bt_uart_write_frame(unsigned char fr_type, unsigned short len)
{
  unsigned char check_sum = 0;
  
  bt_uart_tx_buf[HEAD_FIRST] = 0x55;
  bt_uart_tx_buf[HEAD_SECOND] = 0xaa;
  bt_uart_tx_buf[PROTOCOL_VERSION] = 0x00;
  bt_uart_tx_buf[FRAME_TYPE] = fr_type;
  bt_uart_tx_buf[LENGTH_HIGH] = len >> 8;
  bt_uart_tx_buf[LENGTH_LOW] = len & 0xff;
  
  len += PROTOCOL_HEAD;
  check_sum = get_check_sum((unsigned char *)bt_uart_tx_buf, len - 1);
  bt_uart_tx_buf[len - 1] = check_sum;
  //
  bt_uart_write_data((unsigned char *)bt_uart_tx_buf, len);
}

/*****************************************************************************
函数名称 : heat_beat_check
功能描述 : 心跳包检测
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void heat_beat_check(void)
{
  unsigned char length = 0;
  static unsigned char mcu_reset_state = FALSE;
  
  if(FALSE == mcu_reset_state)
  {
    length = set_bt_uart_byte(length,FALSE);
    mcu_reset_state = TRUE;
  }
  else
  {
    length = set_bt_uart_byte(length,TRUE);
  }
  
  bt_uart_write_frame(HEAT_BEAT_CMD, length);
}
/*****************************************************************************
函数名称  : product_info_update
功能描述  : 产品信息上传
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void product_info_update(void)
{
  unsigned char length = 0;
  
  length = set_bt_uart_buffer(length,(unsigned char *)PRODUCT_KEY,my_strlen((unsigned char *)PRODUCT_KEY));
  length = set_bt_uart_buffer(length,(unsigned char *)MCU_VER,my_strlen((unsigned char *)MCU_VER));
  
  bt_uart_write_frame(PRODUCT_INFO_CMD, length);
}
/*****************************************************************************
函数名称 : get_mcu_bt_mode
功能描述 : 查询mcu和bt的工作模式
输入参数 : 无
返回参数 : 无
*****************************************************************************/
static void get_mcu_bt_mode(void)
{
  unsigned char length = 0;
  
#ifdef BT_CONTROL_SELF_MODE                                   //模块自处理
  length = set_bt_uart_byte(length, BT_STATE_KEY);
  length = set_bt_uart_byte(length, BT_RESERT_KEY);
#else                                                           
  //无需处理数据
#endif
  
  bt_uart_write_frame(WORK_MODE_CMD, length);
}
/*****************************************************************************
函数名称 : get_update_dpid_index
功能描述 : 或许制定DPID在数组中的序号
输入参数 : dpid:dpid
返回参数 : index:dp序号
*****************************************************************************/
static unsigned char get_dowmload_dpid_index(unsigned char dpid)
{
  unsigned char index;
  unsigned char total = get_download_cmd_total();
  
  for(index = 0; index < total; index ++)
  {
    if(download_cmd[index].dp_id == dpid)
    {
      break;
    }
  }
  
  return index;
}
/*****************************************************************************
函数名称 : data_point_handle
功能描述 : 下发数据处理
输入参数 : value:下发数据源指针
返回参数 : ret:返回数据处理结果
*****************************************************************************/
static unsigned char data_point_handle(const unsigned char value[])
{
  unsigned char dp_id,index;
  unsigned char dp_type;
  unsigned char ret;
  unsigned short dp_len;
  
  dp_id = value[0];
  dp_type = value[1];
  dp_len = value[2] * 0x100;
  dp_len += value[3];
  
  index = get_dowmload_dpid_index(dp_id);

  if(dp_type != download_cmd[index].dp_type)
  {
    //错误提示
    return FALSE;
  }
  else
  {
    ret = dp_download_handle(dp_id,value + 4,dp_len);
  }
  
  return ret;
}
/*****************************************************************************
函数名称 : data_handle
功能描述 : 数据帧处理
输入参数 : offset:数据起始位
返回参数 : 无
*****************************************************************************/
void data_handle(unsigned short offset)
{
#ifdef SUPPORT_MCU_FIRM_UPDATE
  unsigned char *firmware_addr;
  static unsigned long firm_length;                                             //MCU升级文件长度
  static unsigned char firm_update_flag;                                        //MCU升级标志
  unsigned long dp_len;
#else
  unsigned short dp_len;
#endif
  
  unsigned char ret;
  unsigned short i,total_len;
  unsigned char cmd_type = bt_uart_rx_buf[offset + FRAME_TYPE];
	
	unsigned char rsp_status;
	
	unsigned char adr_num;
  
  switch(cmd_type)
  {
		case BT_MESH_ENABLE:
			rsp_status = bt_uart_rx_buf[offset + DATA_START];
			if (0 == rsp_status)
			{
				bt_uart_write_frame(BT_MESH_GET_MY_GROUP_ADDRESS, 0);
			}
			else
			{
				//
			}
			break;
		case USER_DEFINE_CMD0:
			cmd0();
			break;
		case USER_DEFINE_CMD1:
			cmd1();
			break;
  
  case HEAT_BEAT_CMD:                                   //心跳包
    heat_beat_check();
    break;
    
  case PRODUCT_INFO_CMD:                                //产品信息
    product_info_update();
    break;
    
  case WORK_MODE_CMD:                                   //查询MCU设定的模块工作模式
    get_mcu_bt_mode();
    break;
    
#ifndef BT_CONTROL_SELF_MODE
  case BT_STATE_CMD:                                  //bt工作状态	
    bt_work_state = bt_uart_rx_buf[offset + DATA_START];
	if (BT_CONNECTED == bt_work_state)//绑定并连接
	{
		savevar();
		all_data_update();
		upload_disable = 0;
  		Exit_network_controlflag = 0;
	}
	else if (BT_NOT_CONNECTED == bt_work_state)//绑定未连接
	{
		upload_disable = 1;
		Exit_network_controlflag = 0;
	}
	else if (BT_UN_BIND == bt_work_state)//蓝牙未绑定
	{
		upload_disable = 1;
		Exit_network_controlflag = 1;
	}
	else		//未知状态
	{
		upload_disable = 1;
		Exit_network_controlflag = 1;		
	}
    bt_uart_write_frame(BT_STATE_CMD,0);
    break;

  case BT_RESET_CMD:                                  //重置bt(bt返回成功)
    reset_bt_flag = RESET_BT_SUCCESS;
    break;
#endif
    
  case DATA_QUERT_CMD:                                  //命令下发
    total_len = bt_uart_rx_buf[offset + LENGTH_HIGH] * 0x100;
    total_len += bt_uart_rx_buf[offset + LENGTH_LOW];
    
    for(i = 0;i < total_len;)
    {
      dp_len = bt_uart_rx_buf[offset + DATA_START + i + 2] * 0x100;
      dp_len += bt_uart_rx_buf[offset + DATA_START + i + 3];
      //
      ret = data_point_handle((unsigned char *)bt_uart_rx_buf + offset + DATA_START + i);
      
      if(SUCCESS == ret)
      {
        //成功提示
		savevar();
      }
      else
      {
        //错误提示
		//不保存参数
      }
      
      i += (dp_len + 4);
    }
    
    break;
  case BT_MESH_GET_MY_GROUP_ADDRESS:                                  //查询群组
    total_len = bt_uart_rx_buf[offset + LENGTH_HIGH] * 0x100;
    total_len += bt_uart_rx_buf[offset + LENGTH_LOW];

	adr_num = bt_uart_rx_buf[offset + DATA_START];
	
	groupaddr[0] = bt_uart_rx_buf[offset + DATA_START + 1] * 0x100;
	groupaddr[0] += bt_uart_rx_buf[offset + DATA_START + 2] ;

	groupaddr[1] = bt_uart_rx_buf[offset + DATA_START + 3] * 0x100;
	groupaddr[1] += bt_uart_rx_buf[offset + DATA_START + 4] ;

	groupaddr[2] = bt_uart_rx_buf[offset + DATA_START + 5] * 0x100;
	groupaddr[2] += bt_uart_rx_buf[offset + DATA_START + 6] ;

	groupaddr[3] = bt_uart_rx_buf[offset + DATA_START + 7] * 0x100;
	groupaddr[3] += bt_uart_rx_buf[offset + DATA_START + 8] ;

	groupaddr[4] = bt_uart_rx_buf[offset + DATA_START + 9] * 0x100;
	groupaddr[4] += bt_uart_rx_buf[offset + DATA_START + 10] ;

	groupaddr[5] = bt_uart_rx_buf[offset + DATA_START + 11] * 0x100;
	groupaddr[5] += bt_uart_rx_buf[offset + DATA_START + 12] ;

	groupaddr[6] = bt_uart_rx_buf[offset + DATA_START + 13] * 0x100;
	groupaddr[6] += bt_uart_rx_buf[offset + DATA_START + 14] ;

	groupaddr[7] = bt_uart_rx_buf[offset + DATA_START + 15] * 0x100;
	groupaddr[7] += bt_uart_rx_buf[offset + DATA_START + 16] ;

	if (groupaddr[0])
	{
		mcu_dp_enum_update(DPID_ADDR0, 1); //VALUE型数据上报;		
	}
	else
	{
		mcu_dp_enum_update(DPID_ADDR0, 0); //VALUE型数据上报;		
	}

	if (groupaddr[1])
	{
		mcu_dp_enum_update(DPID_ADDR1, 1); //VALUE型数据上报;		
	}
	else
	{
		mcu_dp_enum_update(DPID_ADDR1, 0); //VALUE型数据上报;		
	}	
	
	if (groupaddr[2])
	{
		mcu_dp_enum_update(DPID_ADDR2, 1); //VALUE型数据上报;		
	}
	else
	{
		mcu_dp_enum_update(DPID_ADDR2, 0); //VALUE型数据上报;		
	}

	if (groupaddr[3])
	{
		mcu_dp_enum_update(DPID_ADDR3, 1); //VALUE型数据上报;		
	}
	else
	{
		mcu_dp_enum_update(DPID_ADDR3, 0); //VALUE型数据上报;		
	}		
	
	if (groupaddr[4])
	{
		mcu_dp_enum_update(DPID_ADDR4, 1); //VALUE型数据上报;		
	}
	else
	{
		mcu_dp_enum_update(DPID_ADDR4, 0); //VALUE型数据上报;		
	}

	if (groupaddr[5])
	{
		mcu_dp_enum_update(DPID_ADDR5, 1); //VALUE型数据上报;		
	}
	else
	{
		mcu_dp_enum_update(DPID_ADDR5, 0); //VALUE型数据上报;		
	}
	
	if (groupaddr[6])
	{
		mcu_dp_enum_update(DPID_ADDR6, 1); //VALUE型数据上报;		
	}
	else
	{
		mcu_dp_enum_update(DPID_ADDR6, 0); //VALUE型数据上报;		
	}

	if (groupaddr[7])
	{
		mcu_dp_enum_update(DPID_ADDR7, 1); //VALUE型数据上报;		
	}
	else
	{
		mcu_dp_enum_update(DPID_ADDR7, 0); //VALUE型数据上报;		
	}		

    break;
  case STATE_QUERY_CMD:                                 //状态查询
    all_data_update();                               
    break;
    
#ifdef SUPPORT_MCU_FIRM_UPDATE
  case UPDATE_START_CMD:                                //升级开始
    firm_length = bt_uart_rx_buf[offset + DATA_START];
    firm_length <<= 8;
    firm_length |= bt_uart_rx_buf[offset + DATA_START + 1];
    firm_length <<= 8;
    firm_length |= bt_uart_rx_buf[offset + DATA_START + 2];
    firm_length <<= 8;
    firm_length |= bt_uart_rx_buf[offset + DATA_START + 3];
    //
    bt_uart_write_frame(UPDATE_START_CMD,0);
    firm_update_flag = UPDATE_START_CMD;
     break;
    
  case UPDATE_TRANS_CMD:                                //升级传输
    if(firm_update_flag == UPDATE_START_CMD)
    {
      //停止一切数据上报
      stop_update_flag = ENABLE;                                                 
      
      total_len = bt_uart_rx_buf[offset + LENGTH_HIGH] * 0x100;
      total_len += bt_uart_rx_buf[offset + LENGTH_LOW];
      
      dp_len = bt_uart_rx_buf[offset + DATA_START];
      dp_len <<= 8;
      dp_len |= bt_uart_rx_buf[offset + DATA_START + 1];
      dp_len <<= 8;
      dp_len |= bt_uart_rx_buf[offset + DATA_START + 2];
      dp_len <<= 8;
      dp_len |= bt_uart_rx_buf[offset + DATA_START + 3];
      
      firmware_addr = bt_uart_rx_buf + offset + DATA_START + 4;
      if((total_len == 4) && (dp_len == firm_length))
      {
        //最后一包
        ret = mcu_firm_update_handle(firmware_addr,dp_len,0);
        
        firm_update_flag = 0;
      }
      else if((total_len - 4) <= FIRM_UPDATA_SIZE)
      {
        ret = mcu_firm_update_handle(firmware_addr,dp_len,total_len - 4);
      }
      else
      {
        firm_update_flag = 0;
        ret = ERROR;
      }
      
      if(ret == SUCCESS)
      {
        bt_uart_write_frame(UPDATE_TRANS_CMD,0);
      }
      //恢复一切数据上报
      stop_update_flag = DISABLE;    
    }
    break;
#endif      

  default:
    break;
  }
}
/*****************************************************************************
函数名称 : get_queue_total_data
功能描述 : 读取队列内数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
unsigned char get_queue_total_data(void)
{
  if(queue_in != queue_out)
    return 1;
  else
    return 0;
}
/*****************************************************************************
函数名称 : Queue_Read_Byte
功能描述 : 读取队列1字节数据
输入参数 : 无
返回参数 : 无
*****************************************************************************/
unsigned char Queue_Read_Byte(void)
{
  unsigned char value;
  
  if(queue_out != queue_in)
  {
    //有数据
    if(queue_out >= (unsigned char *)(bt_queue_buf + sizeof(bt_queue_buf)))
    {
      //数据已经到末尾
      queue_out = (unsigned char *)(bt_queue_buf);
    }
    
    value = *queue_out ++;   
  }
  
  return value;
}

