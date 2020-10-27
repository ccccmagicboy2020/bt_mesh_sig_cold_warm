/****************************************Copyright (c)*************************
**                               版权所有 (C), 2015-2017, 涂鸦科技
**
**                                 http://www.tuya.com
**
**--------------文件信息-------------------------------------------------------
**文   件   名: protocol.c
**描        述: 下发/上报数据处理函数
**使 用 说 明 :

                  *******非常重要，一定要看哦！！！********

** 1、用户在此文件中实现数据下发/上报功能
** 2、DP的ID/TYPE及数据处理函数都需要用户按照实际定义实现
** 3、当开始某些宏定义后需要用户实现代码的函数内部有#err提示,完成函数后请删除该#err
**
**--------------当前版本修订---------------------------------------------------
** 版  本: v1.0
** 日　期: 2017年5月3日
** 描　述: 1:创建涂鸦bluetooth对接MCU_SDK
**
**-----------------------------------------------------------------------------
******************************************************************************/
//#include "include.h"

#include "bluetooth.h"
#include "string.h"
#include <stdio.h>
  

extern u8 xdata switchcnt;      //复位模块点击次数计数
extern u8 xdata reset_bt_bn;    //复位模块的全局变量
extern u8 xdata SWITCHflag2;   //开关灯的变量
extern u8 xdata SWITCHfXBR;    //开关雷达的变量
extern u8 xdata lightvalue;    //灯亮值
extern u8 xdata XRBoffbrightvalue;  //关雷达后的灯亮值
extern ulong xdata TH;          //雷达感应偏差阈值，数值越大代表越不灵敏
extern u8 xdata LIGHT_TH;       //感光阈值
extern u16 xdata DELAY_NUM;     //感应延时，单位为秒
extern u8 xdata lowlightDELAY_NUM;      //关灯延时，单位为分钟
extern u8 xdata light_ad;               //采到的光感的瞬时值
u8 xdata cdsvalue = 0;              //感光选择值
ulong xdata sensing_th = 0;     //雷达感应阈值，数值越大越灵敏
extern  u8 idata Linkage_flag;	//联动的开关的全局
extern  u8 idata Light_on_flag;	//

//const char xdata led_bn_on[]={"led on"};
//const char xdata led_bn_off[]={"led off"};
//const char xdata radar_bn_on[]={"radar on"};
//const char xdata radar_bn_off[]={"radar off"};

unsigned char DPID_SWITCH_LED2count = 0;
unsigned char DPID_SWITCH_XBRcount = 0;
unsigned char DPID_BRIGHT_VALUEcount = 0;
unsigned char DPID_CDScount = 0;
unsigned char DPID_PIR_DELAYcount = 0;
unsigned char DPID_STANDBY_TIMEcount = 0;
unsigned char DPID_SENSE_STRESScount = 0;

extern u16 idata groupaddr[8];

//extern TYPE_BUFFER_S FlashBuffer;
void send_data(u8 d);
void reset_bt_module(void);
unsigned char PWM3init(unsigned char ab);
void savevar(void);
void Flash_EraseBlock(unsigned int fui_Address);//flash扇区擦除
void FLASH_WriteData(unsigned char fuc_SaveData, unsigned int fui_Address);//flash写入
void Delay_us_1(uint q1);

void reset_bt_module(void)
{
	send_data(0x55);//p15，重置模块
	send_data(0xAA);
	send_data(0x00);
	send_data(0x04);
	send_data(0x00);
	send_data(0x00);
	send_data(0x03);
}

/******************************************************************************
                                移植须知:
1:MCU必须在while中直接调用mcu_api.c内的bt_uart_service()函数
2:程序正常初始化完成后,建议不进行关串口中断,如必须关中断,关中断时间必须短,关中断会引起串口数据包丢失
3:请勿在中断/定时器中断内调用上报函数
******************************************************************************/

         
/******************************************************************************
                              第一步:初始化
1:在需要使用到bt相关文件的文件中include "bt.h"
2:在MCU初始化中调用mcu_api.c文件中的bt_protocol_init()函数
3:将MCU串口单字节发送函数填入protocol.c文件中uart_transmit_output函数内,并删除#error
4:在MCU串口接收函数中调用mcu_api.c文件内的uart_receive_input函数,并将接收到的字节作为参数传入
5:单片机进入while循环后调用mcu_api.c文件内的bt_uart_service()函数
******************************************************************************/

/******************************************************************************
                        1:dp数据点序列类型对照表
          **此为自动生成代码,如在开发平台有相关修改请重新下载MCU_SDK**         
******************************************************************************/
const DOWNLOAD_CMD_S xdata download_cmd[] =
{
  {DPID_SWITCH_LED, DP_TYPE_BOOL},
  {DPID_BRIGHT_VALUE, DP_TYPE_VALUE},
  {DPID_CDS, DP_TYPE_ENUM},
  {DPID_PIR_DELAY, DP_TYPE_VALUE},
  {DPID_SWITCH_XBR, DP_TYPE_BOOL},
  {DPID_STANDBY_TIME, DP_TYPE_VALUE},
  {DPID_SENSE_STRESS, DP_TYPE_VALUE},
  {DPID_ADDR, DP_TYPE_VALUE},
  {DPID_ADDREND, DP_TYPE_VALUE},
  {DPID_GROUP, DP_TYPE_VALUE},
  {DPID_DEBUG, DP_TYPE_STRING},
  //{DPID_TEST_BN0, DP_TYPE_BOOL},
  //{DPID_TEST_BN1, DP_TYPE_BOOL},
  //{DPID_TEST_BN2, DP_TYPE_BOOL},
  {DPID_SWITCH_LED2, DP_TYPE_BOOL},
  {DPID_SWITCH_LINKAGE, DP_TYPE_BOOL},
};




/******************************************************************************
                           2:串口单字节发送函数
请将MCU串口发送函数填入该函数内,并将接收到的数据作为参数传入串口发送函数
******************************************************************************/

/*****************************************************************************
函数名称 : uart_transmit_output
功能描述 : 发数据处理
输入参数 : value:串口收到字节数据
返回参数 : 无
使用说明 : 请将MCU串口发送函数填入该函数内,并将接收到的数据作为参数传入串口发送函数
*****************************************************************************/
void uart_transmit_output(unsigned char value)
{
// #error "请将MCU串口发送函数填入该函数,并删除该行"
    send_data(value);
	
/*
  //示例:
  extern void Uart_PutChar(unsigned char value);
  Uart_PutChar(value);	                                //串口发送函数
*/  
}
/******************************************************************************
                           第二步:实现具体用户函数
1:APP下发数据处理
2:数据上报处理
******************************************************************************/

/******************************************************************************
                            1:所有数据上报处理
当前函数处理全部数据上报(包括可下发/可上报和只上报)
  需要用户按照实际情况实现:
  1:需要实现可下发/可上报数据点上报
  2:需要实现只上报数据点上报
此函数为MCU内部必须调用
用户也可调用此函数实现全部数据上报
******************************************************************************/

//自动化生成数据上报函数

/*****************************************************************************
函数名称 : all_data_update
功能描述 : 系统所有dp点信息上传,实现APP和muc数据同步
输入参数 : 无
返回参数 : 无
使用说明 : 此函数SDK内部需调用;
           MCU必须实现该函数内数据上报功能;包括只上报和可上报可下发型数据
*****************************************************************************/
void all_data_update(void)
{
    u8 light;
    u8 radius;
  //#error "请在此处理可下发可上报数据及只上报数据示例,处理完成后删除该行"
  //此代码为平台自动生成，请按照实际数据修改每个可下发可上报函数和只上报函数
	
    mcu_dp_bool_update(DPID_SWITCH_LED, reset_bt_bn); //复位模块
    mcu_dp_bool_update(DPID_SWITCH_LED2, SWITCHflag2); //灯的开关
    mcu_dp_value_update(DPID_BRIGHT_VALUE, lightvalue); //VALUE型数据上报;

	if(LIGHT_TH==255)
		light=0;
	else if(LIGHT_TH==200)
		light=2;
	else if(LIGHT_TH==40)
		light=3;		
	else if(LIGHT_TH==20)
		light=4;
	else //if(LIGHT_TH==200)
		light=5;

    mcu_dp_enum_update(DPID_CDS, light); //枚举型数据上报;
    mcu_dp_value_update(DPID_PIR_DELAY, DELAY_NUM); //VALUE型数据上报;
    mcu_dp_bool_update(DPID_SWITCH_XBR, SWITCHfXBR); //BOOL型数据上报;
    mcu_dp_value_update(DPID_STANDBY_TIME, lowlightDELAY_NUM); //VALUE型数据上报;

	radius=TH/10000;
	radius=50-radius;

    mcu_dp_value_update(DPID_SENSE_STRESS, radius); //VALUE型数据上报;

    mcu_dp_value_update(DPID_ADDR, 10); //VALUE型数据上报;
    mcu_dp_value_update(DPID_ADDREND, 11); //VALUE型数据上报;
    mcu_dp_value_update(DPID_GROUP, 12); //VALUE型数据上报;


    mcu_dp_string_update(DPID_DEBUG, "1012", 4); //STRING型数据上报;


    //mcu_dp_bool_update(DPID_TEST_BN0,当前测试开关0); //BOOL型数据上报;
    //mcu_dp_bool_update(DPID_TEST_BN1,当前测试开关1); //BOOL型数据上报;
    //mcu_dp_bool_update(DPID_TEST_BN2,当前测试开关2); //BOOL型数据上报;
	
	mcu_dp_bool_update(DPID_SWITCH_LINKAGE,Linkage_flag); //BOOL型数据上报;



}


/******************************************************************************
                                WARNING!!!    
                            2:所有数据上报处理
自动化代码模板函数,具体请用户自行实现数据处理
******************************************************************************/

/*****************************************************************************
函数名称 : dp_download_switch_led_handle
功能描述 : 针对DPID_SWITCH_LED的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_switch_led_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char switch_led;
    
    switch_led = mcu_get_dp_download_bool(value,length);

    reset_bt_bn = switch_led;
  
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_SWITCH_LED, reset_bt_bn);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_bright_value_handle
功能描述 : 针对DPID_BRIGHT_VALUE的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_bright_value_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long bright_value;
    unsigned char i;
    
    bright_value = mcu_get_dp_download_value(value,length);
	
	DPID_BRIGHT_VALUEcount++;
	if(bright_value==lightvalue)
	{
/* 		if(DPID_BRIGHT_VALUEcount<2)
		{
			//DPID_BRIGHT_VALUEcount = 0;
			
			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_value_mesh_update(DPID_BRIGHT_VALUE,bright_value,groupaddr[i]);
				}
			}
		} */
	}
	else
	{
		DPID_BRIGHT_VALUEcount=0;
		for(i=0;i<8;i++)
		{
			if(groupaddr[i] != 0)
			{
				mcu_dp_value_mesh_update(DPID_BRIGHT_VALUE,bright_value,groupaddr[i]);
			}
		}
	}	
	
    lightvalue = bright_value;

	if(SWITCHfXBR==0)
	{
		XRBoffbrightvalue = bright_value;
	}
	
	savevar();
		
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_BRIGHT_VALUE, lightvalue);

    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_cds_handle
功能描述 : 针对DPID_CDS的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_cds_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为ENUM
    unsigned char ret;
    unsigned char cds;
    unsigned char i;
    
    cds = mcu_get_dp_download_enum(value,length);
	
	DPID_CDScount++;
	if(cds==cdsvalue)
	{
		if(DPID_CDScount<2)
		{
/* 			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_enum_mesh_update(DPID_CDS,cds,groupaddr[i]);
				}
			} */
		}
		if((cds==5)&&(light_ad!=LIGHT_TH))
		{
			DPID_CDScount=0;
		}
	}
	else
	{
		DPID_CDScount=0;
		for(i=0;i<8;i++)
		{
			if(groupaddr[i] != 0)
			{
				mcu_dp_enum_mesh_update(DPID_CDS,cds,groupaddr[i]);
			}
		}
	}	
	
    switch(cds) {
        case 0:		//2000LUS
			LIGHT_TH=255;//cds*4;
        break;
        
        case 1:		//300LUX
			LIGHT_TH=255;//cds*4;
        break;
        
        case 2:		//50LUX
			LIGHT_TH=200;
        break;
        
        case 3:	//10LUX
			LIGHT_TH=40;
        break;
        
        case 4:	//5LUX
			LIGHT_TH=20;
        break;
        
		case 5:
			LIGHT_TH = light_ad;
		break;
				
        default:
    
        break;
    }

    cdsvalue = cds;

    savevar();
    //sprintf(temp_str, "%3d", LIGHT_TH);
    //mcu_dp_string_update(DPID_DEBUG, temp_str, strlen(temp_str));
    //处理完DP数据后应有反馈
    ret = mcu_dp_enum_update(DPID_CDS, cdsvalue);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_pir_delay_handle
功能描述 : 针对DPID_PIR_DELAY的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_pir_delay_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long pir_delay;
    unsigned char i;
    
    pir_delay = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    */
	
	DPID_PIR_DELAYcount++;
	if(pir_delay==DELAY_NUM)
	{
/* 		if(DPID_PIR_DELAYcount<2)
		{
			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_value_mesh_update(DPID_PIR_DELAY,pir_delay,groupaddr[i]);
				}
			}
		} */
	}
	else
	{
		DPID_PIR_DELAYcount=0;
		for(i=0;i<8;i++)
		{
			if(groupaddr[i] != 0)
			{
				mcu_dp_value_mesh_update(DPID_PIR_DELAY,pir_delay,groupaddr[i]);
			}
		}
	}
	
    DELAY_NUM = pir_delay;
	savevar();
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_PIR_DELAY, DELAY_NUM);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_switch_xbr_handle
功能描述 : 针对DPID_SWITCH_XBR的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_switch_xbr_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char switch_xbr;
    unsigned char i;
    
    switch_xbr = mcu_get_dp_download_bool(value,length);
	
	DPID_SWITCH_XBRcount++;
	if(switch_xbr==SWITCHfXBR)
	{
/* 		if(DPID_SWITCH_XBRcount<2)
		{
			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_bool_mesh_update(DPID_SWITCH_XBR,switch_xbr,groupaddr[i]);
				}
			}
		} */
	}
	else
	{
		DPID_SWITCH_XBRcount=0;
		for(i=0;i<8;i++)
		{
			if(groupaddr[i] != 0)
			{
				mcu_dp_bool_mesh_update(DPID_SWITCH_XBR,switch_xbr,groupaddr[i]);
			}
		}
	
	}
	
    if(switch_xbr == 0) {
        //开关关
        SWITCHfXBR = 0;
        //mcu_dp_string_update(DPID_DEBUG, radar_bn_off, strlen(radar_bn_off));
    }else {
        //开关开
        SWITCHfXBR = 1;
        //mcu_dp_string_update(DPID_DEBUG, radar_bn_on, strlen(radar_bn_on));
    }
  
    savevar();
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_SWITCH_XBR,SWITCHfXBR);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_standby_time_handle
功能描述 : 针对DPID_STANDBY_TIME的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_standby_time_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long standby_time;
    unsigned char i;
    
    standby_time = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
	DPID_STANDBY_TIMEcount++;
	if(standby_time==lowlightDELAY_NUM)
	{
/* 		if(DPID_STANDBY_TIMEcount<2)
		{
			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_value_mesh_update(DPID_STANDBY_TIME,standby_time,groupaddr[i]);
				}
			}
		} */
	}
	else
	{
		DPID_STANDBY_TIMEcount=0;
		for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_value_mesh_update(DPID_STANDBY_TIME,standby_time,groupaddr[i]);
				}
			}
	
	}
	
    lowlightDELAY_NUM=standby_time;
    
    savevar();
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_STANDBY_TIME, lowlightDELAY_NUM);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_sense_stress_handle
功能描述 : 针对DPID_SENSE_STRESS的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_sense_stress_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long sense_stress;
    unsigned char i;
    
    sense_stress = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
	DPID_SENSE_STRESScount++;
	if(sense_stress==sensing_th)
	{
/* 		if(DPID_SENSE_STRESScount<2)
		{
			for(i=0;i<8;i++)
			{
				if(groupaddr[i] != 0)
				{
					mcu_dp_value_mesh_update(DPID_SENSE_STRESS,sense_stress,groupaddr[i]);
				}
			}
		} */
	}
	else
	{
		DPID_SENSE_STRESScount=0;
		for(i=0;i<8;i++)
		{
			if(groupaddr[i] != 0)
			{
				mcu_dp_value_mesh_update(DPID_SENSE_STRESS,sense_stress,groupaddr[i]);
			}
		}
	}	
	
	sensing_th = sense_stress;
	TH=(50-sense_stress)*10000;
		
	savevar();
    
    //sprintf(temp_str, "%6d", TH);
    //mcu_dp_string_update(DPID_DEBUG, temp_str, strlen(temp_str));    
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_SENSE_STRESS, sensing_th);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_addr_handle
功能描述 : 针对DPID_ADDR的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_addr_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long addr;
    
    addr = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_ADDR,addr);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_addrend_handle
功能描述 : 针对DPID_ADDREND的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_addrend_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long addrend;
    
    addrend = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_ADDREND,addrend);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_group_handle
功能描述 : 针对DPID_GROUP的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_group_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为VALUE
    unsigned char ret;
    unsigned long group;
    
    group = mcu_get_dp_download_value(value,length);
    /*
    //VALUE类型数据处理
    
    */
    
    //处理完DP数据后应有反馈
    ret = mcu_dp_value_update(DPID_GROUP,group);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_test_bn0_handle
功能描述 : 针对DPID_TEST_BN0的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
/* static unsigned char dp_download_test_bn0_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char test_bn0;
    
    test_bn0 = mcu_get_dp_download_bool(value,length);
    if(test_bn0 == 0) {
        //开关关
    }else {
        //开关开
    }
  
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_TEST_BN0,test_bn0);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
} */
/*****************************************************************************
函数名称 : dp_download_test_bn1_handle
功能描述 : 针对DPID_TEST_BN1的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
/* static unsigned char dp_download_test_bn1_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char test_bn1;
    
    test_bn1 = mcu_get_dp_download_bool(value,length);
    if(test_bn1 == 0) {
        //开关关
    }else {
        //开关开
    }
  
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_TEST_BN1,test_bn1);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
} */
/*****************************************************************************
函数名称 : dp_download_test_bn2_handle
功能描述 : 针对DPID_TEST_BN2的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
/* static unsigned char dp_download_test_bn2_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char test_bn2;
    
    test_bn2 = mcu_get_dp_download_bool(value,length);
    if(test_bn2 == 0) {
        //开关关
    }else {
        //开关开
    }
  
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_TEST_BN2,test_bn2);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
} */
/*****************************************************************************
函数名称 : dp_download_switch_led2_handle
功能描述 : 针对DPID_SWITCH_LED2的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_switch_led2_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char switch_led2;
    unsigned char i;
    
    switch_led2 = mcu_get_dp_download_bool(value,length);

    DPID_SWITCH_LED2count++;
    if(switch_led2==SWITCHflag2)
    {
/*     	if(DPID_SWITCH_LED2count<2)
    	{
    		for(i=0;i<8;i++)
    		{
    			if(groupaddr[i] != 0)
    			{
    				mcu_dp_bool_mesh_update(DPID_SWITCH_LED2,switch_led2,groupaddr[i]);
    			}
    		}
    	} */
    }
    else
    {
    	DPID_SWITCH_LED2count=0;
    	for(i=0;i<8;i++)
    	{
    		if(groupaddr[i] != 0)
    		{
    			mcu_dp_bool_mesh_update(DPID_SWITCH_LED2,switch_led2,groupaddr[i]);
    		}
    	}   

    }

    if(switch_led2 == 0) {
        //灯开关关
        SWITCHflag2=0;
    }else {
        //灯开关开
        //mcu_dp_string_update(DPID_DEBUG, led_bn_on, strlen(led_bn_on));
        if(SWITCHfXBR==1)
		{
			Light_on_flag=1;
		}
        SWITCHflag2=1;
    }
  
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_SWITCH_LED2, SWITCHflag2);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}
/*****************************************************************************
函数名称 : dp_download_switch_linkage_handle
功能描述 : 针对DPID_SWITCH_LINKAGE的处理函数
输入参数 : value:数据源数据
        : length:数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERROR
使用说明 : 可下发可上报类型,需要在处理完数据后上报处理结果至app
*****************************************************************************/
static unsigned char dp_download_switch_linkage_handle(const unsigned char value[], unsigned short length)
{
    //示例:当前DP类型为BOOL
    unsigned char ret;
    //0:关/1:开
    unsigned char switch_Linkage;
    unsigned char i;
    switch_Linkage = mcu_get_dp_download_bool(value,length);
		
	if(switch_Linkage==Linkage_flag)
	{
		//
	}
	else
	{
		for(i=0;i<8;i++)
		{
			if(groupaddr[i] != 0)
			{
				mcu_dp_bool_mesh_update(DPID_SWITCH_LINKAGE,switch_Linkage,groupaddr[i]);
			}
		}
	}
    if(switch_Linkage == 0) {
        //雷达开关关
        //LIGHT_OFF;
        //PWM3init(0);
        Linkage_flag=0;
    }else {
        //雷达开关开
        //LIGHT_ON;
        //PWM3init(100);
        Linkage_flag=1;
    }
  	
    //处理完DP数据后应有反馈
    ret = mcu_dp_bool_update(DPID_SWITCH_LINKAGE,switch_Linkage);
    if(ret == SUCCESS)
        return SUCCESS;
    else
        return ERROR;
}

/******************************************************************************
                                WARNING!!!                     
此代码为SDK内部调用,请按照实际dp数据实现函数内部数据
******************************************************************************/
#ifdef SUPPORT_MCU_FIRM_UPDATE
/*****************************************************************************
函数名称 : mcu_firm_update_handle
功能描述 : MCU进入固件升级模式
输入参数 : value:固件缓冲区
           position:当前数据包在于固件位置
           length:当前固件包长度(固件包长度为0时,表示固件包发送完成)
返回参数 : 无
使用说明 : MCU需要自行实现该功能
*****************************************************************************/
unsigned char mcu_firm_update_handle(const unsigned char value[],unsigned long position,unsigned short length)
{
  #error "请自行完成MCU固件升级代码,完成后请删除该行"
  unsigned long addr;
 
  if(length == 0)
  {
#ifdef ENABLE_BOOT
    //固件数据发送完成
    FlashBuffer.magic_code = FIREWARE_UPDATE_FLAG;
    
    if(Earse_Flash(PARA_ADDR) == ERROR)
      return ERROR;
    
    //写入升级标志
    if(Write_Flash(PARA_ADDR,(unsigned char *)&FlashBuffer,sizeof(FlashBuffer)) == ERROR)
      return ERROR;
    
    Reset();
#endif
  }
  else
  {
    //固件数据处理
    addr = FIREWARE_ADDR_H;
     
    if(position % 1024 == 0)
    {
      if(Earse_Flash(addr + position) == ERROR)
        return ERROR;
    }
    
    if(Write_Flash(addr + position,(unsigned char *)value,length) == ERROR)
      return ERROR;
  }

  return SUCCESS;
}
#endif
/******************************************************************************
                                WARNING!!!                     
以下函数用户请勿修改!!
******************************************************************************/

/*****************************************************************************
函数名称 : dp_download_handle
功能描述 : dp下发处理函数
输入参数 : dpid:DP序号
           value:dp数据缓冲区地址
           length:dp数据长度
返回参数 : 成功返回:SUCCESS/失败返回:ERRO
使用说明 : 该函数用户不能修改
*****************************************************************************/
unsigned char dp_download_handle(unsigned char dpid,const unsigned char value[], unsigned short length)
{
  /*********************************
  当前函数处理可下发/可上报数据调用                    
  具体函数内需要实现下发数据处理
  完成用需要将处理结果反馈至APP端,否则APP会认为下发失败
  ***********************************/
  unsigned char ret;
  switch(dpid)
  {
        case DPID_SWITCH_LED:
            //开关处理函数
            ret = dp_download_switch_led_handle(value,length);
			if(ret==1)
			{
				switchcnt ++;
				if(switchcnt>=5)
				{
					switchcnt = 0;
                    reset_bt_module();
				}
			}
        break;
        case DPID_BRIGHT_VALUE:
            //亮度值处理函数
            ret = dp_download_bright_value_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_CDS:
            //光敏参数处理函数
            ret = dp_download_cds_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_PIR_DELAY:
            //感应延时处理函数
            ret = dp_download_pir_delay_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_SWITCH_XBR:
            //感应开关处理函数
            ret = dp_download_switch_xbr_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_STANDBY_TIME:
            //伴亮延时处理函数
            ret = dp_download_standby_time_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_SENSE_STRESS:
            //感应强度处理函数
            ret = dp_download_sense_stress_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_ADDR:
            //设备地址处理函数
            ret = dp_download_addr_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_ADDREND:
            //设备地址结束值处理函数
            ret = dp_download_addrend_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_GROUP:
            //设备群组处理函数
            ret = dp_download_group_handle(value,length);
            switchcnt = 0;
        break;
/*         case DPID_TEST_BN0:
            //测试开关0处理函数
            ret = dp_download_test_bn0_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_TEST_BN1:
            //测试开关1处理函数
            ret = dp_download_test_bn1_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_TEST_BN2:
            //测试开关2处理函数
            ret = dp_download_test_bn2_handle(value,length);
            switchcnt = 0;
        break; */
        case DPID_SWITCH_LED2:
            //灯开关处理函数
            ret = dp_download_switch_led2_handle(value,length);
            switchcnt = 0;
        break;
        case DPID_SWITCH_LINKAGE:
            //联动 处理函数
            ret = dp_download_switch_linkage_handle(value,length);
            switchcnt = 0;
        break;

  default:
        switchcnt = 0;
    break;
  }
  return ret;
}
/*****************************************************************************
函数名称 : get_download_cmd_total
功能描述 : 获取所有dp命令总和
输入参数 : 无
返回参数 : 下发命令总和
使用说明 : 该函数用户不能修改
*****************************************************************************/
unsigned char get_download_cmd_total(void)
{
  return(sizeof(download_cmd) / sizeof(download_cmd[0]));
}

void savevar(void)
{
	unsigned char i;
	Flash_EraseBlock(0x2F00);
	Delay_us_1(10000);

	i=(TH/1000)>>8;
	FLASH_WriteData(i,0x2F00+0);
	Delay_us_1(100);
	
    i=(TH/1000)&0xff;
	FLASH_WriteData(i,0x2F00+1);
	Delay_us_1(100);
	
    i=LIGHT_TH;
	FLASH_WriteData(i,0x2F00+2);
	Delay_us_1(100);
	
	i=DELAY_NUM>>8;
	FLASH_WriteData(i,0x2F00+3);
	Delay_us_1(100);
	i=DELAY_NUM&0xff;//&0xff;
	FLASH_WriteData(i,0x2F00+4);
	Delay_us_1(100);
	
	i=lightvalue;
	FLASH_WriteData(i,0x2F00+5);
	Delay_us_1(100);
	
	i=lowlightDELAY_NUM;
	FLASH_WriteData(i,0x2F00+6);
	Delay_us_1(100);
	
	i=SWITCHfXBR;//&0xff;
	FLASH_WriteData(i,0x2F00+7);
	Delay_us_1(100);
	
//	i=addr;//&0xff;
//	FLASH_WriteData(i,0X2F00+7);
//	Delay_us_1(100);
//	
//	i=devgroup;//&0xff;
//	FLASH_WriteData(i,0X2F00+8);
//	Delay_us_1(100);

//	i=addrend;
//	FLASH_WriteData(i,0X2F00+9);
//	Delay_us_1(100);
	
	Flash_EraseBlock(0x2F80);
	Delay_us_1(10000);
	FLASH_WriteData(0,0x2F80+0);
	
	EA=1;				//-20200927

}


