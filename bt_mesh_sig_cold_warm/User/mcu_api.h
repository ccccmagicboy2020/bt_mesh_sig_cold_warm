/****************************************Copyright (c)*************************
**                               版权所有 (C), 2015-2017, 涂鸦科技
**
**                                 http://www.tuya.com
**
**--------------文件信息-------------------------------------------------------
**文   件   名: mcu_api.c
**描        述: 下发/上报数据处理函数
**使 用 说 明 : 此文件下函数无须用户修改,用户需要调用的文件都在该文件内
**
**
**--------------当前版本修订---------------------------------------------------
** 版  本: v1.0
** 日　期: 2017年5月3日
** 描　述: 1:创建涂鸦bluetooth对接MCU_SDK
**
**-----------------------------------------------------------------------------
******************************************************************************/
#ifndef __MCU_API_H_
#define __MCU_API_H_


#ifdef MCU_API_GLOBAL
  #define MCU_API_EXTERN
#else
  #define MCU_API_EXTERN   extern
#endif


#define LIGHT_ON {}
#define LIGHT_OFF {}
/*****************************************************************************
函数名称 : hex_to_bcd
功能描述 : hex转bcd
输入参数 : Value_H:高字节/Value_L:低字节
返回参数 : bcd_value:转换完成后数据
*****************************************************************************/
unsigned char hex_to_bcd(unsigned char Value_H,unsigned char Value_L);
/*****************************************************************************
函数名称 : my_strlen
功能描述 : 求字符串长度
输入参数 :
返回参数 :
*****************************************************************************/
unsigned long my_strlen(unsigned char *str);

/*****************************************************************************
函数名称 : my_memset
功能描述 : 把src所指内存区域的前count个字节设置成字符c
输入参数 :
返回参数 :
*****************************************************************************/
void *my_memset(void *src,unsigned char ch,unsigned short count);

/*****************************************************************************
函数名称 : mymemcpy
功能描述 : 内存拷贝
输入参数 :
返回参数 :
*****************************************************************************/
void *my_memcpy(void *dest, const void *src, unsigned short count);
/*****************************************************************************

函数名称 : int_to_byte
功能描述 : 将int类型拆分四个字节
输入参数 : number:4字节原数据;value:处理完成后4字节数据
返回参数 :无
****************************************************************************/
void int_to_byte(unsigned long number,unsigned char value[4]);

/*****************************************************************************
函数名称 : byte_to_int
功能描述 : 将4字节合并为1个32bit变量
输入参数 : value:4字节数组
返回参数 : number:合并完成后的32bit变量
****************************************************************************/
unsigned long byte_to_int(const unsigned char value[4]);

#ifndef BT_CONTROL_SELF_MODE
/*****************************************************************************
函数名称 : mcu_get_reset_bt_flag
功能描述 : MCU获取复位bt成功标志
输入参数 : 无
返回参数 : 复位标志:RESET_BT_ERROR:失败/RESET_BT_SUCCESS:成功
使用说明 : 1:MCU主动调用mcu_reset_bt()后调用该函数获取复位状态
           2:如果为模块自处理模式,MCU无须调用该函数
*****************************************************************************/
unsigned char mcu_get_reset_bt_flag(void);
/*****************************************************************************
函数名称 : mcu_reset_bt
功能描述 : MCU主动重置bt工作模式
输入参数 : 无
返回参数 : 无
使用说明 : 1:MCU主动调用,通过mcu_get_reset_bt_flag()函数获取重置bt是否成功
           2:如果为模块自处理模式,MCU无须调用该函数
*****************************************************************************/
void mcu_reset_bt(void);
/*****************************************************************************
函数名称 : mcu_get_bt_work_state
功能描述 : MCU主动获取当前bt工作状态
输入参数 : 无
返回参数 : BT_WORK_SATE_E:
          SMART_CONFIG_STATE:smartconfig配置状态
          AP_STATE:AP 配置状态
          BT_NOT_CONNECTED:BT 配置成功但未连上路由器
          BT_CONNECTED:BT 配置成功且连上路由器
使用说明 : 无
*****************************************************************************/
unsigned char mcu_get_bt_work_state(void);
#endif

/*****************************************************************************
函数名称 : mcu_dp_raw_update
功能描述 : raw型dp数据上传
输入参数 : dpid:id号
           value:当前dp值指针
           len:数据长度
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_raw_update(unsigned char dpid,const unsigned char value[],unsigned short len);

/*****************************************************************************
函数名称 : mcu_dp_bool_update
功能描述 : bool型dp数据上传
输入参数 : dpid:id号
           value:当前dp值
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_bool_update(unsigned char dpid,unsigned char value);
unsigned char mcu_dp_bool_mesh_update(unsigned char dpid,unsigned char value,unsigned int groupa);
unsigned char mcu_dp_enum_mesh_update(unsigned char dpid,unsigned char value,unsigned int groupa);
/*****************************************************************************
函数名称 : mcu_dp_value_update
功能描述 : value型dp数据上传
输入参数 : dpid:id号
           value:当前dp值
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_value_update(unsigned char dpid,unsigned long value);
unsigned char mcu_dp_value_mesh_update(unsigned char dpid,unsigned long value,unsigned int groupa);
/*****************************************************************************
函数名称 : mcu_dp_string_update
功能描述 : rstring型dp数据上传
输入参数 : dpid:id号
           value:当前dp值指针
           len:数据长度
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_string_update(unsigned char dpid,const unsigned char value[],unsigned short len);

/*****************************************************************************
函数名称 : mcu_dp_enum_update
功能描述 : enum型dp数据上传
输入参数 : dpid:id号
           value:当前dp值
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_enum_update(unsigned char dpid,unsigned char value);

/*****************************************************************************
函数名称 : mcu_dp_fault_update
功能描述 : fault型dp数据上传
输入参数 : dpid:id号
           value:当前dp值
返回参数 : 无
*****************************************************************************/
unsigned char mcu_dp_fault_update(unsigned char dpid,unsigned long value);

/*****************************************************************************
函数名称 : mcu_get_dp_download_bool
功能描述 : mcu获取bool型下发dp值
输入参数 : value:dp数据缓冲区地址
           length:dp数据长度
返回参数 : bool:当前dp值
*****************************************************************************/
unsigned char mcu_get_dp_download_bool(const unsigned char value[],unsigned short len);

/*****************************************************************************
函数名称 : mcu_get_dp_download_value
功能描述 : mcu获取value型下发dp值
输入参数 : value:dp数据缓冲区地址
           length:dp数据长度
返回参数 : value:当前dp值
*****************************************************************************/
unsigned long mcu_get_dp_download_value(const unsigned char value[],unsigned short len);

/*****************************************************************************
函数名称 : mcu_get_dp_download_enum
功能描述 : mcu获取enum型下发dp值
输入参数 : value:dp数据缓冲区地址
           length:dp数据长度
返回参数 : enum:当前dp值
*****************************************************************************/
unsigned char mcu_get_dp_download_enum(const unsigned char value[],unsigned short len);

/*****************************************************************************
函数名称 : uart_receive_input
功能描述 : 收数据处理
输入参数 : value:串口收到字节数据
返回参数 : 无
使用说明 : 在MCU串口处理函数中调用该函数,并将接收到的数据作为参数传入
*****************************************************************************/
void uart_receive_input(unsigned char value);

/*****************************************************************************
函数名称  : bt_uart_service
功能描述  : bt串口处理服务
输入参数 : 无
返回参数 : 无
使用说明 : 在MCU主函数while循环中调用该函数
*****************************************************************************/
void bt_uart_service(void);

/*****************************************************************************
函数名称 :  bt_protocol_init
功能描述 : 协议串口初始化函数
输入参数 : 无
返回参数 : 无
使用说明 : 在MCU初始化代码中调用该函数
*****************************************************************************/
void bt_protocol_init(void);
/*****************************************************************************
函数名称 : uart_transmit_output
功能描述 : 发数据处理
输入参数 : value:串口收到字节数据
返回参数 : 无
使用说明 : 请将MCU串口发送函数填入该函数内,并将接收到的数据作为参数传入串口发送函数
*****************************************************************************/
void uart_transmit_output(unsigned char value);

#endif