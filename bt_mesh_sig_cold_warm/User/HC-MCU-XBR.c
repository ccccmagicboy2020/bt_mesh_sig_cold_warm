#define ALLOCATE_EXTERN
#include "HC89S003F4.h"
//#include "Mcu_api.h"
#include "bluetooth.h"

#define V12 //硬件板卡的版本
//#define V10

//#define  VERSION  0X21

#define TH_LOW 30000
#define TH_HIGH 4000000

#define TH_DEF 40000

//允许噪声值偏差范围
#define MAX_DELTA0 20000 //最大偏差低值
#define MAX_DELTA1 60000 //最大偏差高值

#define MAX_DELAY 1800
//最大延时秒数

//感光门限-30对应8LUX左右的AD值,设置为255表示不检测感光
#define LIGHT_TH0 255
//30

//一个检测周期为0.25秒
//#define DELAY_NUM 20

//#define LIGHT_ON P0_6=0
//#define LIGHT_OFF P0_6=1

//void Delay_2us(u16 Cnt);		//延时函数

// bit Timer1_FLAG;
// bit Time_10mS_FLAG;
// bit Time_100mS_FLAG;
// bit Time_200mS_FLAG;
// bit Time_500mS_FLAG;
// bit Time_1S_FLAG;
// bit Time_10S_FLAG;
// bit Time_1Min_FLAG;
// bit Time_1H_FLAG;

volatile ulong Timer_Counter = 0;

//u8 Uart1_RX_Buff[10] = {0};					 //用于存放UART1接收数据
//u8 Uart1_RX_Cnt = 0;								 //UART1接收计数
//u8 Uart1_TX_Buff[20] = {0};					 //用于存放UART1发送数据
//u8 Uart1_TX_Cnt = 0;								 //UART1发送计数

// u16 AN1_Data = 0;
// u16 AN7_Data = 0;
// u8 Uart_Cnt = 0;

u8 xdata SUM1_counter = 0; //???
u8 xdata SUM0_num = 12;	   //???
u8 xdata SUM1_num = 64;	   //???
ulong xdata SUM01;
//ulong xdata SUM2;		   //调试用
ulong xdata SUM10 = 0;	   //SUM1值的几次平均值，时间上的滞后值
ulong xdata SUM0 = 0;	   //
ulong xdata SUM1 = 0;	   //平均绝对离差的累加合的瞬时值
ulong xdata ALL_SUM1 = 0;  //SUM1的累加值
ulong xdata SUM16 = 0;	   //2^16次的累计值变量
ulong xdata SUM = 0;	   //an1的raw累加值
u16 xdata start_times = 1; //???
u16 xdata times = 0;	   //主循环次数
ulong xdata TH;			   //设置误差阈值，可由APP设置的感应强度转换
ulong xdata MAX_DELTA; //最大偏差值
u8 xdata alarm_times = 0;
u8 xdata stop_times = 0; //???

uint xdata LIGHT = 0;	  //伴亮灯秒的计数器
uint xdata LIGHT_off = 0; //无人灭灯的分钟计数器
uint xdata average;		  //an1的raw平均值

u8 xdata light_ad;	//光敏实时值raw
u8 xdata light_ad0; //光敏初始瞬时值raw

u8 xdata check_sum, send_byte;
u8 xdata check_light_times = 8;	 //用于光敏检查的计数器
u8 xdata calc_average_times = 0; //用于计算平均值的计数器
u8 xdata LIGHT_TH;
u16 xdata DELAY_NUM;
u8 xdata lowlightDELAY_NUM;
u8 xdata RXnum = 0;
u8 while_1flag = 0;		  //伴亮完成标志
u8 while_2flag = 0;		  //???
u8 xdata SWITCHflag = 0;  //暂时没有使用
u8 xdata SWITCHflag2 = 0; //灯开关的变量，可由APP设置
u8 xdata SWITCHfXBR = 1;  //雷达感应开关的变量，可由APP设置
u8 xdata lightvalue = 10; //亮度值，可由APP设置
u8 xdata switchcnt = 0;
u8 xdata slowchcnt = 10;				  //亮度渐量值
u8 xdata resetbtcnt = 0;				  //为重置蓝牙模块设置的计数器
u8 xdata XRBoffbrightvalue = 0;			  //当关闭雷达时，APP设置的亮度值
volatile u16 xdata lowlight1mincount = 0; //timer的计数器1ms自加
volatile u8 xdata lowlight1minflag = 0;	  //timer的分钟标志
volatile u16 idata light1scount = 0;	  //timer的计数器1ms自加
volatile u16 idata light1sflag = 0;		  //timer的秒标志
u8 xdata addr = 0;
u8 xdata devgroup = 0;
u8 xdata addrend = 0;
u16 idata groupaddr[8] = {0};
u8 idata check_group_flag = 0;	//检查群组标志
u8 idata check_group_count = 0; //检查群组计数器
u8 idata Linkage_flag = 0;
u8 idata Light_on_flag = 0;
u8 idata Light_on_flagpre = 0;

/*
	 u8 idata groupaddr2 = 0;
	 u8 idata groupaddr3 = 0;
	 u8 idata groupaddr4 = 0;
	 u8 idata groupaddr5 = 0;
	 u8 idata groupaddr6 = 0;
	 u8 idata groupaddr7 = 0;
	 u8 idata groupaddr8 = 0;
	 */
unsigned char PWM3init(unsigned char ab);
void Flash_EraseBlock(unsigned int fui_Address); //扇区擦除
//void FLASH_WriteData(unsigned char fui_Address, unsigned int fuc_SaveData);//写入一个数据
void FLASH_WriteData(unsigned char fuc_SaveData, unsigned int fui_Address);
//void Flash_WriteArr(unsigned int fui_Address,unsigned char fuc_Length,unsigned char *fucp_SaveArr);//写入任意长度数据
void Flash_ReadArr(unsigned int fui_Address, unsigned char fuc_Length, unsigned char *fucp_SaveArr); //读取任意长度数据

//unsigned char guc_Write_a[5] = {0};	//写入数据
unsigned char xdata guc_Read_a[10] = {0x00}; //用于存放读取的数据
unsigned char xdata guc_Read_a1[1] = {0x00}; //用于存放读取的数据
// unsigned char guc_Uartflag = 0;					  //发送标志位
// unsigned char guc_Uartcnt = 0;					  //发送计数
// unsigned char guc_Uartbuf_a[2] = {0x00};	//缓存数组

void Flash_ReadArr(unsigned int fui_Address, unsigned char fuc_Length, unsigned char *fucp_SaveArr)
{
	while (fuc_Length--)
		*(fucp_SaveArr++) = *((unsigned char code *)(fui_Address++)); //读取数据
}

void Delay_us(uint q)
{
	uint j;
	for (j = 0; j < q; j++)
	{
		;
	}
}

void Delay_ms(uint t)
{
	Timer_Counter = 0;
	while (Timer_Counter < t)
	{
		WDTC |= 0x10; //清看门狗
	}
}

void Delay_us_1(uint q1)
{
	uint j;
	for (j = 0; j < q1; j++)
	{
		;
	}
}

// void Delay_ms(uint t)
// {
// 	for(;t>0;t--)
// 	{
// 		Delay_us_1(1000);
// 		WDTC |= 0x10;		//清看门狗
// 	}
// }

/***************************************************************************************
  * @说明  	系统初始化函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/
void InitSYS()
{
	/********************************系统频率初始化***************************************/

	CLKSWR = 0x51;	 //选择内部高频RC为系统时钟，内部高频RC 2分频，Fosc=16MHz
	CLKDIV = 0x01;	 //Fosc 1分频得到Fcpu，Fcpu=16MHz
	FREQ_CLK = 0x10; //IAP频率

	/**********************************低压复位初始化**************************************/

	//	BORC = 0xC0;											 //使能低压复位1.8V，带消抖使能
	//	BORDBC = 0x01;										 //消抖时间BORDBC*8TCPU+2TCPU

	/***********************************看门口初始化***************************************/
	WDTC = 0x5F;   //允许WDT复位，空闲模式下禁止WDT，选择1024分频（内部低频时钟44K）
	WDTCCR = 0X20; //0X20/44	=0.73秒						//0xFF;	 //溢出时间约6秒
				   //溢出计算时间=（WDT分频系数*（WDTCCR+1））/内部低频RC频率
}

/***************************************************************************************
  * @说明  	定时器初始化函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/
void Timer_Init()
{
	/**********************************TIM1配置初始化**************************************/
	TCON1 = 0x00; //T1定时器时钟为Fosc
	TMOD = 0x01;  //T1-16位重装载定时器/计数器,T0-16位定时器

	//Tim1计算时间 	= (65536 - 0xFACB) * (1 / (Fosc /Timer分频系数))
	//				= 1333 / (16000000 / 12)
	//				= 1 ms

	//T1定时1ms
	//反推初值 	= 65536 - ((1/1000) / (1/(Fosc / Timer分频系数)))
	//		   	= 65536 - ((1/1000) / (1/(16000000 / 12)))
	//			= 65536 - 1333
	//			= 0xFACB

	TH1 = 0xFA;
	TL1 = 0xCB;	  //T1定时1ms
	IE |= 0x08;	  //打开T1中断
	TCON |= 0x40; //使能T1

	TH0 = 0xCB;
	TL0 = 0xEB; //T0定时时间10ms

	TCON |= 0x10; //使能T0
}

/***************************************************************************************
  * @说明  	UART1初始化函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/
void UART1_Init()
{
	/**********************************UART配置初始化**************************************/
	P2M0 = P2M0 & 0xF0 | 0x08; //P20设置为推挽输出
	P0M2 = P0M2 & 0xF0 | 0x02; //P04设置为上拉输入
	P0_4 = 1;
	TXD_MAP = 0x20; //TXD映射P20
	RXD_MAP = 0x04; //RXD映射P04
	T4CON = 0x06;	//T4工作模式：UART1波特率发生器

	//波特率计算
	//波特率 = 1/16 * (T4时钟源频率 / 定时器4预分频比) / (65536 - 0xFF98)
	//       = 1/16 * ((16000000 / 1) / 104)
	//		 = 9615.38(误差0.16%)

	//波特率9600
	//反推初值 = (65536 - ((T4时钟源频率 / 定时器4预分频比) * (1 / 16)) / 波特率)
	//		   = (65536 - (16000000 * (1 / 16) / 9600))
	//		   = (65536 - 104.167)
	//         = FF98
	//0xFF98->9600
	//0xFFCC->19200
	//0xFFEF->57600

	TH4 = 0xFF;
	TL4 = 0x98;	  //波特率9600		//0xEE;				//波特率56000
	SCON2 = 0x02; //8位UART，波特率可变
	SCON = 0x10;  //允许串行接收
	IE |= 0X10;	  //使能串口中断
				  //EA = 1;							              	 //使能总中断
}

/***************************************************************************************
  * @说明  	ADC初始化函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/
void ADC_Init()
{

	ADCC0 |= 0x03; //参考源为内部2V
	ADCC0 |= 0x80; //打开ADC转换电源
	Delay_us(20);  //延时20us，确保ADC系统稳定
	ADCC1 = 0x01;  //选择外部通道1
	ADCC2 = 0x4B;  //8分频	  //转换结果12位数据，数据右对齐，ADC时钟16分频-1MHZ//0X4B-8分频//0X49-4分频
}

/***************************************************************************************
  * @说明  	IO口初始化函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/
void GPIO_Init()
{
	//P0M0分高4位与低4位，低4位控制P00输入输出，高4位控制P01输入输出，其他以此类推
	//P0M1高4控制P03，低4控制P02
	//P1M2高4控制P15，低4控制P14

	// 	P0M0 = P0M0&0xF0|0x08;		      //P00设置为推挽输出
	// 	P0M0 = P0M0&0x0F|0x30;				  //P01设置为模拟输入
	// 	P0M3 = P0M3&0x0F|0x30;				  //P07设置为模拟输入
	// 	P0M3 = P0M3&0xF0|0x08;		      //P06设置为推挽输出

#ifdef V11

	P0M0 = P0M0 & 0xF0 | 0x08; //P00设置为推挽输出

	P0M0 = P0M0 & 0x0F | 0x30; //P01设置为模拟输入
	//P0M3 = P0M3&0x0F|0x30;				  //P07设置为模拟输入
	//	P0M0 = P0M0&0x0F|0x80;		      //P01设置为推挽输出

	P0M2 = P0M2 & 0x0F | 0x80; //P05设置为推挽输出

	P0M3 = P0M3 & 0xF0 | 0x03; //P06设置为模拟输入  //|0x08;		      //P06设置为推挽输出

	P0M3 = P0M3 & 0x0F | 0x20; //P07设置为上拉输入

#endif

#ifdef V10

	P0M0 = P0M0 & 0xF0 | 0x08; //P00

	P0M0 = P0M0 & 0x0F | 0x30; //P01
	P0M3 = P0M3 & 0x0F | 0x30; //P07
							   //	P0M0 = P0M0&0x0F|0x80;		      //P01

	P0M3 = P0M3 & 0xF0 | 0x08; //P06

#endif

#ifdef V12

	P1M0 = P1M0 & 0xF0 | 0x08; //P10设置为推挽输出

	P0M0 = P0M0 & 0x0F | 0x30; //P01设置为模拟输入

	P2M1 = P2M1 & 0xF0 | 0x03; //P22设置为模拟输入

	//P0M3 = P0M3&0x0F|0x30;				  //P07设置为模拟输入
	//	P0M0 = P0M0&0x0F|0x80;		      //P01设置为推挽输出

	//P0M2 = P0M2&0x0F|0x80;		      //P05设置为推挽输出

	//P0M3 = P0M3&0xF0|0x03;			//P06设置为模拟输入  //|0x08;		      //P06设置为推挽输出

	//	P0M3 = P0M3&0x0F|0x20;				  //P07设置为上拉输入

#endif
}

// u16 Read_ADC(u8 Channel)
// {
// 	u16 ADC_Temp;
// 	ADCC1 = Channel;						  //选择外部通道1
//   ADCC0 |= 0x40;					//启动ADC转换
// 	while(!(ADCC0&0x20));		//等待ADC转换结束
// 	ADCC0 &=~ 0x20;					//清除标志位
// 	ADC_Temp = ADCR;				//获取ADC的值
// 	return ADC_Temp;
// }

/***************************************************************************************
  * @说明  	系统时钟计数函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/
/*
void System_Time_Cnt(void)
{
    static u32 Timer1Count1 = 0;
    static u8 Timer1Count2 = 0;
    u32 Tampe = 1;
    if(Timer1_FLAG)              //1ms
    {
        Timer1_FLAG = 0;
        Timer1Count1 ++;
        Tampe = Timer1Count1 % 10;
        if(Tampe == 0)
            Time_10mS_FLAG = 1;
        Tampe = Timer1Count1 % 100;
        if(Tampe == 0)
            Time_100mS_FLAG = 1;
        Tampe = Timer1Count1 % 200;
        if(Tampe == 0)
            Time_200mS_FLAG = 1;
        Tampe = Timer1Count1 % 500;
        if(Tampe == 0)
            Time_500mS_FLAG = 1;
        Tampe = Timer1Count1 % 1000;
        if(Tampe == 0)
            Time_1S_FLAG = 1;
        Tampe = Timer1Count1 % 10000;
        if(Tampe == 0)
        {
            Time_10S_FLAG = 1;
            Timer1Count1 = 0;
            Timer1Count2 ++;
            Tampe = Timer1Count2 % 6;
            if(Tampe == 0)
                Time_1Min_FLAG = 1;
            Tampe = Timer1Count2 % 60;
            if(Tampe == 0)
            {
                Time_1H_FLAG = 1;
                Timer1Count2 = 0;
            }
        }
    }
}
*/

/***************************************************************************************
  * @说明  	系统任务处理函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/

/*
void System_Task(void)
{
    u8 temp;
		if(Time_10mS_FLAG)
    {
        Time_10mS_FLAG = 0;
        //User TASK
        
        
        //User TASK
        if(Time_100mS_FLAG)
        {
            Time_100mS_FLAG = 0;
            //User TASK

        
        
            //User TASK
        }
        if(Time_200mS_FLAG)
        {
            Time_200mS_FLAG = 0;
            //User TASK

        
        
            //User TASK
        }
        if(Time_500mS_FLAG)
        {
            Time_500mS_FLAG = 0;
            //User TASK
						AN1_Data = Read_ADC(0x01);
					
            //User TASK
        }
         if(Time_1S_FLAG)
        {
            Time_1S_FLAG = 0;
            //User TASK

						temp = 0xFF & (AN1_Data >> 8);
						SBUF = temp;
						while(!(SCON & 0x02));
						SCON &=~ 0x02;			           //清除发送中断标志位
						temp = 0xFF & AN1_Data;
						SBUF = temp;
						while(!(SCON & 0x02));
						SCON &=~ 0x02;			           //清除发送中断标志位
						P0_0 =~ P0_0;	
						//User TASK
            
        }
        if(Time_10S_FLAG)
        {
            Time_10S_FLAG = 0;
            //User TASK
            

            //User TASK
        }
        if(Time_1Min_FLAG)
        {
            Time_1Min_FLAG = 0;
            //User TASK
        
        
        
            //User TASK
        }
        if(Time_1H_FLAG)
        {
            Time_1H_FLAG = 0;
            //User TASK
        
        
        
            //User TASK
        }
    }
}
*/

void send_data(u8 d)
{
	SBUF = d;
	while (!(SCON & 0x02))
		;
	SCON &= ~0x02;
}

//return 8-bit adc raw
uchar read_ad(uchar ch)
{
	u8 i;
	uint ad_sum;

	//	switch (ch)
	//	{
	//		case 3:
	//			ADC_P16_AN3;
	//			break;
	//		case 4:
	//			ADC_P15_AN4;
	//			break;
	//		case 5:
	//			ADC_P14_AN5;
	//			break;
	//	}

	// 	if(ch==3)
	// 	{
	// 		ADC_P16_AN3;
	// 	}
	// 	else if(ch==4)
	// 	{
	// 		ADC_P15_AN4;
	// 	}

	ADCC1 = ch;	   //选择外部通道
	ADCC0 |= 0x40; //启动ADC转换
	while (!(ADCC0 & 0x20))
		;			//等待ADC转换结束
	ADCC0 &= ~0x20; //清除标志位

	//ADC_Temp = ADCR;				//获取ADC的值

	// 	delay_ms(1);
	//
	// 	ADC_TG;
	Delay_us(100);

	ad_sum = 0;
	//ADC_INT_IF_CLR; //清中断标志位

	for (i = 0; i < 16; i++)
	{
		// 		ADC_TG;
		// 		while(ADC_IF==0){};
		// 		//adc_data = ADC_DATA_RD();
		//
		// 		k=ADC_DH;
		// 		k =k <<8;
		// 		k+=ADC_DL;
		//
		// 		ADC_INT_IF_CLR; //清中断标志位
		// 		ad_sum+=k;

		ADCC0 |= 0x40; //启动ADC转换
		while (!(ADCC0 & 0x20))
			;			//等待ADC转换结束
		ADCC0 &= ~0x20; //清除标志位
		ad_sum += ADCR; //获取ADC的值

		Delay_us(20);
	}

	//ADC_P14_AN5;
	ADCC1 = 1; //切换到an1
	i = ad_sum >> 8;

	Delay_us(100);
	return (i);
}

void set_var(void)
{

	Flash_ReadArr(0X2f00, 10, guc_Read_a); //读取地址0x2F00所在扇区

	TH = guc_Read_a[0];
	TH <<= 8;
	TH += guc_Read_a[1];
	TH *= 1000;
	if (TH < TH_LOW || TH > TH_HIGH)
		TH = TH_DEF;

	LIGHT_TH = guc_Read_a[2];

	if (LIGHT_TH == 0)
		LIGHT_TH = LIGHT_TH0;
	else if (LIGHT_TH == 0XFE)
		LIGHT_TH = 255;

	DELAY_NUM = guc_Read_a[3];
	DELAY_NUM <<= 8;
	DELAY_NUM += guc_Read_a[4];
	if (DELAY_NUM == 0 || DELAY_NUM > MAX_DELAY)
		DELAY_NUM = 5;

	//DELAY_NUM<<=2;
	lightvalue = guc_Read_a[5];
	//if(lightvalue>100)lightvalue=10;
	XRBoffbrightvalue = lightvalue;

	lowlightDELAY_NUM = guc_Read_a[6];
	if (lowlightDELAY_NUM == 0 || lowlightDELAY_NUM > 255)
		lowlightDELAY_NUM = 1;

	SWITCHfXBR = (guc_Read_a[7]) & 0x01;
	//	addr = guc_Read_a[7];
	//
	//	devgroup = guc_Read_a[8];

	//	addrend = guc_Read_a[9];

	Flash_ReadArr(0X2f80, 1, guc_Read_a1); //读取地址0x2F00所在扇区
	resetbtcnt = guc_Read_a1[0];
}

void XBRHandle(void)
{
	u16 k;

	if (while_1flag == 0)
	{
		//send_data(0x66);
		// 				ADC_TG;
		// 				while(ADC_IF==0){};
		// 				//adc_data = ADC_DATA_RD();
		//
		// 				k = ADC_DH<<8;
		// 				//adc_data =adc_data <<8;
		// 				k+= ADC_DL;
		//
		// 				ADC_INT_IF_CLR; //清中断标志位

		ADCC0 |= 0x40; //启动ADC转换
		while (!(ADCC0 & 0x20))
			;			//等待ADC转换结束
		ADCC0 &= ~0x20; //清除标志位
		k = ADCR;		//获取ADC的值

		times++;

		SUM += k;

		//计算信号值与直流电压偏差值
		if (k > average)
		{
			k -= average;
		}
		else
		{
			k = average - k;
		}
		SUM1 += k;

		if ((times & 0x1ff) == 0) //每256次循环检查一次光敏
		{
			if (LIGHT > 0) //正在伴亮的过程中
			{

				//LIGHT++;

				if (slowchcnt < 100)
				{
					slowchcnt = slowchcnt + 2; //
					if (slowchcnt > 100)
					{
						slowchcnt = 100;
					}
				}
				PWM3init(slowchcnt);
			}
			else if (LIGHT_off == 1) //else if((SWITCHflag2==0)&&(LIGHT_off ==1))
			{
				if (slowchcnt > lightvalue)
				{
					if (slowchcnt >= 2)
						slowchcnt -= 2;
					if (slowchcnt < lightvalue)
						slowchcnt = lightvalue;
				}
				PWM3init(slowchcnt);
			}
		}

		if (times >= 8192) //每250ms迭代及判断一次
		{

			WDTC |= 0x10; //清看门狗

			times = 0;

			calc_average_times++;

			SUM16 += SUM;

			if (calc_average_times >= 8) //每2.5S重新计算一次直流电压值
			{
				calc_average_times = 0;

				SUM16 >>= 16;
				//SUM16/=96000;//102400;
				average += SUM16;
				average /= 2;
				SUM16 = 0;
			}

			if (check_light_times < 8) //2s	读取一次感光AD值，瞬时对比值每2s左右更新一次
			{
				check_light_times++;
			}
			else
			{
				if (LIGHT == 0)
				{
					//light_ad=READ_LIGHT();
					light_ad = read_ad(10); //切换到an10

					if ((light_ad <= (light_ad0 + 2)) && (light_ad0 <= (light_ad + 2)))
						light_ad = light_ad0;

					light_ad0 = light_ad;

					check_light_times = 0;
				}
			}

			if (SUM0 == 0)
			{
				SUM0 = SUM1 + 5000;
				if (start_times == 0 && SUM0 > 1000000)
					SUM0 = 1000000; //设计初值
			}

			if (SUM1_counter == 0)
			{
				SUM10 = SUM1;
				MAX_DELTA = 1; //SUM10>>3;
							   //if(MAX_DELTA<MAX_DELTA0)MAX_DELTA=MAX_DELTA0;
			}

			if ((SUM10 < (SUM1 + MAX_DELTA)) && (SUM1 < (SUM10 + MAX_DELTA))) //???????????
			{
				SUM1_counter++;
				ALL_SUM1 += SUM1;
				SUM10 = ALL_SUM1 / SUM1_counter;
				MAX_DELTA = SUM10 >> 3; //迭代突变(最大偏差值)
				if (MAX_DELTA < MAX_DELTA0)
					MAX_DELTA = MAX_DELTA0;
				if (MAX_DELTA > MAX_DELTA1)
					MAX_DELTA = MAX_DELTA1; //保证最大偏差值在一定范围内

				if (SUM0 > SUM10)
				{
					SUM = SUM0 - SUM10;
					if (SUM > 80000)
						SUM0_num = 6;
					else if (SUM > 40000)
						SUM0_num = 9;
					else
						SUM0_num = 12;
				}
				else
				{
					SUM0_num = 12;
				}

				if ((SUM1_counter >= SUM0_num) && (SUM10 < SUM0))
				{
					if (SUM1_num > 16) //???????????????
					{
						if (SUM0_num <= 9)
							SUM0 = SUM10;
						else if (SUM0 > (SUM10 + 4000))
						{
							SUM0 += SUM10;
							SUM0 /= 2;
						}
						SUM1_counter = 0;
						ALL_SUM1 = 0;
					}
				}

				else if (SUM1_counter >= SUM1_num)
				{

					// 							if(SUM0>SUM10)
					// 							{
					// 								if(SUM1_num>16)SUM0=SUM10;	//???????????????
					// 							}
					// 							else

					if (SUM10 > (SUM0 + 4000))
					{
						SUM = SUM10 - SUM0;

						if ((SUM10 < 8000000) && (SUM < 400000))
						//????????????,???????????????100000?,???????
						{
							if (SUM1_num > 16) //????????????
							{
								SUM0 += SUM10;
								SUM0 /= 2;
							}
							else
							{
								if (SUM > 300000)
									SUM1_num = 16;
								else if (SUM > 150000)
									SUM1_num = 12;
								else
									SUM1_num = 8;
								if (SUM1_counter >= SUM1_num)
								{
									SUM0 += SUM10;
									SUM0 /= 2;
								}
							}
						}
					}
					// 							else if((LIGHT>0)&&(TH==TH_LOW))
					// 							{
					// 								SUM=SUM10-SUM0;
					// 								TH+=SUM;		//?????????????
					// 							}

					if (SUM1_counter >= SUM1_num)
					{
						SUM1_counter = 0;
						ALL_SUM1 = 0;
					}
				}
			}
			else
			{
				SUM1_counter = 0;
				ALL_SUM1 = 0;
			}

			if (stop_times > 0) //
			{
				stop_times--;
				if ((SUM0 > (SUM01 + 6000)) && (SUM1 < (SUM01 + 15000)))
					SUM0 = SUM01 + 6000;
			}
			else
			{

				if (start_times > 0)
				{
					start_times--;

					if (start_times > 0) //???????????,???????250*88ms=22S?????????
					{
						//start_times++;
						/*
							if(SUM0>600000)
							{
								TH=10000;
							}
							else */
						if (SUM0 > 8000000)
						{
							TH = 800000;
						}
						else
						{
							SUM = SUM0 + TH;
							if (SUM > 9000000)
							{
								TH = 9000000 - SUM0;
							}
							//if(TH<30000)TH=30000;
						}
					}
					else
					{
						//TH=TH_LOW;
						//start_times=0;
						//							EA=0;
						//							set_var();
						//							EA=1;
					}
				}

				if (SUM1 > (SUM0 + TH))
				{
					//SUM=SUM1-SUM0;

					//	if(SUM>TH)
					//	{
					if ((light_ad <= LIGHT_TH) || (start_times > 0))
					{
						//								send_data(0xaa);

						//if(alarm_times<2)
						//{
						//	alarm_times++;
						//	  }
						//if(alarm_times>=2)	//??????????????
						{
							if (LIGHT == 0)
								SUM01 = SUM0;
							LIGHT = 1;
							Light_on_flag = 1;
							//PC3=0;
							//LIGHT_ON;
							//slowchcnt = slowchcnt+20;//
							//if(slowchcnt>100)
							//{
							//	slowchcnt = 100;
							//}
							//PWM3init(slowchcnt);
							//P0_6=0;
							//									send_data(0xaa);
							//									send_data((TH/1000)>>8);
							//									send_data((TH/1000)&0xff);
							//									send_data(LIGHT_TH);
							//									send_data(DELAY_NUM>>10);
							//									send_data(DELAY_NUM>>2);		//测试用
							//									send_data(slowchcnt);
							//									send_data(0xaa);
							send_data(0xdd);

							SUM1_num = 8;
							LIGHT_off = 0;
							light1scount = 0;
							light1sflag = 0;
							//								send_data(0xaa);
						}
					}
				}
			}

			//SUM2 = SUM1;
			
			//send_data(average >> 4);
			//send_data(light_ad);
			//send_data(SUM0 >> 16);
			//send_data(SUM0 >> 8);
			//send_data(SUM2 >> 16);
			//send_data(SUM2 >> 8); //20200927	测试用

			SUM = 0;
			SUM1 = 0;

			if (LIGHT > 0) //????
			{

				//LIGHT++;

				// 					slowchcnt = slowchcnt+5;//
				// 					if(slowchcnt>=100)
				// 					{
				// 						slowchcnt = 100;
				// 					}
				// 					PWM3init(slowchcnt);

				if (LIGHT > DELAY_NUM)
				{
					LIGHT = 0;
					while_1flag = 1;
					Light_on_flag = 0;
					Light_on_flagpre = 0;
					//while_2flag = 0;
					//break;
				}
			}
		}
	}
	else
	{
		LIGHT_off = 1;
		while_1flag = 0;
		//if(while_2flag==0)
		//{
		//	while_2flag = 1;
		//					send_data(0x55);
		//}
		//PC3=1;
		//LIGHT_OFF;
		//slowchcnt = lightvalue;
		//PWM3init(lightvalue);
		lowlight1mincount = 0;
		lowlight1minflag = 0;

		//P0_6=1;
		//send_data(0x55);
		Delay_ms(250);

		// 		SUM=0;
		// 		SUM1=0;
		// 		times=0;

		SUM16 = 0;
		calc_average_times = 0;
		SUM1_num = 64;

		stop_times = 2;
		//if(start_times==0)TH=TH_LOW;
		check_light_times = 6;

		SUM1_counter = 0;
		ALL_SUM1 = 0;

		//		send_data(0xdd);
		//		send_data(0xdd);
	}
}

void wait1(void)
{
	u8 i, j;

	//等待直流电压稳定
	j = 0;
	while (1)
	{
		SUM = 0;

		// 	  for(i=0;i<4;i++)	//0.52s
		// 	  {
		// 	  	for(t=0;t<8192;t++)	//0.13s
		// 	  	{

		// 				ADCC0 |= 0x40;					//启动ADC转换
		// 				while(!(ADCC0&0x20));		//等待ADC转换结束
		// 				ADCC0 &=~ 0x20;					//清除标志位
		// 				k = ADCR;				//获取ADC的值
		//
		// 				SUM+=k;
		//
		// 	  	}
		//
		// 			WDTC |= 0x10;		//清看门狗

		for (i = 0; i < 128; i++) //
		{

			ADCC0 |= 0x40; //启动ADC转换
			while (!(ADCC0 & 0x20))
				;			//等待ADC转换结束
			ADCC0 &= ~0x20; //清除标志位
			//k = ADCR;				//获取ADC的值

			SUM += ADCR;
		}

		//发送感光AD值
		// 				send_byte=0xFA;
		// 				check_sum=send_byte;
		// 				send_data(send_byte);

		// 				//send_byte=light_ad;
		// 				check_sum+=light_ad;
		// 				send_data(light_ad);

		// 				check_sum+=light_ad;
		// 				send_data(light_ad);
		//
		// 				check_sum+=1;
		// 				send_data(check_sum);

		//}

		Delay_ms(400);

		//WDTC |= 0x10;		//清看门狗

		i = SUM >> 11;
		if ((i > 12) && (i < 141) && (j > 20))
			break;

		j++;

		if (j > 80)
			break; //??35????????????1.1V???????
	}
}
void wait2(void)
{
	u8 i;
	//u8 j;
	u16 k, t;

	SUM = 0;

	for (i = 0; i < 8; i++)
	{
		for (t = 0; t < 8192; t++)
		{
			// 				ADC_TG;
			// 				while(ADC_IF==0){};
			// 				//adc_data = ADC_DATA_RD();
			//
			// 				k = ADC_DH<<8;
			// 				//adc_data =adc_data <<8;
			// 				k+= ADC_DL;
			//
			// 				ADC_INT_IF_CLR; //清中断标志位

			ADCC0 |= 0x40; //启动ADC转换
			while (!(ADCC0 & 0x20))
				;			//等待ADC转换结束
			ADCC0 &= ~0x20; //清除标志位
			k = ADCR;		//获取ADC的值

			SUM += k;
		}
		WDTC |= 0x10; //清看门狗
	}

	average = SUM >> 16;

	// 	light_ad=read_ad(10);
	// 	light_ad0=light_ad;

	// 	Delay_ms(4);	//4ms
}

unsigned char PWM3init(unsigned char ab)
{
	float i11;
	unsigned char j11;
	i11 = ab * 255 / 100;
	j11 = i11;
#ifdef V11
	/************************************PWM3初始化****************************************/
	//P0M3 = P0M3&0xF0|0x08;		//P06设置为推挽输出
	PWM3_MAP = 0x05; //PWM3映射P05口

#endif

#ifdef V10
	PWM3_MAP = 0x06; //PWM3映射P05口

#endif

#ifdef V12
	PWM3_MAP = 0x10; //PWM3映射P10口

#endif

	//周期计算 	= 0xFF / (Fosc / PWM分频系数)		（Fosc见系统时钟配置的部分）
	//			= 0xFF /(16000000 / 4)
	// 			= 255 /4000000
	//			= 63.75us		即15.69KHZ

	PWM3P = 0xFF; //PWM周期为0xFF
	//有效电平时间计算（即占空比）
	//			= 0x55 / (Fosc / PWM分频系数)		（Fosc见系统时钟配置的部分）
	//			= 0x55 /(16000000 / 4)
	// 			= 85 /4000000
	//			= 21.25us		占空比为 21.25 / 63.75 = 34%

	PWM3D = j11;  //PWM占空比设置
	PWM3C = 0x94; //使能PWM3，关闭中断，允许输出，时钟16分频

	return 0;
}

void reset_bt_module(void);

/***************************************************************************************
  * @说明  	主函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/
void main()
{
	u8 i;
	bt_protocol_init(); //mcu_sdk
	InitSYS();
	GPIO_Init();
	//LIGHT_ON;
	//P0_6=0;
	PWM3init(100);
	Timer_Init();
	UART1_Init();
	ADC_Init();

	LVDC = 0xAA; //LVD设置2.4V,禁止中断
	//	消抖时间 = 	(0xFF + 2) * 1/Fcpu
	//			 =	(0xFF + 2) / 16000000	（当前的CPU时钟）
	//			 =	16.0625us
	LVDDBC = 0xFF; //设置消抖时间
	LVDC &= ~0x08; //清除LVD中断标志位
				   //
	EA = 1;

	Delay_ms(200);

	//LIGHT_ON;
	PWM3init(100);
	SWITCHflag = 1;
	light_ad = read_ad(10);
	light_ad0 = light_ad;

	EA = 0;
	set_var(); //从flash读取出变量

	//for(i=0;i<5;i++)send_data(guc_Read_a[i]);

	// 	TXdata[0]=VERSION;
	// 	t=TH/1000;
	// 	TXdata[1]=t>>8;
	// 	TXdata[2]=t;
	// 	TXdata[3]=LIGHT_TH;
	// 	t=DELAY_NUM/4;
	// 	TXdata[4]=t>>8;
	// 	TXdata[5]=t;

	// 	TXdata[6]=light_ad;

	// 	for(i=0;i<7;i++)send_data(TXdata[i]);
	resetbtcnt++;

	Flash_EraseBlock(0x2F80);
	Delay_us_1(10000);
	FLASH_WriteData(resetbtcnt, 0x2F80);
	Delay_us_1(100);

	EA = 1;

	wait1();

	slowchcnt = lightvalue;
	//Delay_ms(200);
	PWM3init(lightvalue);
	//LIGHT_OFF;
	//P0_6=1;
	Delay_ms(300);

	wait2();

	SUM = 0;
	while (1)
	{
		if (resetbtcnt >= 3)	//行为是每三次上电会复位一次蓝牙模块
		{
			resetbtcnt = 0;
			reset_bt_module();
		}

		if (check_group_count <= 2) //一上电间隔一秒获取3次群组地址
		{
			if (check_group_flag == 1)
			{
				check_group_flag = 0;
				check_group_count++;

				send_data(0x55);
				send_data(0xAA);
				send_data(0X00);
				send_data(0XB4); //新的命令字，功能不明
				send_data(0X00);
				send_data(0X00);
				send_data(0Xb3);
			}
		}
		WDTC |= 0x10; //清看门狗

		if (while_1flag == 0)
		{
			if ((times & 0x1f) == 0)
				bt_uart_service();
		}

		if (SWITCHfXBR == 1) //雷达开, app控制
		{
			if (while_2flag == 0)
			{
				while_1flag = 0;

				while_2flag = 1;
				slowchcnt = lightvalue;

				SUM16 = 0;
				calc_average_times = 0;
				SUM1_num = 64;

				stop_times = 2;
				//if(start_times==0)TH=TH_LOW;
				check_light_times = 6;

				SUM1_counter = 0;
				ALL_SUM1 = 0;
			}

			XBRHandle();

			if (LIGHT_off > 0) //关灯延时
			{
				if (lowlight1minflag == 1)
				{
					lowlight1minflag = 0;
					LIGHT_off++;
					if (LIGHT_off >= lowlightDELAY_NUM)
					{
						LIGHT_off = 0;
						PWM3init(0);
					}
				}
			}
			if (LIGHT > 0) //亮灯延时
			{
				if (light1sflag == 1)
				{
					light1sflag = 0;
					LIGHT++;
					//slowchcnt = slowchcnt+20;//
					//					if(slowchcnt>100)
					//					{
					//						slowchcnt = 100;
					//					}
					//					PWM3init(slowchcnt);
				}
			}

			//联动
			if (Linkage_flag == 1)
			{
				if (Light_on_flagpre != Light_on_flag)
				{
					Light_on_flagpre = Light_on_flag;
					LIGHT = 1;
					//PWM3init(100);
					for (i = 0; i < 8; i++)
					{
						if (groupaddr[i] != 0)
						{
							mcu_dp_bool_mesh_update(DPID_SWITCH_LED2, SWITCHflag2, groupaddr[i]);
						}
					}
				}
			}
		}
		else
		{ //雷达关
			while_2flag = 0;
			if (SWITCHflag2 == 0) //关灯
			{
				PWM3init(0);
			}
			else
			{ //开灯
				PWM3init(XRBoffbrightvalue);

				while_1flag = 0;

				slowchcnt = lightvalue;
				//PWM3init(lightvalue);

				SUM16 = 0;
				calc_average_times = 0;
				SUM1_num = 64;

				stop_times = 2;
				//if(start_times==0)TH=TH_LOW;
				check_light_times = 6;

				SUM1_counter = 0;
				ALL_SUM1 = 0;
			}
		}
	}

	// 	while(1)
	// 	{
	// 		System_Time_Cnt();
	// 		System_Task();
	// 		WDTC |= 0x10;              //清狗
	// 	}
}

/***************************************************************************************
  * @说明  	T1中断服务函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/
void TIMER1_Rpt(void) interrupt TIMER1_VECTOR
{
	//Timer1_FLAG = 1;		//1mS
	Timer_Counter++;

	lowlight1mincount++;
	if (lowlight1mincount >= 60000)
	{
		lowlight1mincount = 0;
		lowlight1minflag = 1;
	}
	light1scount++;
	if (light1scount >= 1000)
	{
		check_group_flag = 1;
		light1scount = 0;
		light1sflag = 1;
	}
}

/***************************************************************************************
  * @说明  	UART1中断服务函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/

void UART1_Rpt(void) interrupt UART1_VECTOR
{
	u8 i;
	//u16 t;

	if (SCON & 0x01) //判断接收中断标志位
	{
		i = SBUF;
		uart_receive_input(i); //mcu_sdk
		SCON &= ~0x01;		   //清除接收中断标志位
		EA = 1;
	}
}

void UART2_Rpt(void) interrupt UART2_VECTOR
{
}

// void Delay_2us(u16 Cnt)
// {
// 	while(Cnt--);
// }

void Flash_EraseBlock(unsigned int fui_Address)
{
	while (1)
	{
		LVDC &= ~0x08; //清除LVD中断标志位
		P0_0 = 0;
		if ((LVDC & 0x08) == 0)
			break;
	}
	P0_0 = 1;
	EA = 0;
	IAP_CMD = 0xF00F;		//Flash解锁
	IAP_ADDR = fui_Address; //写入擦除地址
	IAP_CMD = 0xD22D;		//选择操作方式， 扇区擦除
	IAP_CMD = 0xE11E;		//触发后IAP_ADDRL&IAP_ADDRH指向0xFF，同时自动锁定
							//EA=1;
}

/**
  * @说明  	写入一个字节数据到Flash里面
  *         该函数需绝对地址编译，详情请查阅IAP操作应用手册
  * @参数  	fui_Address ：FLASH地址
  *	@参数	  fucp_SaveData：写入的数据
  * @返回值 无
  * @注		  写之前必须先对操作的扇区进行擦除
  */
void FLASH_WriteData(unsigned char fuc_SaveData, unsigned int fui_Address)
{
	while (1)
	{
		LVDC &= ~0x08; //清除LVD中断标志位
		P0_0 = 0;
		if ((LVDC & 0x08) == 0)
			break;
	}
	P0_0 = 1;
	EA = 0;
	IAP_DATA = fuc_SaveData;
	IAP_CMD = 0xF00F; //Flash解锁
	IAP_ADDR = fui_Address;
	IAP_CMD = 0xB44B; //字节编程
	IAP_CMD = 0xE11E; //触发一次操作
					  //EA=1;
}

/**
  * @说明  	写入任意长度的数据到FLASH里面
  *         该函数需绝对地址编译，详情请查阅IAP操作应用手册
  * @参数  	fui_Address ：FLASH起始地址
  *	@参数	  fuc_Length ： 写入数据长度
  *			    取值范围：0x00-0xFF
  *	@参数  *fucp_SaveArr：写入的数据存放区域的首地址
  * @返回值 无
  * @注		  写之前必须先对操作的扇区进行擦除
  */

// void Flash_WriteArr(unsigned int fui_Address,unsigned char fuc_Length,unsigned char *fucp_SaveArr)
// {
// 	unsigned char fui_i = 0;
// 	EA=0;
// 	for(fui_i=0;fui_i<fuc_Length;fui_i++)
// 	{
// 		FLASH_WriteData(*(fucp_SaveArr++), fui_Address++);
// 	}
// 	EA=1;
// }

/**
  * @说明  	从FLASH里面读取任意长度的数据
  * @参数  	fui_Address ：FLASH起始地址
  *	@参数	  fuc_Length ：读取数据长度
  *			    取值范围：0x00-0xFF
  *	@参数	 *fucp_SaveArr：读取数据存放的区域首地址
  * @返回值 无
  * @注		  无
  */
