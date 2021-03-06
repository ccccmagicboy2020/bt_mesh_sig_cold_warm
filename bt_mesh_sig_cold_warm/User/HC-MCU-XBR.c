#define ALLOCATE_EXTERN
#include "HC89S003F4.h"
#include "bluetooth.h"

#define V12 //硬件板卡的版本

#define USER_PARAMETER_START_SECTOR_ADDRESS0 0x2F00
#define USER_PARAMETER_START_SECTOR_ADDRESS1 0x2F80
#define USER_PARAMETER_START_SECTOR_SIZE	14

#define TH_LOW 30000
#define TH_HIGH 4000000

#define TH_DEF 40000

//允许噪声值偏差范围
#define MAX_DELTA0 20000 //最大偏差低值
#define MAX_DELTA1 60000 //最大偏差高值

#define MAX_DELAY 1800
//最大延时秒数
#define MAX_BT_DUTY 3000

//感光门限-30对应8LUX左右的AD值,设置为255表示不检测感光
#define LIGHT_TH0 255

volatile ulong Timer_Counter = 0;

u8 xdata SUM1_counter = 0; //偏差平均值的计算计数器
u8 xdata SUM0_num = 12;	   //SUM0迭代次数
u8 xdata SUM1_num = 64;	   //SUM1迭代次数
ulong xdata SUM01;		   //上一把的SUM0的值
ulong xdata SUM2;		   //调试用snapshot of the SUM1 value
ulong xdata SUM10 = 0;	   //SUM1值的几次平均值，时间上的滞后值
ulong xdata SUM0 = 0;	   //SUM10的平均值
ulong xdata SUM1 = 0;	   //平均绝对离差的累加合的瞬时值
ulong xdata ALL_SUM1 = 0;  //SUM1的累加值
ulong xdata SUM16 = 0;	   //2^16次的累计值变量
ulong xdata SUM = 0;	   //an1的raw累加值
u16 xdata start_times = 1; //???
u16 xdata times = 0;	   //主循环次数
ulong xdata TH;			   //设置误差阈值，可由APP设置的感应强度转换
ulong xdata MAX_DELTA; //最大偏差值
u8 xdata alarm_times = 0;
u8 xdata stop_times = 0; //跳过的节拍数

uint xdata LIGHT = 0;	  //伴亮灯秒的计数器
uint xdata LIGHT_off = 0; //无人灭灯的分钟计数器
uint xdata average;		  //an1的raw平均值

u8 xdata light_ad;	//光敏实时值raw
u8 xdata light_ad0; //光敏初始瞬时值raw

u8 xdata check_light_times = 8;	 //用于光敏检查的计数器
u8 xdata calc_average_times = 0; //用于计算平均值的计数器
u8 xdata LIGHT_TH;
u16 xdata DELAY_NUM;
u8 xdata lowlightDELAY_NUM;
u8 while_1flag = 0;		  //伴亮标志 1==伴亮状态 0==侦测状态
u8 while_2flag = 0;		  //???

u8 xdata SWITCHflag2 = 0; //灯开关的变量，可由APP设置
u8 xdata SWITCHfXBR = 1;  //雷达感应开关的变量，可由APP设置
u8 xdata lightvalue = 10; //亮度值，可由APP设置
u8 xdata slowchcnt = 10;				  //亮度渐变目标值
u8 xdata resetbtcnt = 0;				  //为重置蓝牙模块设置的计数器
u8 xdata XRBoffbrightvalue = 0;			  //当关闭雷达时，APP设置的亮度值(微亮值)
volatile u16 xdata lowlight1mincount = 0; //timer的计数器1ms自加
volatile u8 xdata lowlight1minflag = 0;	  //timer的分钟标志
volatile u16 idata light1scount = 0;	  //timer的计数器1ms自加
volatile u16 idata light1sflag = 0;		  //timer的秒标志

u16 idata groupaddr[8] = {0};
u8 idata Linkage_flag = 0;		//联动标志
u8 xdata temper_value = 0;			//冷暖值

u8 idata bt_join_cnt = 0;
u8 xdata all_day_micro_light_enable = 0;

u16 xdata radar_trig_times = 0;
u16 xdata radar_trig_times_last = 0;

u8 xdata light_status_xxx = LIGHT_STATUS_EMPTY;
u8 xdata light_status_xxx_last = LIGHT_STATUS_EMPTY;

volatile u16 xdata radar_number_count = 0;
volatile u8 xdata radar_number_send_flag = 0;
u8 idata radar_number_send_flag2 = 0;
u8 radar_number_send_flag3 = 0;

u8 xdata person_in_range_flag = PERSON_STATUS_NO_PERSON;
u8 xdata person_in_range_flag_last = PERSON_STATUS_NO_PERSON;

u8 idata ab_last = 0;
u8 idata Exit_network_controlflag = 0;
volatile u16 idata Exit_network_controlflag_toggle_counter = 0;

u16 idata bt_and_sigmesh_duty = 1000;	// unit:ms
u8 find_me_flag = 0;
volatile u8 xdata find_me_counter = 0;

volatile u16 idata person_meter = 0;
u16 idata person_meter_last = 0;

unsigned char upload_disable = 0;

unsigned char PWM0init(unsigned char ab);
unsigned char PWM3init_xxx(unsigned char ab);
unsigned char PWM3init(unsigned char ab);
void Flash_EraseBlock(unsigned int fui_Address); //扇区擦除
void FLASH_WriteData(unsigned char fuc_SaveData, unsigned int fui_Address);
void Flash_ReadArr(unsigned int fui_Address, unsigned char fuc_Length, unsigned char *fucp_SaveArr); //读取任意长度数据
void savevar(void);
void reset_bt_module(void);

unsigned char xdata guc_Read_a[USER_PARAMETER_START_SECTOR_SIZE] = {0x00}; //用于存放读取的数据
unsigned char xdata guc_Read_a1[2] = {0x00}; //用于存放读取的数据

void Flash_ReadArr(unsigned int fui_Address, unsigned char fuc_Length, unsigned char *fucp_SaveArr)
{
	while (fuc_Length--)
		*(fucp_SaveArr++) = *((unsigned char code *)(fui_Address++)); //读取数据
}

void Delay_ms(uint t)
{
	Timer_Counter = 0;
	while (Timer_Counter < t)
	{
		WDTC |= 0x10; //清看门狗
	}
}

void Delay_us(uint q1)
{
	uint j;
	for (j = 0; j < q1; j++)
	{
		;
	}
}

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
	TL4 = 0x98;	  //波特率9600
	
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

#ifdef XBR403_03_2
	ADCC1 = 0x00;  //选择外部通道0
#endif
	
#ifdef XBR403
	ADCC1 = 0x02;  //选择外部通道2
#endif

#ifdef V12
	ADCC1 = 0x01;  //选择外部通道1
#endif
	
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

	P1M0 = P1M0 & 0xFF | 0x88; //P10设置为推挽输出
							   //P11设置为推挽输出
							   
							   

	P0M0 = P0M0 & 0x0F | 0x30; //P01设置为模拟输入

	P2M1 = P2M1 & 0xF0 | 0x03; //P22设置为模拟输入

	//P0M3 = P0M3&0x0F|0x30;				  //P07设置为模拟输入
	//	P0M0 = P0M0&0x0F|0x80;		      //P01设置为推挽输出

	//P0M2 = P0M2&0x0F|0x80;		      //P05设置为推挽输出

	//P0M3 = P0M3&0xF0|0x03;			//P06设置为模拟输入  //|0x08;		      //P06设置为推挽输出

	//	P0M3 = P0M3&0x0F|0x20;				  //P07设置为上拉输入

#endif

#ifdef XBR403
	//PWM & ADC
	P1M0 = P1M0 & 0xF0 | 0x08; //P10设置为推挽输出
	P0M0 = P0M0 & 0xFF | 0x88; //P00设置为推挽输出
							   //P01设置为推挽输出
	P0M1 = P0M1 & 0xFF | 0x83; //P03设置为推挽输出
							   //P02设置为模拟输入
	P0M3 = P0M3 & 0xF0 | 0x08; //P06设置为推挽输出
	P2M1 = P2M1 & 0xF0 | 0x03; //P22设置为模拟输入
#endif

#ifdef XBR403_03_2
	 //PWM & ADC
	 P1M0 = P1M0 & 0x0F | 0x80; //P11  r
	 P0M0 = P0M0 & 0xF0 | 0x03; //P00  if adc an0
	 P0M0 = P0M0 & 0x0F | 0x80; //P01  ww
	 P0M1 = P0M1 & 0xF0 | 0x08; //P02  g
	 P0M1 = P0M1 & 0x0F | 0x80; //P03  b
	 P0M3 = P0M3 & 0x0F | 0x30; //P07  light adc an7
	 P2M3 = P2M3 & 0x0F | 0x80; //P27  cw
#endif

}

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

	ADCC1 = ch;	   //选择外部通道
	ADCC0 |= 0x40; //启动ADC转换
	while (!(ADCC0 & 0x20))
		;			//等待ADC转换结束
	ADCC0 &= ~0x20; //清除标志位

	Delay_us(100);

	ad_sum = 0;

	for (i = 0; i < 16; i++)
	{
		ADCC0 |= 0x40; //启动ADC转换
		while (!(ADCC0 & 0x20))
			;			//等待ADC转换结束
		ADCC0 &= ~0x20; //清除标志位
		ad_sum += ADCR; //获取ADC的值

		Delay_us(20);
	}

#ifdef XBR403_03_2
	ADCC1 = 0x00;  //选择外部通道0
#endif

#ifdef XBR403
	ADCC1 = 0x02;  //选择外部通道2
#endif

#ifdef V12
	ADCC1 = 0x01;  //选择外部通道1
#endif
	
	i = ad_sum >> 8;

	Delay_us(100);
	return (i);
}

void set_var(void)
{

	Flash_ReadArr(USER_PARAMETER_START_SECTOR_ADDRESS0, USER_PARAMETER_START_SECTOR_SIZE, guc_Read_a); //读取地址所在扇区

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

	lightvalue = guc_Read_a[5];
	XRBoffbrightvalue = lightvalue;

	lowlightDELAY_NUM = guc_Read_a[6];
	if (lowlightDELAY_NUM == 0 || lowlightDELAY_NUM > 255)
		lowlightDELAY_NUM = 1;

	SWITCHfXBR = (~guc_Read_a[7]) & 0x01;
	
	Linkage_flag = (guc_Read_a[8]) & 0x01;
	
	SWITCHflag2 = (guc_Read_a[9]) & 0x01;
	
	all_day_micro_light_enable = (guc_Read_a[10]) & 0x01;
	
	temper_value = guc_Read_a[11];

	bt_and_sigmesh_duty = guc_Read_a[12];
	bt_and_sigmesh_duty <<= 8;
	bt_and_sigmesh_duty += guc_Read_a[13];
	if (bt_and_sigmesh_duty == 0 || bt_and_sigmesh_duty > MAX_BT_DUTY)
		bt_and_sigmesh_duty = 1000;	
	//
	Flash_ReadArr(USER_PARAMETER_START_SECTOR_ADDRESS1, 2, guc_Read_a1); //
	resetbtcnt = guc_Read_a1[0];
	bt_join_cnt = guc_Read_a1[1];
	Flash_EraseBlock(USER_PARAMETER_START_SECTOR_ADDRESS1);	
	Delay_us(10000);
	
	resetbtcnt++;
	
	FLASH_WriteData(resetbtcnt, USER_PARAMETER_START_SECTOR_ADDRESS1 + 0);
	Delay_us(100);	

	if (0 == bt_join_cnt)
	{
		reset_bt_module();
	}
	else if (1 == bt_join_cnt)
	{
		FLASH_WriteData(bt_join_cnt, USER_PARAMETER_START_SECTOR_ADDRESS1 + 1);
		Delay_us(100);		
	}
}

void XBRHandle(void)
{
	u16 k;

	if (while_1flag == 0)//侦测状态
	{
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

		if ((times & 0x1ff) == 0) //每256次循环检查一次状态
		{
			if (LIGHT > 0) //正在伴亮的过程中时变亮
			{
				if (slowchcnt < 100)
				{
					slowchcnt = slowchcnt + 2;
					if (slowchcnt > 100)
					{
						slowchcnt = 100;
					}
				}
				PWM3init(slowchcnt);
			}
			else if (LIGHT_off == 1)//灭灯计时开始时变暗
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
				average /= 2;	//得出平均值
				SUM16 = 0;
			}

			if (check_light_times < 8) //2s	读取一次感光AD值，瞬时对比值每2s左右更新一次
			{
				check_light_times++;
			}
			else
			{
				if (LIGHT == 0)	//伴亮未开始，也就是未检测到雷达目标
				{
					
					#ifdef XBR403_03_2
						light_ad = read_ad(7); //切换到an7
					#endif

					#ifdef XBR403
						light_ad = read_ad(10); //切换到an10
					#endif

					#ifdef V12
						light_ad = read_ad(10); //切换到an10
					#endif

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
					if (SUM10 > (SUM0 + 4000))
					{
						SUM = SUM10 - SUM0;

						if ((SUM10 < 8000000) && (SUM < 1200000))
						{
							if (SUM1_num > 16)
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

					if (start_times > 0)
					{
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
						}
					}
					else
					{
						//
					}
				}

				if (SUM1 > (SUM0 + TH))	//正式判断
				{
					if ((light_ad <= LIGHT_TH) || (start_times > 0))	//在一定亮度之下才运行
					{
						{
							if (LIGHT == 0)
								SUM01 = SUM0;
							LIGHT = 1;
							/////////////////////////////////////////////
							radar_trig_times++;
							person_in_range_flag = PERSON_STATUS_HAVE_PERSON;
							radar_number_send_flag2 = 1; //for bt upload
							radar_number_send_flag3 = 1; //for mesh upload
							/////////////////////////////////////////////
							SUM1_num = 8;
							LIGHT_off = 0;
							light1scount = 0;
							light1sflag = 0;
						}
					}
				}
			}

			SUM2 = SUM1;
			
///////////////////////////////////////////////////
//			send_data(average >> 4);
//			send_data(light_ad);
//			send_data(SUM0 >> 16);
//			send_data(SUM0 >> 8);
//			send_data(SUM1 >> 16);
//			send_data(SUM1 >> 8); //20200927	测试用
//			send_data(TH >> 16);
//			send_data(TH >> 8);
////////////////////////////////////////////////////

			SUM = 0;
			SUM1 = 0;

			if (LIGHT > 0)
			{
				if (LIGHT > DELAY_NUM)
				{
					LIGHT = 0;
					while_1flag = 1;
					person_in_range_flag = PERSON_STATUS_NO_PERSON;
				}
			}
		}
	}
	else
	{
		LIGHT_off = 1;
		while_1flag = 0;
		lowlight1mincount = 0;
		lowlight1minflag = 0;

		//////////////////////////
		//Delay_ms(250);//防干扰
		//////////////////////////
		
		SUM16 = 0;
		calc_average_times = 0;
		SUM1_num = 64;

		stop_times = 2;
		check_light_times = 6;

		SUM1_counter = 0;
		ALL_SUM1 = 0;
	}
}

//积累一定的值
void wait1(void)
{
	u8 i, j;

	//等待直流电压稳定
	j = 0;
	while (1)
	{
		SUM = 0;

		for (i = 0; i < 128; i++) //
		{

			ADCC0 |= 0x40; //启动ADC转换
			while (!(ADCC0 & 0x20))
				;			//等待ADC转换结束
			ADCC0 &= ~0x20; //清除标志位
			//k = ADCR;				//获取ADC的值

			SUM += ADCR;
		}

		Delay_ms(400);

		i = SUM >> 11;
		if ((i > 12) && (i < 141) && (j > 20))
			break;

		j++;

		if (j > 80)
			break; //
	}
}

//积累一个2^16次的平均值
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
}
unsigned char PWM0init(unsigned char ab)
{
	float i11;
	u16 j11;
	
	if (1 == ab)
	{
		j11 = 0;
	}
	else
	{
		i11 = ab * 511 / 100;
		j11 = (u16)(i11 + 0.5);
	}

#ifdef XBR403_03_2
	PWM0_MAP = 0x27;					//PWM0通道映射P27口
#endif
	
#ifdef XBR403
	PWM0_MAP = 0x11;					//PWM0通道映射P11口
#endif

#ifdef V12
	PWM0_MAP = 0x11;					//PWM0通道映射P11口
#endif
	
	PWM0C = 0x01;					  	//PWM0高有效，PWM01高有效，时钟8分频 
	
	//独立模式下，PWM0和PWM01共用一个周期寄存器
	//PWM0的占空比调节使用			PWM0组的占空比寄存器
	//PWM01的占空比调节使用			PWM0组的死区寄存器

	//周期计算 	= 0x03ff / (Fosc / PWM分频系数)		（Fosc见系统时钟配置的部分）
	//			= 0x03ff / (16000000 / 8)			
	// 			= 1023   /2000000
	//			= 511.5us		   		约1.955kHz

	PWM0PH = 0x01;						//周期高4位设置为0x03
	PWM0PL = 0xFF;						//周期低8位设置为0xFF

	//占空比计算= 0x0155 / (Fosc / PWM分频系数)		（Fosc见系统时钟配置的部分）
	//			= 0x0155 / (16000000 / 8)			
	// 			= 341 	 / 2000000
	//			= 170.5us		   占空比为 170.5/511.5 = 33.3%

	PWM0DH = (u8)(j11>>8);				//PWM0高4位占空比0x01
	PWM0DL = (u8)j11;					//PWM0低8位占空比0x55

	PWM0EN = 0x0F;						//使能PWM0，工作于独立模式	
	return 0;
}
unsigned char PWM3init_xxx(unsigned char ab)
{
	float i11;
	unsigned char j11;
	
	if (1 == ab)
	{
		j11 = 0;
	}
	else
	{
		i11 = ab * 255 / 100;
		j11 = (unsigned char )(i11 + 0.5);
	}
	
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

#ifdef XBR403_03_2
	PWM3_MAP = 0x01;					//PWM3通道映射P01口
#endif

#ifdef XBR403
	PWM3_MAP = 0x10; //PWM3映射P10口
#endif


	//周期计算 	= 0xFF / (Fosc / PWM分频系数)		（Fosc见系统时钟配置的部分）
	//			= 0xFF /(16000000 / 4)
	// 			= 255 /4000000
	//			= 63.75us		即15.69KHZ

	PWM3P = 0xFF; //周期寄存器//PWM周期为0xFF
	//有效电平时间计算（即占空比）
	//			= 0x55 / (Fosc / PWM分频系数)		（Fosc见系统时钟配置的部分）
	//			= 0x55 /(16000000 / 4)
	// 			= 85 /4000000
	//			= 21.25us		占空比为 21.25 / 63.75 = 34%

	PWM3D = j11;  //PWM占空比设置，占空比寄存器p90
	PWM3C = 0x94; //PWM控制寄存器，使能PWM3，关闭中断，允许输出，时钟16分频

	return 0;
}
unsigned char PWM3init(unsigned char ab)
{
	u8 aa;
	u8 bb;
	
	if (ab_last == ab)
	{
		return 0;
	}
	else
	{
		ab_last = ab;
	}
	
	if (0 == ab)
	{
		light_status_xxx = LIGHT_STATUS_EMPTY;
	}
	else if (100 == ab)
	{
		light_status_xxx = LIGHT_STATUS_FULL;
	}
	else
	{
		light_status_xxx = LIGHT_STATUS_MICRO;
	}
	
	aa = (u8)(temper_value*ab/100 + 0.5);
	
	bb = ab - aa;
	PWM0init(bb);//冷
	PWM3init_xxx(aa);//暖
	
	return 0;
}

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
	
	#ifdef XBR403_03_2
		light_ad = read_ad(7); //切换到an7
	#endif
	
	#ifdef XBR403
		light_ad = read_ad(10); //切换到an10
	#endif
	
	#ifdef V12
		light_ad = read_ad(10); //切换到an10
	#endif
	
	light_ad0 = light_ad;

	EA = 0;
	set_var(); //从flash读取出变量
	
	PWM3init(100);

	EA = 1;

	wait1();

	slowchcnt = lightvalue;

	PWM3init(lightvalue);

	Delay_ms(300);

	wait2();

	SUM = 0;
	
	upload_disable = 0;	
	
	if (resetbtcnt > 3)
	{
		resetbtcnt = 0;
		reset_bt_module();
	}
	else
	{
		savevar();
	}
		
	while (1)
	{
		//find me
		if (find_me_flag)
		{
			PWM3init(100);
			Delay_ms(100);
			PWM3init(0);
			Delay_ms(100);
			find_me_counter++;
			if (3 <= find_me_counter)
			{
				find_me_flag = 0;
				find_me_counter = 0;
			}
		}
		//配网闪灯
		if (Exit_network_controlflag)
		{
			PWM3init(100);
			Delay_ms(100);
			PWM3init(0);
			Delay_ms(100);
			Exit_network_controlflag_toggle_counter++;
			if (300 <= Exit_network_controlflag_toggle_counter)	//1 min toggle led
			{
				Exit_network_controlflag = 0;
			}
		}
		//上报通信
		if (upload_disable == 0)
		{
			//灯状态更新
			if (light_status_xxx != light_status_xxx_last)
			{
				mcu_dp_enum_update(DPID_LIGHT_STATUS,light_status_xxx);
				light_status_xxx_last = light_status_xxx;
			}
			//人状态更新
			if (person_in_range_flag != person_in_range_flag_last)
			{
				mcu_dp_enum_update(DPID_PERSON_IN_RANGE,person_in_range_flag);
				person_in_range_flag_last = person_in_range_flag;
				
				if (person_in_range_flag == PERSON_STATUS_HAVE_PERSON)
				{
					person_meter++;				
				}
			}
			//人表状态更新
			if (person_meter != person_meter_last)
			{
				mcu_dp_value_update(DPID_PERSON_METER,person_meter);
				person_meter_last = person_meter;
			}		
			
			//周期控制
			if (1 == radar_number_send_flag)
			{
				radar_number_send_flag = 0;
				//雷达有触发
				if (1 == radar_number_send_flag2)
				{
					radar_number_send_flag2 = 0;
					//雷达计数更新及if_sum更新
					if (radar_trig_times_last != radar_trig_times)
					{
						//雷达计数更新
						mcu_dp_value_update(DPID_RADAR_TRIGGER_TIMES,radar_trig_times);
						radar_trig_times_last = radar_trig_times;
						//中频更新
						mcu_dp_value_update(DPID_IF_SUM, SUM2);
					}			
				}
			}
		}

		if (1 == radar_number_send_flag3)
		{
			radar_number_send_flag3 = 0;
			//联动开启的话
			if (Linkage_flag == 1)
			{
				for (i=0;i<8;i++)
				{
					//公共群组具有群号
					if (groupaddr[i] != 0)
					{
						mcu_dp_enum_mesh_update(DPID_PERSON_IN_RANGE_EX, 0, groupaddr[i]);
					}							
				}
			}				
		}

		WDTC |= 0x10; //清看门狗

		bt_uart_service();	//串口解包异步处理

		if (SWITCHfXBR == 1) //雷达开控制
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
				
				check_light_times = 6;

				SUM1_counter = 0;
				ALL_SUM1 = 0;
			}

			XBRHandle();

			if (LIGHT_off > 0) //灭灯延时，单位分钟
			{
				if (lowlight1minflag == 1)
				{
					lowlight1minflag = 0;
					LIGHT_off++;
					if (LIGHT_off >= lowlightDELAY_NUM)
					{
						LIGHT_off = 0;
						if (1 == all_day_micro_light_enable)
						{
							//
						}
						else
						{
							PWM3init(0);
						}
					}
				}
			}
			
			if (LIGHT > 0) //亮灯延时
			{
				if (light1sflag == 1)
				{
					light1sflag = 0;
					LIGHT++;
				}
			}
		}
		else
		{ //雷达开关控制
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

				SUM16 = 0;
				calc_average_times = 0;
				SUM1_num = 64;

				stop_times = 2;

				check_light_times = 6;

				SUM1_counter = 0;
				ALL_SUM1 = 0;
			}
		}
	}
}

/***************************************************************************************
  * @说明  	T1中断服务函数
  *	@参数	  无
  * @返回值 无
  * @注		  无
***************************************************************************************/
void TIMER1_Rpt(void) interrupt TIMER1_VECTOR
{
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
		light1scount = 0;
		light1sflag = 1;
	}
	radar_number_count++;
	if (radar_number_count >= bt_and_sigmesh_duty)	//T=500ms~3s
	{
		radar_number_count = 0;
		radar_number_send_flag = 1;
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

void savevar(void)
{
	unsigned char i;
	Flash_EraseBlock(USER_PARAMETER_START_SECTOR_ADDRESS0);
	Delay_us(10000);

	i=(TH/1000)>>8;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+0);
	Delay_us(100);
	
    i=(TH/1000)&0xff;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+1);
	Delay_us(100);
	
    i=LIGHT_TH;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+2);
	Delay_us(100);
	
	i=DELAY_NUM>>8;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+3);
	Delay_us(100);
	i=DELAY_NUM&0xff;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+4);
	Delay_us(100);
	
	i=lightvalue;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+5);
	Delay_us(100);
	
	i=lowlightDELAY_NUM;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+6);
	Delay_us(100);
	
	i=~SWITCHfXBR;//&0xff;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+7);
	Delay_us(100);
	
	i=Linkage_flag;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+8);
	Delay_us(100);	
	
	i=SWITCHflag2;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+9);
	Delay_us(100);	
	
	i=all_day_micro_light_enable;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+10);
	Delay_us(100);
	
	i=temper_value;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+11);
	Delay_us(100);

	i=bt_and_sigmesh_duty>>8;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+12);
	Delay_us(100);
	i=bt_and_sigmesh_duty&0xff;
	FLASH_WriteData(i,USER_PARAMETER_START_SECTOR_ADDRESS0+13);
	Delay_us(100);	

////////////////////////////////////////////////////////
	
	Flash_EraseBlock(USER_PARAMETER_START_SECTOR_ADDRESS1);
	Delay_us(10000);
	FLASH_WriteData(0, USER_PARAMETER_START_SECTOR_ADDRESS1+0);//clear resetbtcnt
	FLASH_WriteData(1, USER_PARAMETER_START_SECTOR_ADDRESS1+1);//clear join count
	
	EA=1;				//-20200927

}

