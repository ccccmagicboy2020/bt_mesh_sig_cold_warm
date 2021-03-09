#define ALLOCATE_EXTERN
#include "HC89S003F4.h"
#include "bluetooth.h"

#define V12 //Ӳ���忨�İ汾

#define USER_PARAMETER_START_SECTOR_ADDRESS0 0x2F00
#define USER_PARAMETER_START_SECTOR_ADDRESS1 0x2F80
#define USER_PARAMETER_START_SECTOR_SIZE	14

#define TH_LOW 30000
#define TH_HIGH 4000000

#define TH_DEF 40000

//��������ֵƫ�Χ
#define MAX_DELTA0 20000 //���ƫ���ֵ
#define MAX_DELTA1 60000 //���ƫ���ֵ

#define MAX_DELAY 1800
//�����ʱ����
#define MAX_BT_DUTY 3000

//�й�����-30��Ӧ8LUX���ҵ�ADֵ,����Ϊ255��ʾ�����й�
#define LIGHT_TH0 255

volatile ulong Timer_Counter = 0;

u8 xdata SUM1_counter = 0; //ƫ��ƽ��ֵ�ļ��������
u8 xdata SUM0_num = 12;	   //SUM0��������
u8 xdata SUM1_num = 64;	   //SUM1��������
ulong xdata SUM01;		   //��һ�ѵ�SUM0��ֵ
ulong xdata SUM2;		   //������snapshot of the SUM1 value
ulong xdata SUM10 = 0;	   //SUM1ֵ�ļ���ƽ��ֵ��ʱ���ϵ��ͺ�ֵ
ulong xdata SUM0 = 0;	   //SUM10��ƽ��ֵ
ulong xdata SUM1 = 0;	   //ƽ�����������ۼӺϵ�˲ʱֵ
ulong xdata ALL_SUM1 = 0;  //SUM1���ۼ�ֵ
ulong xdata SUM16 = 0;	   //2^16�ε��ۼ�ֵ����
ulong xdata SUM = 0;	   //an1��raw�ۼ�ֵ
u16 xdata start_times = 1; //???
u16 xdata times = 0;	   //��ѭ������
ulong xdata TH;			   //���������ֵ������APP���õĸ�Ӧǿ��ת��
ulong xdata MAX_DELTA; //���ƫ��ֵ
u8 xdata alarm_times = 0;
u8 xdata stop_times = 0; //�����Ľ�����

uint xdata LIGHT = 0;	  //��������ļ�����
uint xdata LIGHT_off = 0; //������Ƶķ��Ӽ�����
uint xdata average;		  //an1��rawƽ��ֵ

u8 xdata light_ad;	//����ʵʱֵraw
u8 xdata light_ad0; //������ʼ˲ʱֵraw

u8 xdata check_light_times = 8;	 //���ڹ������ļ�����
u8 xdata calc_average_times = 0; //���ڼ���ƽ��ֵ�ļ�����
u8 xdata LIGHT_TH;
u16 xdata DELAY_NUM;
u8 xdata lowlightDELAY_NUM;
u8 while_1flag = 0;		  //������־ 1==����״̬ 0==���״̬
u8 while_2flag = 0;		  //???

u8 xdata SWITCHflag2 = 0; //�ƿ��صı���������APP����
u8 xdata SWITCHfXBR = 1;  //�״��Ӧ���صı���������APP����
u8 xdata lightvalue = 10; //����ֵ������APP����
u8 xdata slowchcnt = 10;				  //���Ƚ���Ŀ��ֵ
u8 xdata resetbtcnt = 0;				  //Ϊ��������ģ�����õļ�����
u8 xdata XRBoffbrightvalue = 0;			  //���ر��״�ʱ��APP���õ�����ֵ(΢��ֵ)
volatile u16 xdata lowlight1mincount = 0; //timer�ļ�����1ms�Լ�
volatile u8 xdata lowlight1minflag = 0;	  //timer�ķ��ӱ�־
volatile u16 idata light1scount = 0;	  //timer�ļ�����1ms�Լ�
volatile u16 idata light1sflag = 0;		  //timer�����־

u16 idata groupaddr[8] = {0};
u8 idata Linkage_flag = 0;		//������־
u8 xdata temper_value = 0;			//��ůֵ

u8 idata bt_join_cnt = 0;
u8 xdata all_day_micro_light_enable = 0;

u16 xdata radar_trig_times = 0;
u16 xdata radar_trig_times_last = 0;

u8 xdata light_status_xxx = LIGHT_STATUS_EMPTY;
u8 xdata light_status_xxx_last = LIGHT_STATUS_EMPTY;

volatile u16 xdata radar_number_count = 0;
volatile u8 xdata radar_number_send_flag = 0;
u8 idata radar_number_send_flag2 = 0;

u8 xdata person_in_range_flag = PERSON_STATUS_NO_PERSON;
u8 xdata person_in_range_flag_last = PERSON_STATUS_NO_PERSON;

u8 idata ab_last = 0;
u8 idata Exit_network_controlflag = 0;
u16 idata Exit_network_controlflag_toggle_counter = 0;

u8 idata iam_myself_flag = 0;
u16 idata bt_and_sigmesh_duty = 1000;	// unit:ms
u8 xdata find_me_flag = 0;
u8 xdata find_me_counter = 0;

unsigned char PWM0init(unsigned char ab);
unsigned char PWM3init_xxx(unsigned char ab);
unsigned char PWM3init(unsigned char ab);
void Flash_EraseBlock(unsigned int fui_Address); //��������
void FLASH_WriteData(unsigned char fuc_SaveData, unsigned int fui_Address);
void Flash_ReadArr(unsigned int fui_Address, unsigned char fuc_Length, unsigned char *fucp_SaveArr); //��ȡ���ⳤ������
void savevar(void);
void reset_bt_module(void);

unsigned char xdata guc_Read_a[USER_PARAMETER_START_SECTOR_SIZE] = {0x00}; //���ڴ�Ŷ�ȡ������
unsigned char xdata guc_Read_a1[2] = {0x00}; //���ڴ�Ŷ�ȡ������

void Flash_ReadArr(unsigned int fui_Address, unsigned char fuc_Length, unsigned char *fucp_SaveArr)
{
	while (fuc_Length--)
		*(fucp_SaveArr++) = *((unsigned char code *)(fui_Address++)); //��ȡ����
}

void Delay_ms(uint t)
{
	Timer_Counter = 0;
	while (Timer_Counter < t)
	{
		WDTC |= 0x10; //�忴�Ź�
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
  * @˵��  	ϵͳ��ʼ������
  *	@����	  ��
  * @����ֵ ��
  * @ע		  ��
***************************************************************************************/
void InitSYS()
{
	/********************************ϵͳƵ�ʳ�ʼ��***************************************/

	CLKSWR = 0x51;	 //ѡ���ڲ���ƵRCΪϵͳʱ�ӣ��ڲ���ƵRC 2��Ƶ��Fosc=16MHz
	CLKDIV = 0x01;	 //Fosc 1��Ƶ�õ�Fcpu��Fcpu=16MHz
	FREQ_CLK = 0x10; //IAPƵ��

	/**********************************��ѹ��λ��ʼ��**************************************/

	//	BORC = 0xC0;											 //ʹ�ܵ�ѹ��λ1.8V��������ʹ��
	//	BORDBC = 0x01;										 //����ʱ��BORDBC*8TCPU+2TCPU

	/***********************************���ſڳ�ʼ��***************************************/
	WDTC = 0x5F;   //����WDT��λ������ģʽ�½�ֹWDT��ѡ��1024��Ƶ���ڲ���Ƶʱ��44K��
	WDTCCR = 0X20; //0X20/44	=0.73��						//0xFF;	 //���ʱ��Լ6��
				   //�������ʱ��=��WDT��Ƶϵ��*��WDTCCR+1����/�ڲ���ƵRCƵ��
}

/***************************************************************************************
  * @˵��  	��ʱ����ʼ������
  *	@����	  ��
  * @����ֵ ��
  * @ע		  ��
***************************************************************************************/
void Timer_Init()
{
	/**********************************TIM1���ó�ʼ��**************************************/
	TCON1 = 0x00; //T1��ʱ��ʱ��ΪFosc
	TMOD = 0x01;  //T1-16λ��װ�ض�ʱ��/������,T0-16λ��ʱ��

	//Tim1����ʱ�� 	= (65536 - 0xFACB) * (1 / (Fosc /Timer��Ƶϵ��))
	//				= 1333 / (16000000 / 12)
	//				= 1 ms

	//T1��ʱ1ms
	//���Ƴ�ֵ 	= 65536 - ((1/1000) / (1/(Fosc / Timer��Ƶϵ��)))
	//		   	= 65536 - ((1/1000) / (1/(16000000 / 12)))
	//			= 65536 - 1333
	//			= 0xFACB

	TH1 = 0xFA;
	TL1 = 0xCB;	  //T1��ʱ1ms
	IE |= 0x08;	  //��T1�ж�
	TCON |= 0x40; //ʹ��T1

	TH0 = 0xCB;
	TL0 = 0xEB; //T0��ʱʱ��10ms

	TCON |= 0x10; //ʹ��T0
}

/***************************************************************************************
  * @˵��  	UART1��ʼ������
  *	@����	  ��
  * @����ֵ ��
  * @ע		  ��
***************************************************************************************/
void UART1_Init()
{
	/**********************************UART���ó�ʼ��**************************************/
	P2M0 = P2M0 & 0xF0 | 0x08; //P20����Ϊ�������
	P0M2 = P0M2 & 0xF0 | 0x02; //P04����Ϊ��������
	P0_4 = 1;
	TXD_MAP = 0x20; //TXDӳ��P20
	RXD_MAP = 0x04; //RXDӳ��P04
	T4CON = 0x06;	//T4����ģʽ��UART1�����ʷ�����

	//�����ʼ���
	//������ = 1/16 * (T4ʱ��ԴƵ�� / ��ʱ��4Ԥ��Ƶ��) / (65536 - 0xFF98)
	//       = 1/16 * ((16000000 / 1) / 104)
	//		 = 9615.38(���0.16%)

	//������9600
	//���Ƴ�ֵ = (65536 - ((T4ʱ��ԴƵ�� / ��ʱ��4Ԥ��Ƶ��) * (1 / 16)) / ������)
	//		   = (65536 - (16000000 * (1 / 16) / 9600))
	//		   = (65536 - 104.167)
	//         = FF98
	//0xFF98->9600
	//0xFFCC->19200
	//0xFFEF->57600

	TH4 = 0xFF;
	TL4 = 0x98;	  //������9600		//0xEE;				//������56000
	SCON2 = 0x02; //8λUART�������ʿɱ�
	SCON = 0x10;  //�����н���
	IE |= 0X10;	  //ʹ�ܴ����ж�
				  //EA = 1;							              	 //ʹ�����ж�
}

/***************************************************************************************
  * @˵��  	ADC��ʼ������
  *	@����	  ��
  * @����ֵ ��
  * @ע		  ��
***************************************************************************************/
void ADC_Init()
{

	ADCC0 |= 0x03; //�ο�ԴΪ�ڲ�2V
	ADCC0 |= 0x80; //��ADCת����Դ
	Delay_us(20);  //��ʱ20us��ȷ��ADCϵͳ�ȶ�

#ifdef XBR403_03_2
	ADCC1 = 0x00;  //ѡ���ⲿͨ��0
#endif
	
#ifdef XBR403
	ADCC1 = 0x02;  //ѡ���ⲿͨ��2
#endif

#ifdef V12
	ADCC1 = 0x01;  //ѡ���ⲿͨ��1
#endif
	
	ADCC2 = 0x4B;  //8��Ƶ	  //ת�����12λ���ݣ������Ҷ��룬ADCʱ��16��Ƶ-1MHZ//0X4B-8��Ƶ//0X49-4��Ƶ
}

/***************************************************************************************
  * @˵��  	IO�ڳ�ʼ������
  *	@����	  ��
  * @����ֵ ��
  * @ע		  ��
***************************************************************************************/
void GPIO_Init()
{
	//P0M0�ָ�4λ���4λ����4λ����P00�����������4λ����P01��������������Դ�����
	//P0M1��4����P03����4����P02
	//P1M2��4����P15����4����P14

	// 	P0M0 = P0M0&0xF0|0x08;		      //P00����Ϊ�������
	// 	P0M0 = P0M0&0x0F|0x30;				  //P01����Ϊģ������
	// 	P0M3 = P0M3&0x0F|0x30;				  //P07����Ϊģ������
	// 	P0M3 = P0M3&0xF0|0x08;		      //P06����Ϊ�������

#ifdef V11

	P0M0 = P0M0 & 0xF0 | 0x08; //P00����Ϊ�������

	P0M0 = P0M0 & 0x0F | 0x30; //P01����Ϊģ������
	//P0M3 = P0M3&0x0F|0x30;				  //P07����Ϊģ������
	//	P0M0 = P0M0&0x0F|0x80;		      //P01����Ϊ�������

	P0M2 = P0M2 & 0x0F | 0x80; //P05����Ϊ�������

	P0M3 = P0M3 & 0xF0 | 0x03; //P06����Ϊģ������  //|0x08;		      //P06����Ϊ�������

	P0M3 = P0M3 & 0x0F | 0x20; //P07����Ϊ��������

#endif

#ifdef V10

	P0M0 = P0M0 & 0xF0 | 0x08; //P00

	P0M0 = P0M0 & 0x0F | 0x30; //P01
	P0M3 = P0M3 & 0x0F | 0x30; //P07
							   //	P0M0 = P0M0&0x0F|0x80;		      //P01

	P0M3 = P0M3 & 0xF0 | 0x08; //P06

#endif

#ifdef V12

	P1M0 = P1M0 & 0xFF | 0x88; //P10����Ϊ�������
							   //P11����Ϊ�������
							   
							   

	P0M0 = P0M0 & 0x0F | 0x30; //P01����Ϊģ������

	P2M1 = P2M1 & 0xF0 | 0x03; //P22����Ϊģ������

	//P0M3 = P0M3&0x0F|0x30;				  //P07����Ϊģ������
	//	P0M0 = P0M0&0x0F|0x80;		      //P01����Ϊ�������

	//P0M2 = P0M2&0x0F|0x80;		      //P05����Ϊ�������

	//P0M3 = P0M3&0xF0|0x03;			//P06����Ϊģ������  //|0x08;		      //P06����Ϊ�������

	//	P0M3 = P0M3&0x0F|0x20;				  //P07����Ϊ��������

#endif

#ifdef XBR403
	//PWM & ADC
	P1M0 = P1M0 & 0xF0 | 0x08; //P10����Ϊ�������
	P0M0 = P0M0 & 0xFF | 0x88; //P00����Ϊ�������
							   //P01����Ϊ�������
	P0M1 = P0M1 & 0xFF | 0x83; //P03����Ϊ�������
							   //P02����Ϊģ������
	P0M3 = P0M3 & 0xF0 | 0x08; //P06����Ϊ�������
	P2M1 = P2M1 & 0xF0 | 0x03; //P22����Ϊģ������
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

	ADCC1 = ch;	   //ѡ���ⲿͨ��
	ADCC0 |= 0x40; //����ADCת��
	while (!(ADCC0 & 0x20))
		;			//�ȴ�ADCת������
	ADCC0 &= ~0x20; //�����־λ

	Delay_us(100);

	ad_sum = 0;

	for (i = 0; i < 16; i++)
	{
		ADCC0 |= 0x40; //����ADCת��
		while (!(ADCC0 & 0x20))
			;			//�ȴ�ADCת������
		ADCC0 &= ~0x20; //�����־λ
		ad_sum += ADCR; //��ȡADC��ֵ

		Delay_us(20);
	}

#ifdef XBR403_03_2
	ADCC1 = 0x00;  //ѡ���ⲿͨ��0
#endif

#ifdef XBR403
	ADCC1 = 0x02;  //ѡ���ⲿͨ��2
#endif

#ifdef V12
	ADCC1 = 0x01;  //ѡ���ⲿͨ��1
#endif
	
	i = ad_sum >> 8;

	Delay_us(100);
	return (i);
}

void set_var(void)
{

	Flash_ReadArr(USER_PARAMETER_START_SECTOR_ADDRESS0, USER_PARAMETER_START_SECTOR_SIZE, guc_Read_a); //��ȡ��ַ��������

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

	if (while_1flag == 0)//���״̬
	{
		ADCC0 |= 0x40; //����ADCת��
		while (!(ADCC0 & 0x20))
			;			//�ȴ�ADCת������
		ADCC0 &= ~0x20; //�����־λ
		k = ADCR;		//��ȡADC��ֵ

		times++;

		SUM += k;

		//�����ź�ֵ��ֱ����ѹƫ��ֵ
		if (k > average)
		{
			k -= average;
		}
		else
		{
			k = average - k;
		}
		SUM1 += k;

		if ((times & 0x1ff) == 0) //ÿ256��ѭ�����һ��״̬
		{
			if (LIGHT > 0) //���ڰ����Ĺ�����ʱ����
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
			else if (LIGHT_off == 1)//��Ƽ�ʱ��ʼʱ�䰵
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

		if (times >= 8192) //ÿ250ms�������ж�һ��
		{

			WDTC |= 0x10; //�忴�Ź�

			times = 0;

			calc_average_times++;

			SUM16 += SUM;

			if (calc_average_times >= 8) //ÿ2.5S���¼���һ��ֱ����ѹֵ
			{
				calc_average_times = 0;

				SUM16 >>= 16;
				//SUM16/=96000;//102400;
				average += SUM16;
				average /= 2;	//�ó�ƽ��ֵ
				SUM16 = 0;
			}

			if (check_light_times < 8) //2s	��ȡһ�θй�ADֵ��˲ʱ�Ա�ֵÿ2s���Ҹ���һ��
			{
				check_light_times++;
			}
			else
			{
				if (LIGHT == 0)	//����δ��ʼ��Ҳ����δ��⵽�״�Ŀ��
				{
					
					#ifdef XBR403_03_2
						light_ad = read_ad(7); //�л���an7
					#endif

					#ifdef XBR403
						light_ad = read_ad(10); //�л���an10
					#endif

					#ifdef V12
						light_ad = read_ad(10); //�л���an10
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
					SUM0 = 1000000; //��Ƴ�ֵ
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
				MAX_DELTA = SUM10 >> 3; //����ͻ��(���ƫ��ֵ)
				if (MAX_DELTA < MAX_DELTA0)
					MAX_DELTA = MAX_DELTA0;
				if (MAX_DELTA > MAX_DELTA1)
					MAX_DELTA = MAX_DELTA1; //��֤���ƫ��ֵ��һ����Χ��

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

						if ((SUM10 < 8000000) && (SUM < 400000))
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

				if (SUM1 > (SUM0 + TH))	//��ʽ�ж�
				{
					if ((light_ad <= LIGHT_TH) || (start_times > 0))	//��һ������֮�²�����
					{
						{
							if (LIGHT == 0)
								SUM01 = SUM0;
							LIGHT = 1;
							/////////////////////////////////////////////
							radar_trig_times++;
							person_in_range_flag = PERSON_STATUS_HAVE_PERSON;
							radar_number_send_flag2 = 1;
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
//			send_data(SUM1 >> 8); //20200927	������
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

		Delay_ms(250);

		SUM16 = 0;
		calc_average_times = 0;
		SUM1_num = 64;

		stop_times = 2;
		check_light_times = 6;

		SUM1_counter = 0;
		ALL_SUM1 = 0;
	}
}

//����һ����ֵ
void wait1(void)
{
	u8 i, j;

	//�ȴ�ֱ����ѹ�ȶ�
	j = 0;
	while (1)
	{
		SUM = 0;

		for (i = 0; i < 128; i++) //
		{

			ADCC0 |= 0x40; //����ADCת��
			while (!(ADCC0 & 0x20))
				;			//�ȴ�ADCת������
			ADCC0 &= ~0x20; //�����־λ
			//k = ADCR;				//��ȡADC��ֵ

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

//����һ��2^16�ε�ƽ��ֵ
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
			ADCC0 |= 0x40; //����ADCת��
			while (!(ADCC0 & 0x20))
				;			//�ȴ�ADCת������
			ADCC0 &= ~0x20; //�����־λ
			k = ADCR;		//��ȡADC��ֵ

			SUM += k;
		}
		WDTC |= 0x10; //�忴�Ź�
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
	PWM0_MAP = 0x27;					//PWM0ͨ��ӳ��P27��
#endif
	
#ifdef XBR403
	PWM0_MAP = 0x11;					//PWM0ͨ��ӳ��P11��
#endif

#ifdef V12
	PWM0_MAP = 0x11;					//PWM0ͨ��ӳ��P11��
#endif
	
	PWM0C = 0x01;					  	//PWM0����Ч��PWM01����Ч��ʱ��8��Ƶ 
	
	//����ģʽ�£�PWM0��PWM01����һ�����ڼĴ���
	//PWM0��ռ�ձȵ���ʹ��			PWM0���ռ�ձȼĴ���
	//PWM01��ռ�ձȵ���ʹ��			PWM0��������Ĵ���

	//���ڼ��� 	= 0x03ff / (Fosc / PWM��Ƶϵ��)		��Fosc��ϵͳʱ�����õĲ��֣�
	//			= 0x03ff / (16000000 / 8)			
	// 			= 1023   /2000000
	//			= 511.5us		   		Լ1.955kHz

	PWM0PH = 0x01;						//���ڸ�4λ����Ϊ0x03
	PWM0PL = 0xFF;						//���ڵ�8λ����Ϊ0xFF

	//ռ�ձȼ���= 0x0155 / (Fosc / PWM��Ƶϵ��)		��Fosc��ϵͳʱ�����õĲ��֣�
	//			= 0x0155 / (16000000 / 8)			
	// 			= 341 	 / 2000000
	//			= 170.5us		   ռ�ձ�Ϊ 170.5/511.5 = 33.3%

	PWM0DH = (u8)(j11>>8);				//PWM0��4λռ�ձ�0x01
	PWM0DL = (u8)j11;					//PWM0��8λռ�ձ�0x55

	PWM0EN = 0x0F;						//ʹ��PWM0�������ڶ���ģʽ	
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
	/************************************PWM3��ʼ��****************************************/
	//P0M3 = P0M3&0xF0|0x08;		//P06����Ϊ�������
	PWM3_MAP = 0x05; //PWM3ӳ��P05��

#endif

#ifdef V10
	PWM3_MAP = 0x06; //PWM3ӳ��P05��

#endif

#ifdef V12
	PWM3_MAP = 0x10; //PWM3ӳ��P10��

#endif

#ifdef XBR403_03_2
	PWM3_MAP = 0x01;					//PWM3ͨ��ӳ��P01��
#endif

#ifdef XBR403
	PWM3_MAP = 0x10; //PWM3ӳ��P10��
#endif


	//���ڼ��� 	= 0xFF / (Fosc / PWM��Ƶϵ��)		��Fosc��ϵͳʱ�����õĲ��֣�
	//			= 0xFF /(16000000 / 4)
	// 			= 255 /4000000
	//			= 63.75us		��15.69KHZ

	PWM3P = 0xFF; //���ڼĴ���//PWM����Ϊ0xFF
	//��Ч��ƽʱ����㣨��ռ�ձȣ�
	//			= 0x55 / (Fosc / PWM��Ƶϵ��)		��Fosc��ϵͳʱ�����õĲ��֣�
	//			= 0x55 /(16000000 / 4)
	// 			= 85 /4000000
	//			= 21.25us		ռ�ձ�Ϊ 21.25 / 63.75 = 34%

	PWM3D = j11;  //PWMռ�ձ����ã�ռ�ձȼĴ���p90
	PWM3C = 0x94; //PWM���ƼĴ�����ʹ��PWM3���ر��жϣ����������ʱ��16��Ƶ

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
	PWM0init(bb);//��
	PWM3init_xxx(aa);//ů
	
	return 0;
}

/***************************************************************************************
  * @˵��  	������
  *	@����	  ��
  * @����ֵ ��
  * @ע		  ��
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

	LVDC = 0xAA; //LVD����2.4V,��ֹ�ж�
	//	����ʱ�� = 	(0xFF + 2) * 1/Fcpu
	//			 =	(0xFF + 2) / 16000000	����ǰ��CPUʱ�ӣ�
	//			 =	16.0625us
	LVDDBC = 0xFF; //��������ʱ��
	LVDC &= ~0x08; //���LVD�жϱ�־λ
				   //
	EA = 1;

	Delay_ms(200);
	
	#ifdef XBR403_03_2
		light_ad = read_ad(7); //�л���an7
	#endif
	
	#ifdef XBR403
		light_ad = read_ad(10); //�л���an10
	#endif
	
	#ifdef V12
		light_ad = read_ad(10); //�л���an10
	#endif
	
	light_ad0 = light_ad;

	EA = 0;
	set_var(); //��flash��ȡ������
	
	PWM3init(100);

	EA = 1;

	wait1();

	slowchcnt = lightvalue;

	PWM3init(lightvalue);

	Delay_ms(300);

	wait2();

	SUM = 0;
	
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
			if (3 <= find_me_counter)	//1 min toggle led
			{
				find_me_flag = 0;
				find_me_counter = 0;
			}
		}
		//��������
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
		//��״̬����
		if (light_status_xxx != light_status_xxx_last)
		{
			mcu_dp_enum_update(DPID_LIGHT_STATUS,light_status_xxx);
			light_status_xxx_last = light_status_xxx;
		}
		//��״̬����
		if (person_in_range_flag != person_in_range_flag_last)
		{
			mcu_dp_enum_update(DPID_PERSON_IN_RANGE,person_in_range_flag);
			person_in_range_flag_last = person_in_range_flag;
		}		
		
		//���ڿ���
		if (1 == radar_number_send_flag)
		{
			radar_number_send_flag = 0;
			//�״��д���
			if (1 == radar_number_send_flag2)
			{
				radar_number_send_flag2 = 0;
				//�״��������
				if (radar_trig_times_last != radar_trig_times)
				{
					mcu_dp_value_update(DPID_RADAR_TRIGGER_TIMES,radar_trig_times);
					radar_trig_times_last = radar_trig_times;					
				}
				//���������Ļ�
				if (Linkage_flag == 1)
				{
					for (i=0;i<8;i++)
					{
						//����Ⱥ�����Ⱥ��
						if (groupaddr[i] != 0)
						{	//����mesh������
							iam_myself_flag = 1;
							mcu_dp_enum_mesh_update(DPID_PERSON_IN_RANGE_EX, 0, groupaddr[i]);
							Delay_ms(100);								
						}							
					}
				}				
			}
		}		

		WDTC |= 0x10; //�忴�Ź�

		if (while_1flag == 0)//���״̬
		{
			if ((times & 0x1f) == 0)
				bt_uart_service();	//���ڽ���첽����
		}

		if (SWITCHfXBR == 1) //�״￪����
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

			if (LIGHT_off > 0) //�����ʱ����λ����
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
			
			if (LIGHT > 0) //������ʱ
			{
				if (light1sflag == 1)
				{
					light1sflag = 0;
					LIGHT++;
				}
			}
		}
		else
		{ //�״￪�ؿ���
			while_2flag = 0;
			if (SWITCHflag2 == 0) //�ص�
			{
				PWM3init(0);
			}
			else
			{ //����
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
  * @˵��  	T1�жϷ�����
  *	@����	  ��
  * @����ֵ ��
  * @ע		  ��
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
  * @˵��  	UART1�жϷ�����
  *	@����	  ��
  * @����ֵ ��
  * @ע		  ��
***************************************************************************************/

void UART1_Rpt(void) interrupt UART1_VECTOR
{
	u8 i;
	//u16 t;

	if (SCON & 0x01) //�жϽ����жϱ�־λ
	{
		i = SBUF;
		uart_receive_input(i); //mcu_sdk
		SCON &= ~0x01;		   //��������жϱ�־λ
		EA = 1;
	}
}

void Flash_EraseBlock(unsigned int fui_Address)
{
	while (1)
	{
		LVDC &= ~0x08; //���LVD�жϱ�־λ
		P0_0 = 0;
		if ((LVDC & 0x08) == 0)
			break;
	}
	P0_0 = 1;
	EA = 0;
	IAP_CMD = 0xF00F;		//Flash����
	IAP_ADDR = fui_Address; //д�������ַ
	IAP_CMD = 0xD22D;		//ѡ�������ʽ�� ��������
	IAP_CMD = 0xE11E;		//������IAP_ADDRL&IAP_ADDRHָ��0xFF��ͬʱ�Զ�����
							//EA=1;
}

/**
  * @˵��  	д��һ���ֽ����ݵ�Flash����
  *         �ú�������Ե�ַ���룬���������IAP����Ӧ���ֲ�
  * @����  	fui_Address ��FLASH��ַ
  *	@����	  fucp_SaveData��д�������
  * @����ֵ ��
  * @ע		  д֮ǰ�����ȶԲ������������в���
  */
void FLASH_WriteData(unsigned char fuc_SaveData, unsigned int fui_Address)
{
	while (1)
	{
		LVDC &= ~0x08; //���LVD�жϱ�־λ
		P0_0 = 0;
		if ((LVDC & 0x08) == 0)
			break;
	}
	P0_0 = 1;
	EA = 0;
	IAP_DATA = fuc_SaveData;
	IAP_CMD = 0xF00F; //Flash����
	IAP_ADDR = fui_Address;
	IAP_CMD = 0xB44B; //�ֽڱ��
	IAP_CMD = 0xE11E; //����һ�β���
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

