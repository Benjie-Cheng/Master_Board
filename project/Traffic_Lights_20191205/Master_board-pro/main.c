/*---------------------------------------------------------------------*/
/* --- Fuctin: Mini traffic lights- -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/
/*************	��������˵��	**************
��ͨ�ƣ�0.28�繲��������ܣ�TXD,RXD P32��P55�����Զ��忪����
��Ƭ����STC15W201AS-SOP16,���ö�̬ɨ�跽ʽ��ԼIO
�����ʣ�115200
������־��
1��
2��
3��
4��
******************************************/
#include "config.h"
#include "debug.h"

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))	//Timer 0 �ж�Ƶ��, 1000��/��
#define	Baudrate1	115200UL                            //ͨ�Ų�����115200


/*************	IO����	**************/
sbit	LED8_A = P1^1;
sbit	LED8_B = P1^0;
sbit	LED8_C = P1^5;
sbit	LED8_D = P1^4;
sbit	LED8_E = P1^3;
sbit	LED8_F = P1^2;
sbit	LED8_G = P5^4;

sbit	P_COM0 = P3^6;
sbit	P_COM1 = P3^7;
sbit	P_COM2 = P3^3;

enum{
	LEDL_RED=5, 
	LEDL_GREEN=1,
	LEDR_RED=3, 
	LEDR_GREEN=2, 
	LEDG_RED=0,
	LEDG_GREEN=4,  	
};
typedef enum{
	ON=1,
	OFF=0,
	ON_ALL=0xff,
	OFF_ALL=0xfe,
	OFF_ON=0xfd	
}LogicType;
	
#define GPIO_CHECK_PIN P30
#define GPIO_OUT_PIN   P31

//========================================================================
/********************** ������ ************************/
BOOL B_1ms;	 //1ms��־
BOOL B_TX1_Busy;  //����æ��־

//�������ʱ�� 1�� ����ر���
volatile unsigned char vGu8TimeFlag_1=0;
volatile u32 vGu32TimeCnt_1=0;	
volatile unsigned char vGu8TimeFlag_2=0;
volatile unsigned int vGu16TimeCnt_2=0;	
volatile unsigned char vGu8LeveCheckFlag=0;

#define GPIO_FILTER_TIME 50 //�˲��ġ� �ȶ�ʱ�䡱 50ms
#define LED_TIME_1S  1000  //ʱ���� 10000ms
/*-----------------�Զ���ʱ������---------------------*/
#define RIGHT_LED_TIME 15
#define FRONT_LED_TIME 25
#define LEFT_LED_TIME 35	
#define LowTimeS 5
/*----------------------------------------------------*/
volatile unsigned int g_count = FRONT_LED_TIME;
volatile unsigned int y_count = RIGHT_LED_TIME;
volatile unsigned int r_count = LEFT_LED_TIME;
BOOL FlashFlag = FALSE;
BOOL LowTime = FALSE;

static u8 Gu8Step = 0; //�����ʱ�� 1 �� switch �л�����

#define	INDEX_MAX 3	//��ʾλ����
static u8 	Display_Code[1]={0x00};		    //1��com ��״̬;
static u8 	LED8[3] = {0x00,0x00,0x00};		//��ʾ����֧��2λ,ʮλ����λ,LED״̬��
static u8	display_index = 0;				//��ʾλ����	

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //���մ���1��������
u8 RX_CONT = 0; 
u8 code t_display[]={						//������
//	 0    1    2    3    4    5    6    7    8    9
	0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,
//black
	0xff,
};	

void uart1_config();	// ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void TaskDisplayScan(void);
void vTaskfFlashLed(void);
void TaskLedRunScan(void);

typedef struct _TASK_COMPONENTS
{
	u8 Run;                 // �������б�ǣ�0-�����У�1����
	u16 Timer;               // ��ʱ��
	u16 ItvTime;             // �������м��ʱ��
	void (*TaskHook)(void); // Ҫ���е�������
} TASK_COMPONENTS;   
static TASK_COMPONENTS TaskComps[] =
{
	{0, 1000,  1000, vTaskfFlashLed},           // ����ɨ��1s
//	{0, 80, 80, TaskLedRunScan},         		// �����	
//	{0, 50, 50, vKey_Service}					// �����������50ms
//	{0, 10, 10, TaskRTC}				        // RTC����ʱ
//	{0, 30, 30, TaskDispStatus}
	// ��������������
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // ����LED
//	TAST_LED_RUN,        	//�����
//	TASK_KEY_SERV,
//	TASK_RTC,
//	TASK_DISP_WS,             // ����״̬��ʾ// ��������������
	// ��������������
	TASKS_MAX                 // �ܵĿɹ�����Ķ�ʱ������Ŀ
} TASK_LIST;

//========================================================================
// ����: u8 ms��ʱ������
//========================================================================
void  delay_ms(u8 ms)
{
     u16 i;
	 do{
	      i = MAIN_Fosc / 13000;
		  while(--i)	;   //14T per loop
     }while(--ms);
}
void timer0_init(void)
{
	AUXR = 0x80;	//Timer0 set as 1T, 16 bits timer auto-reload, 
	TH0 = (u8)(Timer0_Reload / 256);
	TL0 = (u8)(Timer0_Reload % 256);
	ET0 = 1;	//Timer0 interrupt enable
	TR0 = 1;	//Tiner0 run
	EA = 1;		//�����ж�
}	
//========================================================================
// ����: set_timer2_baudraye(u16 dat)
// ����: ����Timer2�������ʷ�������
// ����: dat: Timer2����װֵ.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================

void set_timer2_baudraye(u16 dat)
{
	AUXR &= ~(1<<4);	//Timer stop
	AUXR &= ~(1<<3);	//Timer2 set As Timer
	AUXR |=  (1<<2);	//Timer2 set as 1T mode
	TH2 = dat / 256;
	TL2 = dat % 256;
	IE2  &= ~(1<<2);	//��ֹ�ж�
	AUXR |=  (1<<4);	//Timer run enable
}

//========================================================================
// ����: Ӳ����ʼ�� ��
//========================================================================
void stc15x_hw_init(void)
{
	P0n_standard(0xff);	//����Ϊ׼˫���
	P1n_standard(0xff);	//����Ϊ׼˫���
	P2n_standard(0xff);//����Ϊ׼˫���
	P0n_push_pull(0xc8);//����Ϊǿ����
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���	
	timer0_init();
	uart1_config();
	//GPIO_OUT_PIN = 0;//����ͣ���P26���
}

void TaskRemarks(void)
{
	u8 i;    
	for (i=0; i<TASKS_MAX; i++)           // �������ʱ�䴦��
	{
		if (TaskComps[i].Timer)           // ʱ�䲻Ϊ0
		{
			TaskComps[i].Timer--;         // ��ȥһ������
			if (TaskComps[i].Timer == 0)  // ʱ�������
			{
				TaskComps[i].Timer = TaskComps[i].ItvTime;       // �ָ���ʱ��ֵ��������һ��
				TaskComps[i].Run = 1;           // �����������	
			}
		}
	}
}
void TaskProcess(void)
{
	u8 i;    
	for (i=0; i<TASKS_MAX; i++)           // �������ʱ�䴦��
	{
		if (TaskComps[i].Run)           // ʱ�䲻Ϊ0
		{
			TaskComps[i].TaskHook();         // ��������
			TaskComps[i].Run = 0;          // ��־��0
		}
	}
}
void Kled_Set(LogicType type,u8 Nled)//LED����
{
	u8 val;
	val = LED[2];
	if(type==ON)
		LED[2] = setbit(val,Nled);//�ߵ�ƽ��ͨ
	else if(type==OFF)
		LED[2] = clrbit(val,Nled);	
	else if(ON_ALL == type)	
		LED[2] = 0xff;
	else if(OFF_ALL == type)
		LED[2] = 0x00;
	else if(OFF_ON == type)
		LED[2] = reversebit(val,Nled);//��ת
}
void BitX_Set(LogicType type,u8 Xbit)
{
	u8 val=0;
	if(type==ON)
		Display_Code[0] = setbit(val,Xbit);//�ߵ�ƽ��ͨ
	else if(type==OFF)
		Display_Code[0] = clrbit(val,Xbit);	
	else if(ON_ALL == type)	
		Display_Code[0] = 0xff;
	else if(OFF_ALL == type)
		Display_Code[0] = 0x00;
	else if(OFF_ON == type)
		Display_Code[0] = reversebit(val,Xbit);//��ת
	
}
void TaskDisplayScan(void)//10ms ˢ��һ��
{ 
	if(FlashFlag)
	{
		P_COM0 = 0;
		P_COM1 = 0;
		P_COM3 = 0;
		return;
	}
	for(display_index = 0;display_index < INDEX_MAX;display_index++){
		BitX_Set(ON,display_index);//com ��ɨ�裬������λ
#if 0
	_nop_();
	_nop_();
	BitX_Set(OFF,display_index);//�ı�����
#endif	
		P_COM0=P_COM1=P_COM2=0;//��ʾ����ǰ�ȹر�λ����Ӱ;
		_nop_();
		//������ʾ����ͨ����ʾ
		LED8_A = getbit(LED8[display_index],0);
		LED8_B = getbit(LED8[display_index],1);
		LED8_C = getbit(LED8[display_index],2);
		LED8_D = getbit(LED8[display_index],3);
		LED8_E = getbit(LED8[display_index],4);
		LED8_F = getbit(LED8[display_index],5);
		LED8_G = getbit(LED8[display_index],6);

		//�������ʾ
		P_COM0 = getbit(Display_Code[0],0);
		P_COM1 = getbit(Display_Code[0],1);
		P_COM2 = getbit(Display_Code[0],2);
	}
}
void vTaskfFlashLed(void)
{ 
	if(!LowTime)
		return;
	FlashFlag = ~FlashFlag;//0.5s
		//key_led_reverse();
	switch(Gu8Step)
	{
		case 0:	
			print_char(mod(g_count,10));
			print_char(rem(g_count,10));
		break;
		case 1:
			print_char(mod(y_count,10));
			print_char(rem(y_count,10));
		break;			
		case 2:
			print_char(mod(r_count,10));
			print_char(rem(r_count,10));
		break;	
	}
}
void Gpio_ValRead(void)	
{	
	static unsigned char Su8KeyLock1; //1 �Ű���������

	if(GPIO_CHECK_PIN==1)
	{
		Su8KeyLock1=0; //��������
		vGu8TimeFlag_2=0;
		vGu16TimeCnt_2=0;
	}
	else if(0==Su8KeyLock1)
	{
		vGu8TimeFlag_2=1;//������ʱ��2
		if(vGu16TimeCnt_2>=GPIO_FILTER_TIME) //�˲��ġ� �ȶ�ʱ�䡱 GPIO_FILTER_TIME�� ������ 50ms��
		{
			vGu8TimeFlag_2=0;//�˲�ʱ�䵽����ʱ����0
			Su8KeyLock1=1; //����������,����һֱ����
		}		
	}
}	
void Turn_Direction(u8 status)
{
	switch(status)
	{
		case 0://��ת
			Kled_Set(ON,LEDL_GREEN);
			Kled_Set(OFF,LEDL_RED);
			Kled_Set(ON,LEDG_RED);
			Kled_Set(OFF,LEDG_GREEN);
			Kled_Set(ON,LEDR_RED);
			Kled_Set(OFF,LEDR_GREEN);
			break;
		case 1://��ǰ
			Kled_Set(OFF,LEDL_GREEN);
			Kled_Set(ON,LEDL_RED);
			Kled_Set(ON,LEDG_GREEN);
			Kled_Set(OFF,LEDG_RED);
			Kled_Set(OFF,LEDR_GREEN);
			Kled_Set(ON,LEDR_GREEN);
			break;
		case 2://����
			Kled_Set(OFF,LEDL_GREEN);
			Kled_Set(ON,LEDL_RED);
			Kled_Set(OFF,LEDG_GREEN);
			Kled_Set(ON,LEDG_RED);
			Kled_Set(ON,LEDR_GREEN);
			Kled_Set(OFF,LEDR_RED);
		break;
	}
}
void Traffic_Led(void)
{
	switch(Gu8Step)
	{
		case 0://��ת��35s
			vGu8TimeFlag_1 = 1;
			Turn_Direction(Gu8Step);
			vGu8LeveCheckFlag = 0;//�رյ�ƽ���
			r_count = LEFT_LED_TIME;
			LED8[0] = t_display[mod(g_count,10)];
			LED8[1] = t_display[rem(g_count,10)];
			if(g_count<=LowTimeS)
				LowTime = TRUE;
			else
				LowTime = FALSE;
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				g_count--;
				vGu32TimeCnt_1 = 0;
				if(g_count == 0)
					Gu8Step++;
			}	
			break;
		case 1://��ǰ,25s
			g_count = FRONT_LED_TIME;
			Turn_Direction(Gu8Step);
			LED8[0] = t_display[mod(y_count,10)];
			LED8[1] = t_display[rem(y_count,10)];	
			vGu8LeveCheckFlag = 1;//������ƽ���
			if(y_count<=LowTimeS)
				LowTime = TRUE;
			else
				LowTime = FALSE;
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				y_count--;	
				vGu32TimeCnt_1 = 0;
				if(y_count == 0)
					Gu8Step++;
			}
			break;
		case 2://����,15s
			y_count = RIGHT_LED_TIME;
			Turn_Direction(Gu8Step);
			LED8[0] = t_display[mod(r_count,10)];
			LED8[1] = t_display[rem(r_count,10)];	
			if(r_count<=LowTimeS)
				LowTime = TRUE;
			else
				LowTime = FALSE;
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				r_count--;		
				vGu32TimeCnt_1 = 0;
				if(r_count == 0)
				{
					Gu8Step=0;		
					vGu32TimeCnt_1 = 0;
					vGu8TimeFlag_1 = 0;
				}
			}
			break;	
		default:	
			break;
	}
}
//========================================================================
// ����: void main(void)
// ����: ������.
// ����: ��.
// ����: ��.
// �汾: V1.0, 2012-10-22
//========================================================================
void main(void)
{
	stc15x_hw_init();
	puts_to_SerialPort("I am Traffic Lights!\n");
	while(1)
	{
		if(B_1ms)	//1ms��
		{
			B_1ms = 0;	
			Traffic_Led();//ָʾ��״̬�л�		
			if(vGu8LeveCheckFlag)
				Gpio_ValRead();//�͵�ƽ���
			else
			{
				vGu8TimeFlag_2=0;//����⣬��ʱ����0
			}
		}
		TaskDisplayScan();
		TaskProcess();//��ʾ����˸led
	}
}

//========================================================================
// ����: Timer0 1ms�жϺ�����
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	B_1ms = 1;		//1ms��־
	TaskRemarks();
	if(vGu8TimeFlag_1){
		vGu32TimeCnt_1++;
	}
	else
		vGu32TimeCnt_1 = 0;
	if(vGu8TimeFlag_2)
		vGu16TimeCnt_2++;
}
//========================================================================
// ����: void print_string(u8 *puts)
// ����: ����1�����ַ���������
// ����: puts:  �ַ���ָ��.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================

void print_string(u8 *puts)
{
  for (; *puts != '*';	puts++)   	//����ֹͣ��*����
	{
		SBUF = *puts;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
	}
}
void print_char(u8 dat)
{
		SBUF = dat;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
}
void puts_to_SerialPort(u8 *puts)
{
    for (; *puts != 0;	puts++)   	//����ֹͣ��0����
	{
		SBUF = *puts;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
	}
}
//========================================================================
// ����: void	uart1_config()
// ����: UART1��ʼ��������
// ����: brt: ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
#if 1
void uart1_config()
{
	/*********** ������ʹ�ö�ʱ��2 *****************/
	AUXR |= 0x01;		//S1 BRT Use Timer2;
	set_timer2_baudraye(65536UL - (MAIN_Fosc / 4) / Baudrate1);

	SCON = (SCON & 0x3f) | 0x40;	//UART1ģʽ, 0x00: ͬ����λ���, 0x40: 8λ����,�ɱ䲨����, 0x80: 9λ����,�̶�������, 0xc0: 9λ����,�ɱ䲨����
	S1_USE_P30P31();//UART1 ʹ��P30 P31��	Ĭ��
	ES  = 1;	//�����ж�
	REN = 1;	//�������
	EA = 1;

	B_TX1_Busy = 0;
}
#endif
//========================================================================
// ����: void uart1_int (void) interrupt UART1_VECTOR
// ����: UART1�жϺ�����
// ����: nine.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
void uart1_int (void) interrupt UART1_VECTOR
{
	if(RI)
	{
		RI = 0;
		Rec_Buf[RX_CONT] = SBUF;                    //�Ѵ���1����SBUF�Ĵ����������δ�ŵ�����Rec_Buf��
		RX_CONT++;                                   
		if(RX_CONT>Buf_Max)                          //���������ڶ����������������ʱ�����ǽ�������֮ǰֵ
		{
			RX_CONT = 0;
		}   
	}

	if(TI)
	{
		TI = 0;
		B_TX1_Busy = 0;
	}
}