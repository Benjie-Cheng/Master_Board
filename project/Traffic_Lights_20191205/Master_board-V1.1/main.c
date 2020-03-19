/*---------------------------------------------------------------------*/
/* --- Fuctin: Mini traffic lights- -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/
/*************	��������˵��	**************
��ͨ�ƣ�0.28�繲��������ܣ�TXD,RXD GPIO�����Զ��忪����
��Ƭ����STC15W401AS-TSSOP
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
#define  LED8_DATA P1
/*****
sbit	LED8_A = P1^1;
sbit	LED8_B = P1^0;
sbit	LED8_C = P1^5;
sbit	LED8_D = P1^4;
sbit	LED8_E = P1^3;
sbit	LED8_F = P1^2;
sbit	LED8_G = P1^6;
sbit	LED8_DP = P1^7;
*******/
sbit	P_COM0 = P3^6;
sbit	P_COM1 = P3^7;
//sbit	P_COM3 = P1^7;

sbit	LED_L1 = P3^3;
sbit	LED_L2 = P5^5;
sbit	LED_R1 = P3^5;
sbit	LED_R2 = P3^4;
sbit	LED_G1 = P5^4;
sbit	LED_G2 = P3^2;

enum{
	LEDL_RED=0, 
	LEDL_GREEN,
	LEDR_RED, 
	LEDR_GREEN, 
	LEDG_RED,
	LEDG_GREEN,  	
};

	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd
	#define RED_LED     2
	#define YELLOW_LED  1
	#define GREEN_LED   0
	#define CAR_LED     3
	
#define GPIO_CHECK_PIN P30
#define GPIO_OUT_PIN   P31

//========================================================================
/********************** ������ ************************/
BOOL B_1ms;	 //1ms��־
BOOL B_TX1_Busy;  //����æ��־

//��������ʱ�� 1�� ����ر���
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
#define LOW_TIMES 5
/*----------------------------------------------------*/
volatile unsigned int g_count = FRONT_LED_TIME;
volatile unsigned int y_count = RIGHT_LED_TIME;
volatile unsigned int r_count = LEFT_LED_TIME;
BOOL flash_flag = FALSE;
BOOL low_time = FALSE;

static u8 Gu8Step = 0; //������ʱ�� 1 �� switch �л�����

#define	INDEX_MAX 2	//��ʾλ����
static u8 	Display_Code[2]={0x00,0x00};		//1��com;2��LED״̬
static u8 	LED8[2] = {0x00,0x00};		//��ʾ����֧��2λ,ʮλ����λ
static u8	display_index = 0;						//��ʾλ����	

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //���մ���1��������
u8 RX_CONT = 0; 
u8 code t_display[]={						//����
//	 0    1    2    3    4    5    6    7    8    9
	0xC0,0xDE,0xA4,0x8C,0x9A,0x89,0x81,0xDC,0x80,0x88,
//black
	0xff,
//0. 1. 2. 3. 4. 5. 6. 7. 8. 9.	
	0x40,0x5E,0x24,0x0C,0x1A,0x09,0x01,0x5C,0x00
};	


void uart1_config();	// ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
int Get_KeyVal(int val);
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
//	{0, 80, 80, TaskLedRunScan},         		// ������	
//	{0, 50, 50, vKey_Service}					// �����������50ms
//	{0, 10, 10, TaskRTC}				        // RTC����ʱ
//	{0, 30, 30, TaskDispStatus}
	// ���������������
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // ����LED
//	TAST_LED_RUN,        	//������
//	TASK_KEY_SERV,
//	TASK_RTC,
//	TASK_DISP_WS,             // ����״̬��ʾ// ��������������񡣡�����
	// ���������������
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
	P2n_push_pull(0xff);	//����Ϊ׼˫���
	P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���	
	timer0_init();
	uart1_config();
	GPIO_OUT_PIN = 0;//����ͣ���P26���
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
void BitX_Set(int status,u8 Xbit)
{
	u8 val=0;
	if(status==ON)
		Display_Code[0] = setbit(val,Xbit);//�ߵ�ƽ��ͨ
	else if(status==OFF)
		Display_Code[0] = clrbit(val,Xbit);	
	else if(ON_ALL == status)	
		Display_Code[0] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[0] = 0x00;
	else if(OFF_ON == status)
		Display_Code[0] = reversebit(val,Xbit);//��ת
	
}
void TaskDisplayScan(void)//10ms ˢ��һ��
{ 
	if(flash_flag)
	{
		P_COM0 = 0;
		P_COM1 = 0;
		LED_L1 = 1;
		LED_L2 = 1;
		LED_R1 = 1;
		LED_R2 = 1;
		LED_G1 = 1;
		LED_G2 = 1;
		return;
	}
	for(display_index = 0;display_index < INDEX_MAX;display_index++){
		BitX_Set(ON,display_index);//com ��ɨ�裬������λ
#if 0
	_nop_();
	_nop_();
	BitX_Set(OFF,display_index);//�ı�����
#endif	
		//�������ʾ
		P_COM0 = getbit(Display_Code[0],0);
		P_COM1 = getbit(Display_Code[0],1);
		LED8_DATA = LED8[display_index];
		//��ͨ����ʾ
		LED_L1 = getbit(Display_Code[1],LEDL_RED);
		LED_L2 = getbit(Display_Code[1],LEDL_GREEN);
		LED_R1 = getbit(Display_Code[1],LEDR_RED);
		LED_R2 = getbit(Display_Code[1],LEDR_GREEN);
		LED_G1 = getbit(Display_Code[1],LEDG_RED);
		LED_G2 = getbit(Display_Code[1],LEDG_GREEN);
	}
}
void vTaskfFlashLed(void)
{ 
	if(!low_time)
		return;
	flash_flag = ~flash_flag;//0.5s
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
void TaskLedRunScan(void)
{
	static u8 LED_RUN_STEP=0;
	LED_RUN_STEP++;
	switch(LED_RUN_STEP)
	{
	case 1:
		LED8[3] = ~0x01;
		LED8[2] = ~0x00;
		break;
	case 2:
		LED8[3] = ~0x02;
		LED8[2] = ~0x00;
		break;
	case 3:
		LED8[3] = ~0x04;
		LED8[2] = ~0x00;
		break;
	case 4:
		LED8[3] = ~0x08;
		LED8[2] = ~0x00;
		break;
	case 5:
		LED8[3] = ~0x00;
		LED8[2] = ~0x08;
		break;
	case 6:
		LED8[3] = ~0x00;
		LED8[2] = ~0x10;
		break;
	case 7:
		LED8[3] = ~0x00;
		LED8[2] = ~0x20;
		break;
	case 8:
		LED8[3] = ~0x00;
		LED8[2] = ~0x01;
		LED_RUN_STEP = 0;
		break;
	}
}
void Kled_Set(int status,u8 Nled)//LED����
{
	u8 val;
	val = Display_Code[1];
	if(status==ON)
		Display_Code[1] = setbit(val,Nled);//�ߵ�ƽ��ͨ
	else if(status==OFF)
		Display_Code[1] = clrbit(val,Nled);	
	else if(ON_ALL == status)	
		Display_Code[1] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[1] = 0x00;
	else if(OFF_ON == status)
		Display_Code[1] = reversebit(val,Nled);//��ת
}
void Gpio_ValRead(void)	
{	
	static unsigned char Su8KeyLock1; //1 �Ű���������

	if(GPIO_CHECK_PIN==1)
	{
		Su8KeyLock1=0; //��������
		vGu8TimeFlag_2=0;
		vGu16TimeCnt_2=0;

		//Kled_Set(OFF,CAR_LED);//����ֹͣ
	}
	else if(0==Su8KeyLock1)
	{
		vGu8TimeFlag_2=1;//������ʱ��2
		if(vGu16TimeCnt_2>=GPIO_FILTER_TIME) //�˲��ġ� �ȶ�ʱ�䡱 GPIO_FILTER_TIME�� ������ 50ms��
		{
			vGu8TimeFlag_2=0;//�˲�ʱ�䵽����ʱ����0
			Su8KeyLock1=1; //����������,����һֱ����
			//Kled_Set(ON,CAR_LED);//����ֹͣ
			//key_led_on(TRUE);//������ʾled
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
			if(g_count<=LOW_TIMES)
				low_time = TRUE;
			else
				low_time = FALSE;
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
			if(y_count<=LOW_TIMES)
				low_time = TRUE;
			else
				low_time = FALSE;
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
			if(r_count<=LOW_TIMES)
				low_time = TRUE;
			else
				low_time = FALSE;
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
				//Kled_Set(OFF,CAR_LED);//��������
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
	REN = 1;	//��������
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