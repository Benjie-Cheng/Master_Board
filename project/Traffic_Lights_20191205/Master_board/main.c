/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/
/*************	��������˵��	**************
8K8R:8��key����8��Relay�����幦�ܼ�spec

��STC��MCU��IO��ʽ������̽����ʹ����շ����ݡ�
1��8*8 �������ʱֻ���Բ�����һ�С�
2���������а�������ʱ�����ڷ���K1-K8�������Ƶ�led ״̬��������������ʱ�޷���״̬���ӻ�������ʱ״̬����ң����Ǵ��°�һ���ָܻ�����
3���ӻ����ݰ���ֵ��ͨ��74HC595���ư���LED״̬��
4���а�������ʱ��һ�Ź��õ�LEDָʾ���������𣬲���ʱ����1Hz��˸��

����ȱ�ݣ�
1��8*8���̵�LED���ڰ����Ҳ࣬ÿ�ΰ��¾�����ָ�ڵ������Էֱ�̵���״̬�������Ҳ�����ϲࡣ
2��LED������Ϊ�ߵ�ƽ��������������������ס���ɲ����ⲿ�������͵�ƽ������ʽ

v1.1��
1���޸Ĳ�����115200->9600           20190929
2��ͳһͨ�����ݸ�ʽ                 20190929
******************************************/
#include "config.h"
#include "debug.h"

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 �ж�Ƶ��, 1000��/��
#define	Baudrate1	115200UL                                //ͨ�Ų�����115200

#define KEY_LED_GPIO P55

	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd
	#define RED_LED     2
	#define YELLOW_LED  1
	#define GREEN_LED   0
	#define CAR_LED     3
	
#define GPIO_CHECK_PIN P26
#define GPIO_OUT_PIN   P10

//========================================================================
/*************	���ر�������	**************/
/********************** ������ ************************/
BOOL B_1ms;	 //1ms��־
u16	msecond; //1s����
u8 cnt50ms;  //50ms����
u8 cnt10ms;  //10ms����
BOOL B_TX1_Busy;  //����æ��־

//�������ʱ�� 1�� ����ر���
volatile unsigned char vGu8TimeFlag_1=0;
volatile u32 vGu32TimeCnt_1=0;	
volatile unsigned char vGu8TimeFlag_2=0;
volatile unsigned int vGu16TimeCnt_2=0;	
volatile unsigned char vGu8LeveCheckFlag=0;

#define G_LED_TIME 59
#define Y_LED_TIME 4
#define R_LED_TIME 29	
volatile unsigned int g_count = G_LED_TIME;
volatile unsigned int y_count = Y_LED_TIME;
volatile unsigned int r_count = R_LED_TIME;
BOOL flash_flag = TRUE;

#define GPIO_FILTER_TIME 50 //�˲��ġ� �ȶ�ʱ�䡱 50ms

#define LED_TIME_1S  1000  //ʱ���� 10000ms
#define LED_TIME_60S 60000 //ʱ���� 60000ms
#define LED_TIME_65S 65000 //ʱ���� 65000ms
#define LED_TIME_95S 95000 //ʱ���� 95000ms

static u8 Gu8Step = 0; //�����ʱ�� 1 �� switch �л�����
#define	INDEX_MAX 2	//��ʾλ����
static u8 	Display_Code[3]={0x00,0x00,0x00};		//1��595���ư�����LED��,����595����ܵ���ʱ��
static u8 	LED8[4] = {0x00,0x00,0x00,0x00};		//��ʾ����֧����λ
static u8	display_index = 0;						//��ʾλ����	

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //���մ���1��������
u8 RX_CONT = 0; 
u8 code t_display[]={						//��������׼�ֿ⣬����ȡ��
//	 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,
//black	 -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e,
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	//0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1


/********************** A ��ָʾ���õ�595 ************************/
sbit	A_HC595_SER   = P3^4;	//pin 34	SER		data input
sbit	A_HC595_RCLK  = P3^7;	//pin 37	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P3^5;	//pin 35	SRCLK	Shift data clock
sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		�͵�ƽ ʹ��enable pin
sbit	A_HC595_MR    = P3^6;	//pin 36	�͵�ƽ��λ	

void uart1_config();	// ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
int Get_KeyVal(int val);
void TaskDisplayScan(void);
void vTaskfFlashLed(void);

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
//	{0, 10, 10, TaskDisplayScan},         		// ��ʾʱ��,LED 10msˢ��	
//	{0, 50, 50, vKey_Service}					// �����������50ms
//	{0, 10, 10, TaskRTC}				        // RTC����ʱ
//	{0, 30, 30, TaskDispStatus}
	// ��������������
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // ����LED
//	TAST_DISP_TIME,        // ��ʾʱ��
//	TASK_KEY_SERV,
//	TASK_RTC,
//	TASK_DISP_WS,             // ����״̬��ʾ// �������������񡣡�����
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
	P2n_push_pull(0xff);	//����Ϊ׼˫���
	P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���	
	timer0_init();
	uart1_config();
	GPIO_OUT_PIN = 0;//����ͣ���P26���
	A_HC595_MR = 1;//��λ��ֹ
	A_HC595_OE = 0;//ʹ��оƬ
}
/**************** ��HC595����һ���ֽں��� ******************/
void vDataIn595(u8 dat)
{		
	u8	i;
	for(i=0; i<8; i++)
	{
		dat <<= 1;
		A_HC595_SER   = CY;
		A_HC595_SRCLK = 0;
		NOP2();
		A_HC595_SRCLK = 1;
	}
}
/**************** HC595�������溯�� ******************/
void vDataOut595()
{		
	A_HC595_RCLK = 0;
	NOP2();
	A_HC595_RCLK = 1;
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
/*****************************************************
	���м�ɨ�����
	ʹ��XY����8x8���ķ�����ֻ�ܵ������ٶȿ�

    Y  P20   P21   P22   P23   P24   P25   P26   P27
        |     |     |     |     |     |     |     |
X       |     |     |     |     |     |     |     |
P10 -- K00 - K01 - K02 - K03 - K04 - K05 - K06 - K07
        |     |     |     |     |     |     |     |
P11 -- K08 - K09 - K10 - K11 - K12 - K13 - K14 - K15
        |     |     |     |     |     |     |     |
P12 -- K16 - K17 - K18 - K19 - K20 - K21 - K22 - K23
        |     |     |     |     |     |     |     |
P13 -- K24 - K25 - K26 - K27 - K28 - K29 - K30 - K31
        |     |     |     |     |     |     |     |
P14 -- K32 - K33 - K34 - K35 - K36 - K37 - K38 - K39
        |     |     |     |     |     |     |     |
P15 -- K40 - K41 - K42 - K43 - K44 - K45 - K46 - K47
        |     |     |     |     |     |     |     |
P16 -- K48 - K49 - K50 - K51 - K52 - K53 - K54 - K55
        |     |     |     |     |     |     |     |
P17 -- K56 - K57 - K58 - K59 - K60 - K61 - K62 - K63
******************************************************/
void gpio_key_delay(void)
{
	u8 i;
	i = 60;
	while(--i)	;
}
void BitX_Set(int status,u8 Xbit)
{
/*
	u8 val;
	Display_Code[2] = 0x00;//������λ����0������λ��ÿ��ֻ����һλ
	val = Display_Code[2];
	if(En)
		Display_Code[2] = setbit(val,Xbit);//�������ߵ�ƽ��ͨ������	
*/
	u8 val;
	//val = Display_Code[2];
	val =0x00;
	if(status==ON)
		Display_Code[2] = setbit(val,Xbit);//�ߵ�ƽ��ͨ
	else if(status==OFF)
		Display_Code[2] = clrbit(val,Xbit);	
	else if(ON_ALL == status)	
		Display_Code[2] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[2] = 0x00;
	else if(OFF_ON == status)
		Display_Code[2] = reversebit(val,Xbit);//��ת
}
void TaskDisplayScan(void)//10ms ˢ��һ��
{ 
	for(display_index = 0;display_index < INDEX_MAX;display_index++){
		//DisplayChange();//��ֵ������ֵ�����ı������comλ
		BitX_Set(ON,display_index);//com ��ɨ�裬������λ
#if 0
	_nop_();
	_nop_();
	BitX_Set(OFF,display_index);//�ı�����
#endif		
		Display_Code[1] = LED8[display_index];
		//Display_Code[1] = LED8[1];
										//���Լ���
		//delay_ms(254);
		//delay_ms(254);
		vDataIn595(Display_Code[2]);//���λ��//������Ӱ
		vDataIn595(Display_Code[1]);//�������
		vDataIn595(Display_Code[0]);//������̵�״̬
		vDataOut595();				//�����������
	}
}
void vTaskfFlashLed(void)
{ 
	if(flash_flag)
		key_led_reverse();
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
void Kled_Set(int status,u8 Nled)
{
	u8 val;
	val = Display_Code[0];
	if(status==ON)
		Display_Code[0] = setbit(val,Nled);//�ߵ�ƽ��ͨ
	else if(status==OFF)
		Display_Code[0] = clrbit(val,Nled);	
	else if(ON_ALL == status)	
		Display_Code[0] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[0] = 0x00;
	else if(OFF_ON == status)
		Display_Code[0] = reversebit(val,Nled);//��ת
}
void Gpio_ValRead(void)	
{	
	static unsigned char Su8KeyLock1; //1 �Ű���������

	if(GPIO_CHECK_PIN==1)
	{
		Su8KeyLock1=0; //��������
		vGu8TimeFlag_2=0;
		vGu16TimeCnt_2=0;
		flash_flag = TRUE;
		Kled_Set(OFF,CAR_LED);//����ֹͣ
	}
	else if(0==Su8KeyLock1)
	{
		vGu8TimeFlag_2=1;//������ʱ��2
		if(vGu16TimeCnt_2>=GPIO_FILTER_TIME) //�˲��ġ� �ȶ�ʱ�䡱 GPIO_FILTER_TIME�� ������ 50ms��
		{
			vGu8TimeFlag_2=0;//�˲�ʱ�䵽����ʱ����0
			Su8KeyLock1=1; //����������,����һֱ����
			Kled_Set(ON,CAR_LED);//����ֹͣ
			key_led_on(TRUE);//������ʾled
			flash_flag = FALSE;
		}		
	}
}	
void Traffic_Led(void)
{
	switch(Gu8Step)
	{
		case 0:
			vGu8TimeFlag_1 = 1;
			Kled_Set(ON,GREEN_LED);//�̵���
			Kled_Set(OFF,RED_LED);//�����
			vGu8LeveCheckFlag = 0;//�رյ�ƽ���
			r_count = R_LED_TIME;
			LED8[0] = ~t_display[mod(g_count,10)];
			LED8[1] = ~t_display[rem(g_count,10)];
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				g_count--;
				vGu32TimeCnt_1 = 0;
				if(g_count == 0)
					Gu8Step++;
			}	
			break;
		case 1:
			g_count = G_LED_TIME;
			Kled_Set(OFF,GREEN_LED);//�̵���
			Kled_Set(ON,YELLOW_LED);//�Ƶ���
			LED8[0] = ~t_display[mod(y_count,10)];
			LED8[1] = ~t_display[rem(y_count,10)];	
			vGu8LeveCheckFlag = 1;//������ƽ���
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				y_count--;	
				vGu32TimeCnt_1 = 0;
				if(y_count == 0)
					Gu8Step++;
			}
			break;
		case 2:
			y_count = Y_LED_TIME;
			Kled_Set(OFF,GREEN_LED);//�̵���
			Kled_Set(OFF,YELLOW_LED);//�Ƶ���
			Kled_Set(ON,RED_LED);//�����
			LED8[0] = ~t_display[mod(r_count,10)];
			LED8[1] = ~t_display[rem(r_count,10)];	
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
	vDataIn595(0x00);
	vDataOut595();	//����Ĭ�Ϲر�ͨ����ʾLED
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
				Kled_Set(OFF,CAR_LED);//��������
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