/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/
/*************	XXX_MC204S_K8_V10	**************
XXX_0008:�ÿ��ư���Ҫ����С��Ŀ���������幦�ܼ�spec

1��TM1650 ������λ����������ܣ�ʵ��4*7 ����ɨ�裬Ԥ�������ڡ�
2��595���ư���LED����ʾ��Ԥ���м����ڡ�
3��PT2272 ң�ع��ܽӿڡ�
4��BT05����uart�ӿڡ�
5������WS2811 �ӿڡ�
6����Դ12V,5V��GND
7��GPIO��չԤ����

����ȱ�ݣ�
1��
2��

v1.0������趨
1��������115200 ������35M          
2��
******************************************/
#include "config.h"
#include "debug.h"

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 �ж�Ƶ��, 1000��/��
#define	Baudrate1	115200UL                                //ͨ�Ų�����115200

	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd
	#define OFF_ON_ALL 0xfc
	#define RED_LED     2
	#define YELLOW_LED  1
	#define GREEN_LED   0
	
	#define RUN_STOP 0xff
	#define RUN_START 0x01
	
	#define KEY_START 0x01
	#define KEY_UP    0x02
	#define KEY_DOWN  0x03
	#define KEY_STOP  0x04
	#define KEY_NULL  0xff
	#define S1        1
	#define S2        2
	#define S3        3
	#define S4        4
	#define S5        5
	#define S6        6
	#define S7        7
	#define S8        8
	#define S9        9
	#define S10       10
	#define S11       11
	#define S12       12
	#define S13       13
	#define S14       14
	#define S15       15
	#define S16       16
	
	
#define KEY_FILTER_TIME 20 //�˲��ġ� �ȶ�ʱ�䡱 20ms

/*
PT2272оƬ���ƹܽ�
#define GPIO_VT_CHECK  P14
#define PT2272_D0      P10
#define PT2272_D1      P11
#define PT2272_D2      P12
#define PT2272_D3      P13
*/
#define PT2272_DATA(x)  x>>0


#define SET_SCL_OUT_TM1650()    //{SCL_TM1650=1; PC_CR1_C17 = 1; PC_CR2_C27 = 0;}
#define SET_SDA_OUT_TM1650()    //{PC_DDR_DDR6=1; PC_CR1_C16 = 1; PC_CR2_C26 = 0;}
#define SET_SDA_IN_TM1650()     //{PC_DDR_DDR6=0; PC_CR1_C16 = 0; PC_CR2_C26 = 0;}


BOOL LED_OFF_All_Flag = FALSE;

BOOL update_flag = FALSE;
BOOL Key_EventProtect = FALSE;
//-----------------------------------------------------------------------------------------------------------------
//�������ʱ�� 1�� ����ر���
volatile unsigned char vGu8TimeFlag_1=0;//20���Ӽ���
volatile u32 vGu32TimeCnt_1=0;
volatile unsigned char vGu8TimeFlag_2=0;//5���Ӽ���
volatile u32 vGu32TimeCnt_2=0;
volatile unsigned char vGu8TimeFlag_3=0;//5���Ӽ���
volatile u32 vGu32TimeCnt_3=0;
unsigned int  uiKeyTimeCnt=0; //����ȥ������ʱ������ //����ȥ������ʱ������


//========================================================================
/*************	���ر�������	**************/
BOOL B_1ms;	 		//1ms��־
BOOL B_TX1_Busy;  	//����æ��־

static u8 Gu8Step = 0;				  //switch �л�����
static u8 ucKeyStep = 0;
volatile unsigned char Key_Code=0xff;//��ʱ���ж���ʹ��
#define DISP_LENGTH 2
static u8 	Display_Code[DISP_LENGTH]={0x00,0x00};		   //�����ƣ�L_MOS��H_MOS��
#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //���մ���1��������
u8 RX_CONT = 0; 
u8 code TM1650_CODE[]={						//TM1650��ʾ����,��������׼�ֿ⣬����ȡ��
//	 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,
//black	 -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e,
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	//0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1
u8 code pt2272[] = {//PT2272��������
//   1     2    3    4    5    6    7    8   9    10   11   12   13   14   15
	0x01,0x02,0x04,0x0a,0x0b,0x06,0x07,0x0c,0x09,0x03,0x0e,0x0d,0x0f,0x08,0x05	
};
u8 code KeyVal[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};

/********************** A ��ָʾ���õ�595 ************************/
sbit	A_HC595_SER   = P5^5;	//pin 55	SER		data input
sbit	A_HC595_RCLK  = P1^5;	//pin 15	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P5^4;	//pin 54	SRCLK	Shift data clock
//sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		�͵�ƽ ʹ��enable pin
sbit	A_HC595_MR    = P3^2;	//pin 32	�͵�ƽ��λ	

void uart1_config();	// ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void TaskDisplayScan(void);
void vTaskfFlashLed(void);
void Date_EventProcess(void);

typedef struct _TASK_COMPONENTS
{
	u8 Run;                  //�������б�ǣ�0-�����У�1����
	u16 Timer;               // ��ʱ��
	u16 ItvTime;             // �������м��ʱ��
	void (*TaskHook)(void); // Ҫ���е�������
} TASK_COMPONENTS;   
static TASK_COMPONENTS TaskComps[] =
{
	{0, 1000,  1000, vTaskfFlashLed},           // led��˸1s
	{0, 10, 10, TaskDisplayScan},         		// 595����ˢ��10msһ��
//	{0, 50, 50, vKey_Service}					// �����������50ms
//	{0, 10, 10, TaskRTC}				        // RTC����ʱ
//	{0, 30, 30, TaskDispStatus}
	// ��������������
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // ����LED
	TAST_595_UPDATE,        // ��ʾʱ��
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
	P2n_standard(0xff);	//����Ϊ׼˫���
	P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���
	A_HC595_MR = 0;     //�������������	
	timer0_init();
	uart1_config();
	A_HC595_MR = 1;     //��λ��ֹ
	//A_HC595_OE = 0;   //ʹ��оƬ,������Ч
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
	for (i=0; i<TASKS_MAX; i++)            // �������ʱ�䴦��
	{
		if (TaskComps[i].Run)              // ʱ�䲻Ϊ0
		{
			TaskComps[i].TaskHook();       // ��������
			TaskComps[i].Run = 0;          // ��־��0
		}
	}
}
u16 Bit8_ToBit16(u8 high,u8 low)
{
	u16 val;
	val = (high << 8) | low;;
	return val;
}
u8 Bit16_ToBit8(u16 num,u8 high)
{
	u16 val;
	if(high)
		val = (num >> 8) & 0xff;//��8λ
	else
		val = num & 0xff; //��8λ
	return val;
}
void Kled_Set(int status,u8 Kled)
{
	u16 val;
	val = Bit8_ToBit16(Display_Code[1],Display_Code[0]);//H,L
	if(status==ON)
		setbit(val,Kled);//�ߵ�ƽ��ͨ������͵�ƽ��ͨ��ȡ�����������
	else if(status==OFF)
		clrbit(val,Kled);	
	else if(ON_ALL == status)	
		val = 0xffff;
	else if(OFF_ALL == status)
		val= 0x0000;
	else if(OFF_ON_ALL == status)
		val= ~val;
	else if(OFF_ON == status)
		reversebit(val,Kled);//��ת
	Display_Code[1] = Bit16_ToBit8(val,1);//��8λ
	Display_Code[0] = Bit16_ToBit8(val,0);//��8λ
}
void Nled_Set(int status,u8 Nled)
{
	u16 val;
	val = Bit8_ToBit16(Display_Code[1],Display_Code[0]);//H,L
	if(status==ON)
		setbit(val,Nled);//�ߵ�ƽ��ͨ
	else if(status==OFF)
		clrbit(val,Nled);	
	else if(ON_ALL == status)	
		val = 0xffff;
	else if(OFF_ALL == status)
		val= 0x0000;
	else if(OFF_ON == status)
		reversebit(val,Nled);//��ת
	Display_Code[1] = Bit16_ToBit8(val,1);//��8λ
	Display_Code[0] = Bit16_ToBit8(val,0);//��8λ
}
int Get_Pt2272State(void)
{
	u8 i = 0,val;
	for(i=0;i<15;i++)
	{
		if(PT2272_DATA(P1)==pt2272[i]) 
		{
			val = i+1;
			break;
		}
		else 
			val = 0;
	}
	return val;
}
void KeyTask(void)
{
	u8 led_bit=0;
	if(Key_Code==0)
	return;
	//print_char(Key_Code);
	led_bit = Key_Code;
	led_bit--;
	Gu8Step = 0;
	//print_char(led_bit);
	switch(Key_Code)
	{
		case S1:
		case S2:
		case S3:
		case S4:
		case S5:
		case S6:
		case S7:
		case S8:
			Kled_Set(OFF_ON,led_bit);
			break;
		case S9:
		case S10:
		case S11:
		case S12:
		case S13:
		case S14:
		case S15:
			Kled_Set(ON,led_bit);//��λ��������һ���Ӽ�ʱ
			break;
		default : 
		
			break;
	}	
	Key_Code = 0;//����ֵ��0����ֹ��������
	Key_EventProtect = FALSE;//����������������Խ��ܰ���ֵ��
}
void Date_EventProcess(void)
{

}
void State_EventProcess(void)
{
	switch(Gu8Step)
	{
		case 0:
			break;
		case 1:
			break;			
		case 2:
			break;			
		default : 
		
			break;			
	}
}
void KeyScan(void)
{
	static unsigned char Su8KeyLock5;
	static unsigned int  Su16KeyCnt5;

	if(Key_EventProtect)
		return;//����а������£���ִ��ң�ؼ�⡣
	
	if(!(P1&0x0f))//�����Ϊ0��δ����
	{ 
	
		Su8KeyLock5=0;
		Su16KeyCnt5=0;

	}
	else if(0==Su8KeyLock5)
	{
		Su16KeyCnt5++;
		if(Su16KeyCnt5>=KEY_FILTER_TIME)
		{
			
			Su8KeyLock5=1; 
			//key_led_on(1);			
			Key_EventProtect = TRUE;
			Key_Code = Get_Pt2272State();	
		}
	}
	
}
//------------------------------------------------------------
void TaskDisplayScan(void)//595����ˢ��10msһ��
{ 
	if(LED_OFF_All_Flag)//���ȫ��
	{
		Display_Code[0] = 0x00;
		vDataIn595(~Display_Code[0]);//�������ָʾ�ƹ�
		vDataOut595();
		return;
	}
	vDataIn595(~Display_Code[0]);//�������ָʾ�Ƶ�8λ״̬���͵�ƽ����
	vDataOut595();				//�����������
}
void vTaskfFlashLed(void)//1s һ��
{ 
	Kled_Set(OFF_ON,7);//���λLED����
	//print_char(GPIO_OUT_ROW);
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
	vDataIn595(0xff);
	vDataOut595();	//����Ĭ�Ϲر�ͨ����ʾLED
	puts_to_SerialPort("I am LED controller XXX_MC204S_K8_V10!\n");
	puts_to_SerialPort("Contact me: 15901856750\n");
	key_led_on(0);
	Init_Tm1650()//����ܿ���ʾ
	while (1)
	{	
		//Led_StateUpdate();//LED״̬����
		KeyTask();
		Date_EventProcess();
		State_EventProcess();
		TaskProcess();//595���º���˸led����ִ��		 
	}
}

//========================================================================
// ����: Timer0 1ms�жϺ�����
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	B_1ms = 1;		//1ms��־
	KeyScan();
	TaskRemarks();
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