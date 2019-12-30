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

	
	
//��������
#define	KEY1_GPIO P12

//========================================================================
/*************	���ر�������	**************/
/********************** ������ ************************/
BOOL B_1ms;	 //1ms��־
u16	msecond; //1s����
u8 cnt50ms;  //50ms����
u8 cnt10ms;  //10ms����
BOOL B_TX1_Busy;  //����æ��־

#define     E2PROM_LENGTH 2
static u8 	E2PROM_Strings[E2PROM_LENGTH] = {0x02,0x03};//data0��ͨʱ�䣬data1����ʱ��
static u8	tmp[E2PROM_LENGTH] = {0x01,0x01};

BOOL flash_flag = TRUE;//��˸���������б�־
BOOL update_flag = FALSE;//EEPROM ���±�־

#define KEY_FILTER_TIME 20 //�˲��ġ� �ȶ�ʱ�䡱 20ms


#define	INDEX_MAX 4	//��ʾλ����
static u8 	Display_Code[2]={0x00,0x00};		//����ܶ����λ�룬�̵�����led״̬��
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
sbit	A_HC595_SER   = P1^1;	//pin 34	SER		data input
sbit	A_HC595_RCLK  = P3^2;	//pin 37	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P1^0;	//pin 35	SRCLK	Shift data clock
//sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		�͵�ƽ ʹ��enable pin
//sbit	A_HC595_MR    = P3^6;	//pin 36	�͵�ƽ��λ	

void uart1_config();	// ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void KeyScan(void);
void KeyTask(void);
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
	//A_HC595_MR = 1;//��λ��ֹ
	//A_HC595_OE = 0;//ʹ��оƬ
	KEY1_GPIO = 1;//�������ţ�Ĭ�����ߵ�ƽ
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
void BitX_Set(int status,u8 Xbit)
{
	u8 val;
	val = Display_Code[1]&0xf0;//����λÿ�����㣬���������λɨ��
	if(status==ON)
		Display_Code[1] = setbit(val,Xbit);//�ߵ�ƽ��ͨ
	else if(status==OFF)
		Display_Code[1] = clrbit(val,Xbit);	
	else if(ON_ALL == status)	
		Display_Code[1] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[1] = 0x00;
	else if(OFF_ON == status)
		Display_Code[1] = reversebit(val,Xbit);//��ת
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
		Display_Code[0] = LED8[display_index];

		vDataIn595(Display_Code[1]);//���λ��
		vDataIn595(Display_Code[0]);//�������
		vDataOut595();				//�����������
	}	
}
void vTaskfFlashLed(void)
{ 
	if(flash_flag)
		key_led_reverse();
	puts_to_SerialPort(tmp);
	
}

void KeyScan(void)
{	
	static unsigned char Su8KeyLock1;
	static unsigned int  Su16KeyCnt1;
	if(Key_EventProtect)
		return;
    //������������K1��ɨ��ʶ��
	if(0!=KEY1_GPIO)
	{
		Su8KeyLock1=0;
		Su16KeyCnt1=0;   
	}
	else if(0==Su8KeyLock1)
	{
		Su16KeyCnt1++;
		if(Su16KeyCnt1>=KEY_FILTER_TIME)
		{
			Su8KeyLock1=1;  
			vGu8KeySec=1;    //����1�ż�
			Key_EventProtect = TRUE;
		}
	}
}	
void KeyTask(void)
{
	//print_char(vGu8KeySec);
	if(0==vGu8KeySec)
	{
		return; //�����Ĵ��������0��ζ���ް�����������ִ�д˺�������Ĵ���
	}
	switch(vGu8KeySec) //���ݲ�ͬ�İ����������ִ�ж�Ӧ�Ĵ���
	{
		case 1:     //����K1
			vGu8KeySec=0; 
			break;
		case 2:     //����K2
			vGu8KeySec=0;  
			break;
		case 3:     //����K3
			vGu8KeySec=0;  
			break;
		case 4:     //����K4
			vGu8KeySec=0; 
			break;
		case 5:     //����K5
			vGu8KeySec=0; 
			update_flag = TRUE;
			break;
		default:     
			 
			break;
	}

	Key_EventProtect = FALSE;
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
	u16 i;
	stc15x_hw_init();
	vDataIn595(0x00);
	vDataOut595();	//����Ĭ�Ϲر�ͨ����ʾLED
	puts_to_SerialPort("I am Traffic Lights!\n");
	while(1)
	{
		if(B_1ms)	//1ms��
		{
			B_1ms = 0;	
			KeyTask();    //������������
			TaskProcess();//����������led��˸
			TaskDisplayScan();
		}
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