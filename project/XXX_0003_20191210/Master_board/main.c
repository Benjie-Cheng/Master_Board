/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/
/*************	��������˵��	**************
8K8R:4��key����12·��16�ֹ��ܣ����幦�ܼ�spec

��STC��MCU��IO��ʽ������̽����ʹ����շ����ݡ�
1����ʼ���»������е�һֱ����ֹͣ��������һ�����������ָʾ�������
2��ģʽ+��ģʽ- ����LED����������ģʽ��Ӧ+��-��
3��PT2272 ����1��ʼ��2+��3-��4��ͣ
4��

����ȱ�ݣ�
1��
2��

v1.1��
1��������115200                     20191210
2��
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
	
	#define RUN_STOP 0xff
	#define RUN_START 0x01
	
	#define KEY_START 0x01
	#define KEY_UP    0x02
	#define KEY_DOWN  0x03
	#define KEY_STOP  0x04
	
#define KEY_FILTER_TIME 20 //�˲��ġ� �ȶ�ʱ�䡱 20ms

#define KEY_INPUT1 P23  //������������K1������ڡ�
#define KEY_INPUT2 P20  //��ֹͣ������K2������ڡ�
#define KEY_INPUT3 P22  //��ģʽ+������K3������ڡ�
#define KEY_INPUT4 P21  //��ģʽ-������K4������ڡ�
#define GPIO_OUT_PIN   P10
#define GPIO_VT_CHECK  P13
/*
#define PT2272_D0      P14
#define PT2272_D1      P15
#define PT2272_D2      P16
#define PT2272_D3      P17
*/
#define PT2272_DATA(x)  x>>4
BOOL update_flag = TRUE;
BOOL Key_EventProtect = FALSE;

#define LED_TIME_60S 60000 //ʱ���� 60000ms
#define LED_TIME_65S 65000 //ʱ���� 65000ms
#define LED_TIME_95S 95000 //ʱ���� 95000ms

//========================================================================
/*************	���ر�������	**************/
BOOL B_1ms;	 		//1ms��־
BOOL B_TX1_Busy;  	//����æ��־

volatile unsigned char vGu8KeySec=0;  //�����Ĵ������
static u8 Gu8Step = 1;				  //switch �л�����
int Key_Code=0xff;
static u8 Key_CodeOld = 0x00 ;
#define	INDEX_MAX 2	//��ʾλ����
static u8 	Display_Code[3]={0x00,0x00,0x00};		//�����ƣ�L_MOS��H_MOS��
//static u8 	LED8[4] = {0x00,0x00,0x00,0x00};		//��ʾ����֧����λ
//static u8	display_index = 0;						//��ʾλ����	

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //���մ���1��������
u8 RX_CONT = 0; 
u8 code t_display[]={						//��������׼�ֿ⣬����ȡ��
//	 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,
//black	 -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e,
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	//0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1
u8 code pt2272[] = {
//   1     2    3    4    5    6    7    8   9    10   11   12   13   14   15
	0x01,0x02,0x04,0x0a,0x0b,0x06,0x07,0x0c,0x09,0x03,0x0e,0x0d,0x0f,0x08,0x05	
};

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
	P2n_push_pull(0xff);	//����Ϊ׼˫���
	P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���	
	timer0_init();
	uart1_config();
	GPIO_OUT_PIN = 0;//����ͣ��ð������
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
	u8 val;
	val = Display_Code[0];
	if(status==ON)
		setbit(val,Kled);//�ߵ�ƽ��ͨ
	else if(status==OFF)
		clrbit(val,Kled);	
	else if(ON_ALL == status)	
		val = 0xff;
	else if(OFF_ALL == status)
		val= 0x00;
	else if(OFF_ON == status)
		reversebit(val,Kled);//��ת
	Display_Code[0] = val;
}
void Nled_Set(int status,u8 Nled)
{
	u16 val;
	val = Bit8_ToBit16(Display_Code[2],Display_Code[1]);//H,L
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
	Display_Code[2] = Bit16_ToBit8(val,1);//��8λ
	Display_Code[1] = Bit16_ToBit8(val,0);//��8λ
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
void Transsion_Keycode(int key_code)
{
	if(key_code==Key_CodeOld)
		return;
	switch(key_code)
	{
		case KEY_START:
		vGu8KeySec=1;    //����������
		break;
		case KEY_UP:
		vGu8KeySec=3;    //����+�ż�
		Kled_Set(ON,2);//����+����	
		break;
		case KEY_DOWN:
		vGu8KeySec=4;    //����-�ż�
		Kled_Set(ON,1);//����-����	
		break;
		case KEY_STOP:
		vGu8KeySec=2;    //����4�ż�
		Kled_Set(ON,0);//��ͣ������
		break;
		default : 
		vGu8KeySec=0;    //����4�ż�
		Key_EventProtect = FALSE;
		break;
		key_code = 0;
		Key_CodeOld = key_code;
	}
	
}
void KeyScan(void)
{
	static unsigned char Su8KeyLock1;
	static unsigned int  Su16KeyCnt1;
	static unsigned char Su8KeyLock2;
	static unsigned int  Su16KeyCnt2; 	
	static unsigned char Su8KeyLock3;
	static unsigned int  Su16KeyCnt3; 
	static unsigned char Su8KeyLock4;
	static unsigned int  Su16KeyCnt4; 
	static unsigned char Su8KeyLock5;
	static unsigned int  Su16KeyCnt5;
	
	if(Key_EventProtect)
		return;
    //������������K1��ɨ��ʶ��
	if(0!=KEY_INPUT1)
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
   //��ֹͣ������K2��ɨ��ʶ��
	if(0!=KEY_INPUT2)
	{
		Su8KeyLock2=0;
		Su16KeyCnt2=0; 
		if(!Su8KeyLock5)
			Kled_Set(OFF,0);//��������			
	}
	else if(0==Su8KeyLock2)
	{
		Su16KeyCnt2++;
		if(Su16KeyCnt2>=KEY_FILTER_TIME)
		{
			Su8KeyLock2=1;  
			vGu8KeySec=2;    //����2�ż�
			Key_EventProtect = TRUE;
			Kled_Set(ON,0);//��������
		}
	}
   //��ģʽ+������K3��ɨ��ʶ��
	if(0!=KEY_INPUT3)
	{
		Su8KeyLock3=0;
		Su16KeyCnt3=0; 
		if(!Su8KeyLock5)
			Kled_Set(OFF,2);//��������		
	}
	else if(0==Su8KeyLock3)
	{
		Su16KeyCnt3++;
		if(Su16KeyCnt3>=KEY_FILTER_TIME)
		{
			Su8KeyLock3=1;  
			vGu8KeySec=3;    //����3�ż�
			Key_EventProtect = TRUE;
			Kled_Set(ON,2);//��������
		}
	}
   //��ģʽ-������K4��ɨ��ʶ��
	if(0!=KEY_INPUT4)
	{
		Su8KeyLock4=0;
		Su16KeyCnt4=0; 
		if(!Su8KeyLock5)
			Kled_Set(OFF,1);//��������		
	}
	else if(0==Su8KeyLock4)
	{
		Su16KeyCnt4++;
		if(Su16KeyCnt4>=KEY_FILTER_TIME)
		{
			Su8KeyLock4=1;  
			vGu8KeySec=4;    //����4�ż�
			Key_EventProtect = TRUE;
			Kled_Set(ON,1);//��������
		}
	}
	
	if(Key_EventProtect)
		return;//����а������£���ִ��ң�ؼ�⡣
	
	if(!(P1&0xf0))//�����Ϊ0��δ����
	{ 
	
		Su8KeyLock5=0;
		Su16KeyCnt5=0;
		if(!Su8KeyLock2)
			Kled_Set(OFF,0);//��ͣ������
		else if(!Su8KeyLock3)
			Kled_Set(OFF,2);//����+����	
		else if(!Su8KeyLock4)
			Kled_Set(OFF,1);//����-����	
	}
	else if(0==Su8KeyLock5)
	{
		Su16KeyCnt5++;
		if(Su16KeyCnt5>=KEY_FILTER_TIME)
		{
			
			Su8KeyLock5=1;  
			Key_EventProtect = TRUE;
			Key_Code = Get_Pt2272State();
			Transsion_Keycode(Key_Code);	
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
	if((Gu8Step == RUN_STOP)&&(vGu8KeySec != 1))
	{
		vGu8KeySec = 0;
		Key_EventProtect = FALSE;
		return;//�����ֹͣ�����Ҳ���������,��ִ��
	}
	switch(vGu8KeySec) //���ݲ�ͬ�İ����������ִ�ж�Ӧ�Ĵ���
	{
		case 1:     //1�Ű�����������������K1
			Gu8Step = RUN_START;
			Kled_Set(ON,3);//��������
			vGu8KeySec=0; 
		break;
		case 2:     //2�Ű�������ֹͣ������K2
			Gu8Step = RUN_STOP;
			Kled_Set(OFF,1);//��������
			Kled_Set(OFF,2);//��������
			Kled_Set(OFF,3);//��������
			vGu8KeySec=0;  
		break;
		case 3:     //3�Ű�������ģʽ+������K3
			Gu8Step++;
			vGu8KeySec=0;  
		break;
		case 4:     //4�Ű�������ģʽ-������K4
			Gu8Step--;
			vGu8KeySec=0; 
		break;
		default:     
			 
		break;
	}
	if(Gu8Step >16 && (Gu8Step != RUN_STOP))
		Gu8Step = 1;
	else if(Gu8Step < 1)
		Gu8Step = 16;
	update_flag = TRUE;
	Key_EventProtect = FALSE;
}	
void Led_StateUpdate(void)
{
	if(!update_flag)
		return ;
	Nled_Set(OFF_ALL,1);//�����ı�ʱ������0һ��;
	switch(Gu8Step)
	{
		case 1:
			Kled_Set(ON,3);//��������
			Nled_Set(ON,0);
			Nled_Set(ON,2);
		break;
		case 2:
			Nled_Set(ON,4);
		break;
		case 3:
			Nled_Set(ON,1);
			Nled_Set(ON,3);
		break;	
		case 4:
			Nled_Set(ON,5);
		break;
		case 5:
			Nled_Set(ON,0);
			Nled_Set(ON,2);
		break;
		case 6:
			Nled_Set(ON,6);
		break;
		case 7:
			Nled_Set(ON,1);
			Nled_Set(ON,3);
		break;
		case 8:
			Nled_Set(ON,5);
		break;
		case 9:
			Nled_Set(ON,0);
			Nled_Set(ON,7);
		break;
		case 10:
			Nled_Set(ON,9);
		break;
		case 11:
			Nled_Set(ON,1);
			Nled_Set(ON,8);
		break;
		case 12:
			Nled_Set(ON,10);
		break;
		case 13:
			Nled_Set(ON,0);
			Nled_Set(ON,7);
		break;
		case 14:
			Nled_Set(ON,11);
		break;
		case 15:
			Nled_Set(ON,0);
			Nled_Set(ON,8);
		break;
		case 16:
			Nled_Set(ON,10);
		break;
		
		default:
		break;
		update_flag = FALSE;//���º�ʱ��λ0;
	}
}

void TaskDisplayScan(void)//10ms ˢ��һ��
{ 
	vDataIn595(Display_Code[2]);//��8λMOS�ܵ�״̬
	vDataIn595(Display_Code[1]);//��8λMOS�ܵ�״̬
	vDataIn595(Display_Code[0]);//�������ָʾ��״̬
	vDataOut595();				//�����������
}
void vTaskfFlashLed(void)
{ 
	key_led_reverse();
	//print_char(Get_Pt2272State());
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
	puts_to_SerialPort("I am LED controller XXX_0003_20191210!\n");
	puts_to_SerialPort("Contact me: 15901856750\n");
	Kled_Set(ON,3);//��������
	while (1)
	{	
		KeyTask();    //������������
		Led_StateUpdate();//LED״̬����
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