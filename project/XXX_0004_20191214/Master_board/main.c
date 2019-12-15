/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/
/*************	��������˵��	**************
XXX_0004:PT2272+����������ʱ���ܵĿ����������幦�ܼ�spec

��STC��MCU��IO��ʽ������̽����ʹ����շ����ݡ�
1��
2��
3��PT2272 ����1��ʼ
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

#define KEY_INPUT1 P23  //������������K1������ڡ�
#define KEY_INPUT2 P20  //��ֹͣ������K2������ڡ�
#define KEY_INPUT3 P22  //��ģʽ+������K3������ڡ�
#define KEY_INPUT4 P21  //��ģʽ-������K4������ڡ�

#define GPIO_OUT_ROW    P2   //����������С�
#define GPIO_OUT_LIN1   P10  //��������̵�1�С�
#define GPIO_OUT_LIN2   P11  //��������̵�2�С�
/*
#define GPIO_VT_CHECK  P13
#define PT2272_D0      P14
#define PT2272_D1      P15
#define PT2272_D2      P16
#define PT2272_D3      P17
*/
#define PT2272_DATA(x)  x>>4
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
//#define LED_TIME_20MIN 1200000 //ʱ���� 20*60000ms���޷��ų����ͼ���
//#define LED_TIME_5MIN  300000  //ʱ���� 5*60000ms���޷��ų����ͼ���

#define LED_TIME_20MIN 120000 //ʱ���� 20*60000ms���޷��ų����ͼ���
#define LED_TIME_5MIN  60000  //ʱ���� 5*60000ms���޷��ų����ͼ���
#define LED_TIME_1MIN  30000  //ʱ���� 5*60000ms���޷��ų����ͼ���

#define LED_TIME_60S 60000 //ʱ���� 60000ms
#define LED_TIME_65S 65000 //ʱ���� 65000ms
#define LED_TIME_95S 95000 //ʱ���� 95000ms

//========================================================================
/*************	���ر�������	**************/
BOOL B_1ms;	 		//1ms��־
BOOL B_TX1_Busy;  	//����æ��־

volatile unsigned char vGu8KeySec=0;  //�����Ĵ������
static u8 Gu8Step = 0;				  //switch �л�����
static u8 ucKeyStep = 0;
int Key_Code=0xff;
#define	UART1_TX_LENGTH 19
#define DISP_LENGTH 2
static u8 	Display_Code[DISP_LENGTH]={0x00,0x00};		   //�����ƣ�L_MOS��H_MOS��
static u8 To_Marster_Data[UART1_TX_LENGTH]={
	0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,'*'};//���ذ�̵�������״̬��֡ͷ0x05,֡β0xff;
u8 code To_Marster_OffData[]={
	0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,'*'};//ȫ��ָ��;

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
u8 code KeyVal[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};

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
void Date_EventProcess(void);

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
	//P2n_push_pull(0xff);	//����Ϊ׼˫���
	P2n_standard(0xff);	//����Ϊ׼˫���
	P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���	
	timer0_init();
	uart1_config();
	GPIO_OUT_LIN1 = 0;//����ͣ��ð������
	GPIO_OUT_LIN2 = 0;//����ͣ��ð������
	A_HC595_MR = 1;//��λ��ֹ
	A_HC595_OE = 0;//ʹ��оƬ
	P2 = 0xff;
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
		setbit(val,Kled);//�ߵ�ƽ��ͨ
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
int Get_KeyVal(void)
{
	u8 i = 0,val;
	for(i=0;i<8;i++)
	{
		if(GPIO_OUT_ROW==(~KeyVal[i])) 
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
//����ް���ֵ������,T2=20min��������;5min����ʱ����Ҫ����һ�α���ֵ
	if(Key_Code==0&&!update_flag)
	return;
	//print_char(Key_Code);
	vGu32TimeCnt_1 = 0;//�а������£�20min��ʱ��0
	vGu32TimeCnt_2 = 0;//�а������£�5min��ʱ��0
	led_bit = Key_Code;
	led_bit--;
	Gu8Step = 0;
	if(vGu8TimeFlag_2)//����ǹر�״̬����ʱ�а������£�ȫ����λ��Ȼ���Ű���ֵ����
			Kled_Set(OFF_ALL,led_bit);
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
			update_flag = TRUE;//��Ҫ����һ��
			vGu8TimeFlag_1 = 1;//��ʼ20min��ʱ
			break;
		case S9:
		case S10:
		case S11:
		case S12:
		case S13:
		case S14:
		case S15:
			Kled_Set(ON,led_bit);//��λ��������һ���Ӽ�ʱ
			vGu8TimeFlag_1 = 0;//��ͣ20min��ʱ
			vGu8TimeFlag_3 = 1;//1���Ӽ���
			vGu32TimeCnt_1 = 0;//20min��ʱ��0
			vGu32TimeCnt_2 = 0;//5min��ʱ��0
			update_flag = TRUE;//��Ҫ����һ��
			break;
		default : 
		
			break;
	}	
	Key_Code = 0;//����ֵ��0����ֹ��������
	
	Key_EventProtect = FALSE;//����������������Խ��ܰ���ֵ��
	//Date_EventProcess();	
}
void Date_EventProcess(void)
{
	u8 i=0,j=0;
	if(vGu32TimeCnt_3>=LED_TIME_1MIN)//1min��ʱ��
	{
		Kled_Set(OFF_ON_ALL,0);//ȫ��ȡ��һ��
		update_flag = TRUE;//һ���Ӻ�ĸ���
		vGu8TimeFlag_3 = 0;//1���Ӽ���ֹͣ
		vGu32TimeCnt_3 = 0;
		vGu8TimeFlag_1 = 1;//��ʼ20min��ʱ
	}
	if(update_flag == FALSE)
		return;
	for(j=0;j<DISP_LENGTH;j++){
		for(i=0;i<8;i++)
		{
			if(getbit(Display_Code[j],i))
				To_Marster_Data[j*8+i+1] = 0xfe;
			else
				To_Marster_Data[j*8+i+1] = 0x00;
		}
	}
#if defined CMD_ON_OFF_ALL
	if(KEY8_VAL == KeyCode)
		print_string(LED_CMD);
	else
		print_string(To_Marster_Data);
#else
	print_string(To_Marster_Data);
	update_flag = FALSE;
#endif
		
}
void State_EventProcess(void)
{
	switch(Gu8Step)
	{
		case 0:
			vGu8TimeFlag_2 = 0;
			if(vGu32TimeCnt_1>=LED_TIME_20MIN)//20min��ʱ��
			{
				vGu8TimeFlag_1 = 0;//ֹͣ��ʱ
				vGu32TimeCnt_1 = 0;
				Gu8Step++;
			}
			break;
		case 1:
			vGu8TimeFlag_2 = 1;//����5���ӵ���ʱ,��ʱ595 ��Ҫ����ȫ������
			print_string(To_Marster_OffData);//����ȫ��ָ�����
			Gu8Step++;
			break;			
		case 2:
			if(vGu32TimeCnt_2>=LED_TIME_5MIN)//5min��ʱ��
			{
				vGu8TimeFlag_2 = 0;//ֹͣ��ʱ
				vGu32TimeCnt_2 = 0;
				vGu8TimeFlag_1 = 1;//��ʼ20min��ʱ
				Gu8Step = 0;
				update_flag = TRUE;
				//����һ�α����ֵ
			}
			break;			
		default : 
		
			break;			
	}
}
void KeyScan(void)
{
	static unsigned char Su8KeyLock5;
	static unsigned int  Su16KeyCnt5;

	if(Key_EventProtect)//�а���δ�����������°����¼�
		return;
	switch(ucKeyStep)
	{
		case 0://ȫ������
			GPIO_OUT_LIN1 = 0;
			GPIO_OUT_LIN2 = 0;
			ucKeyStep++;
		case 1:     //�˴���С��ʱ�����ȴ��ղ�������ź��ȶ������ж������źš�����ȥ������ʱ��
			uiKeyTimeCnt++;
			if(uiKeyTimeCnt>1)
			{
				uiKeyTimeCnt=0;
				ucKeyStep++;     //�л�����һ�����в���
			}
			break;
		case 2://����а������£�ȥ��ʱ����
			if(GPIO_OUT_ROW==0xff)
				ucKeyStep = 0;
			else
				ucKeyStep++;
			break;
		case 3:
			uiKeyTimeCnt++;
			if(uiKeyTimeCnt>KEY_FILTER_TIME)//����
			{
				uiKeyTimeCnt=0;
				ucKeyStep++;     //�л�����һ�����в���
			}
			break;
		case 4:
			if(GPIO_OUT_ROW==0xff)
				ucKeyStep = 0;
			else if(GPIO_OUT_ROW!=0xff)
			{
				GPIO_OUT_LIN1 = 0;
				GPIO_OUT_LIN2 = 1;
				ucKeyStep++;     //�л�����һ�����в���
			}
		case 5:
			uiKeyTimeCnt++;
			if(uiKeyTimeCnt>1)
			{
				uiKeyTimeCnt=0;
				ucKeyStep++;     //�л�����һ�����в���
			}
			break;
		case 6:
			if(GPIO_OUT_ROW!=0xff)//�����д���
			{
				Key_Code = Get_KeyVal();//��ȡ����ֵ
			}
			else 
			{
				Key_Code = 8+Get_KeyVal();//��ȡ����ֵ
			}
			GPIO_OUT_LIN1 = 0;
			GPIO_OUT_LIN2 = 0;
			Key_EventProtect = TRUE;
			ucKeyStep++;     //�л�����һ�����в���
		case 7:
			if(GPIO_OUT_ROW==0xff)
				ucKeyStep = 0;
			break;	
		default:
			break;	
	}
/*
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
		//if(!Su8KeyLock5)
			//Kled_Set(OFF,0);//��������			
	}
	else if(0==Su8KeyLock2)
	{
		Su16KeyCnt2++;
		if(Su16KeyCnt2>=KEY_FILTER_TIME)
		{
			Su8KeyLock2=1;  
			vGu8KeySec=2;    //����2�ż�
			Key_EventProtect = TRUE;
			//Kled_Set(ON,0);//��������
		}
	}
   //��ģʽ+������K3��ɨ��ʶ��
	if(0!=KEY_INPUT3)
	{
		Su8KeyLock3=0;
		Su16KeyCnt3=0; 
		//if(!Su8KeyLock5)
			//Kled_Set(OFF,2);//��������		
	}
	else if(0==Su8KeyLock3)
	{
		Su16KeyCnt3++;
		if(Su16KeyCnt3>=KEY_FILTER_TIME)
		{
			Su8KeyLock3=1;  
			vGu8KeySec=3;    //����3�ż�
			Key_EventProtect = TRUE;
			//Kled_Set(ON,2);//��������
		}
	}
   //��ģʽ-������K4��ɨ��ʶ��
	if(0!=KEY_INPUT4)
	{
		Su8KeyLock4=0;
		Su16KeyCnt4=0; 
	//	if(!Su8KeyLock5)
			//Kled_Set(OFF,1);//��������		
	}
	else if(0==Su8KeyLock4)
	{
		Su16KeyCnt4++;
		if(Su16KeyCnt4>=KEY_FILTER_TIME)
		{
			Su8KeyLock4=1;  
			vGu8KeySec=4;    //����4�ż�
			Key_EventProtect = TRUE;
			//Kled_Set(ON,1);//��������
		}
	}
*/
	if(Key_EventProtect)
		return;//����а������£���ִ��ң�ؼ�⡣
	
	if(!(P1&0xf0))//�����Ϊ0��δ����
	{ 
	
		Su8KeyLock5=0;
		Su16KeyCnt5=0;
		//key_led_on(0);
/*
		if(!Su8KeyLock2)
			Kled_Set(OFF,0);//��ͣ������
		else if(!Su8KeyLock3)
			Kled_Set(OFF,2);//����+����	
		else if(!Su8KeyLock4)
			Kled_Set(OFF,1);//����-����	
*/
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
	if(vGu8TimeFlag_2)//���ȫ��
	{
		vDataIn595(0x00);//�������ָʾ�ƹ�
		vDataIn595(0x00);//�������ָʾ�ƹ�
		vDataOut595();
		return;
	}
	vDataIn595(Display_Code[1]);//�������ָʾ�Ƶ͸�λ״̬
	vDataIn595(Display_Code[0]);//�������ָʾ�Ƶ�8λ״̬
	vDataOut595();				//�����������
}
void vTaskfFlashLed(void)
{ 
	if(vGu8TimeFlag_1)//��״̬����
		key_led_reverse();
	else if(vGu8TimeFlag_2)//��״̬����
		key_led_on(0);
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
	vDataIn595(0x00);
	vDataIn595(0x00);
	vDataOut595();	//����Ĭ�Ϲر�ͨ����ʾLED
	puts_to_SerialPort("I am LED controller XXX_0004_20191214!\n");
	puts_to_SerialPort("Contact me: 15901856750\n");
	key_led_on(0);
	while (1)
	{	
		//KeyTask();    //������������
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
	if(vGu8TimeFlag_1){
		vGu32TimeCnt_1++;
	}
	else
		vGu32TimeCnt_1 = 0;
	if(vGu8TimeFlag_2){
		vGu32TimeCnt_2++;
	}
	else
		vGu32TimeCnt_2 = 0;
	if(vGu8TimeFlag_3){
		vGu32TimeCnt_3++;
	}
	else
		vGu32TimeCnt_3 = 0;	
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