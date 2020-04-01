/*---------------------------------------------------------------------*/
/* --- Fuctin: Timer_Controller_only ----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/
/*************	XXX_MC204S_K8_V10	**************
�ɿع���ʱ������:��·��ʱ������-����չ3·
��·��ʱ������˵����
1��PT2272 ң��+�������ơ�
2���ɿع����ǿ���޵�𻨣���ȫ�ɿ���
3��ģʽ������->˫��->˫��
4����չ����·����ʹ��ң����ʹ�ܺ͹رա�����ɱ��档ͬʱ����ʱ���������ơ�

Ӳ����Դ��
1��TM1650 ����0.36����λ����������ܣ�ʵ��K1-K4 ���̡�
2��595����LED����ʾ����չ�ڹ��
3��PT2272 ң�ع��ܽӿڡ�
4��BT05����uart�ӿڡ�



�Ѿ������
1��������ư�ͬһ�������ܿ�������Ҫ���-PT2272��ң��Ӳ����ַ��������
2������ʱ���������������������ɡ�
3�����ң�ؿ��ƶ˿�ʹ�ܹ��ܣ�д��eeprom���棬bin�ļ��������ö˿��Ƿ�ʹ�á�

����ȱ�ݣ�
1��
2��

v1.0������趨
1��������115200 ������35M   
       
E2PROM ���ݶ��壺
data0��ͨʱ��
data1����ʱ��
data2������ģʽ
data3:֧��PT2272
data4:�˿�֧������

������־��
1�����ң�ؿ��ƶ˿�ʹ�ܹ��ܣ���Ҫд��eeprom���档
******************************************/
#include "system.h"
#include "eeprom.h"
#include "TM1650_I2C.h"

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 �ж�Ƶ��, 1000��/��
#define	Baudrate1	115200UL                                //ͨ�Ų�����115200

//---------------define in system.h-----------------//
	SysRunStruct SysRun;
	SetTimestruct SetTime;
	RunTimestruct RunTime;

volatile unsigned char vGu8KeySec=0;  //�����Ĵ������

//========================================================================
/*************	���ر�������	**************/
/********************** ������ ************************/
BOOL B_1ms;	 //1ms��־
BOOL B_TX1_Busy;  //����æ��־

//�������ʱ�� 1�� ����ر���
volatile unsigned char vGu8TimeFlag_1=0;
volatile u32 vGu32TimeCnt_1=0;	
volatile unsigned char vGu8TimeFlag_2=0;
volatile u32 vGu32TimeCnt_2=0;
	

static u8 EraseStep = 0; //������������
static u8 Brightness = 8;

#define     SET_TIME_MAX 99  //���ֵ
#define     BLMAX 8//�������
#define     BLMIM 1//������С
static u8 	E2PROM_Strings[MAX_CFG] = {0x00,0x00,0x00,0x01,0xff};
/*
data0��ͨʱ��
data1����ʱ��
data2������ģʽ
data3:֧��PT2272
data4:�˿�֧������
*/

BOOL LedFlash = TRUE;//��˸���������б�־
BOOL E2promErase = FALSE;//e2prom ���±�־
BOOL DotFlag = TRUE;//����ܵ���˸���������б�־
BOOL E2promDis = FALSE;//��ʾEEPROM ʱ��
BOOL Key_EventProtect = FALSE;


static u8   Support_Pt2272 = 0;

#define KEY_FILTER_TIME 20 //�˲��ġ� �ȶ�ʱ�䡱 20ms

#define KEY_TIME_1S  1000  //ʱ���� 10000ms
#define KEY_TIME_2S  500  //ʱ���� 10000ms
#define KEY_TIME_025S 80  //0.25���ӵ�ʱ����Ҫ�Ķ�ʱ�жϴ���
#define KEY_TIME_60S 60000 //ʱ���� 60000ms


static u8 	Display_Code[2]={0xff,0x00};		//0�����led״̬��� 2:�˿�ʹ��״̬��
static u8 	LED8[4] = {0x00,0x00,0x00,0x00};	//��ʾ����֧����λ

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //���մ���1��������
u8 RX_CONT = 0; 
u8 code t_display[]={						//��������׼�ֿ⣬����ȡ��
//	 0    1    2    3    4    5    6    7    8    9  
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};	
u8 code t_display1[]={						//��������׼�ֿ⣬����ȡ��
//	0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	
u8 code t_display2[]={						//��������׼�ֿ⣬����ȡ��
    //A    B    C    D    E    F   black -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x77,0x7C,0x39,0x5E,0x79,0x71,0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e};
u8 code KeyVal[] = {0xDF,0xD7,0xC7,0xCF};
u8 code pt2272[] = {
//   1     2    3    4    5    6    7    8   9    10   11   12   13   14   15
//	0x01,0x02,0x04,0x0a,0x0b,0x06,0x07,0x0c,0x09,0x03,0x0e,0x0d,0x0f,0x08,0x05	
		0x0a,0x04,0x01,0x02,0x0b,0x06,0x07,0x0c,0x09,0x03,0x0e,0x0d,0x0f,0x08,0x05	
};
/********************** A ��ָʾ���õ�595 ************************/
sbit	A_HC595_SER   = P1^3;	//pin 55	SER		data input
sbit	A_HC595_RCLK  = P1^4;	//pin 15	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P1^2;	//pin 54	SRCLK	Shift data clock
//sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		�͵�ƽ ʹ��enable pin
sbit	A_HC595_MR    = P1^5;	//pin 32	�͵�ƽ��λ

#define PT2272_D0      P11
#define PT2272_D1      P10
#define PT2272_D2      P37
#define PT2272_D3      P36
//#define PT2272_DATA    (PT2272_D0 + PT2272_D1*2)// + PT2272_D2*4 + PT2272_D3*8)
void InitData(void);
void vDataIn595(u8 dat);
void vDataOut595(void);
void uart1_config();	// ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void KeyProcess(KeyEnum key);
void TaskDisplayScan(void);//��ʾɨ��
void vTaskfFlashLed(void);//led ��˸
void TaskLedRunScan(void);//�����
void vEepromUpdate(void);//EEPROM��д
void Date_Transform(u8 num1,u8 num2);
	
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
	{0, 100, 100, TaskLedRunScan},         		// �����
	{0, 11, 11, vEepromUpdate},					// e2promд������50ms
//	{0, 10, 10, TaskRTC},				        // RTC����ʱ
//	{0, 30, 30, TaskDispStatus},
	// ��������������
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // ����LED
	TAST_LED_RUN,          //�����
	TAST_E2PROM_RUN,        //E2PROM ����
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
	//P1n_pure_input(0x03);
	//P3n_pure_input(0xc0);
	P2n_push_pull(0xff);	//����Ϊ׼˫���
	P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���
	A_HC595_MR = 1;     //�������������
	timer0_init();
	uart1_config();
	A_HC595_MR = 1;     //��λ��ֹ
	//PT2272_D0 = 0;
	//PT2272_D1 = 0;
	//PT2272_D2 = 0;
	//PT2272_D3 = 0;
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
void vDataOut595(void)
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
#define COM_MASK 0x78 //0111 1000
void ComX_Set(LogicType type,Hc595Bit Xbit)
{
	u8 val;
	val = Display_Code[0];//����λÿ�����㣬���������λɨ��
	if(type==ON)
		Display_Code[0] = clrbit(val,Xbit);//�͵�ƽ��ͨ
	else if(type==OFF)
		Display_Code[0] = setbit(val,Xbit);	
	else if(ON_ALL == type)	
		Display_Code[0] = (~COM_MASK) | val;//COMλ��1;
	else if(OFF_ALL == type)
		Display_Code[0] = COM_MASK & val;//COMλ��0
	else if(OFF_ON == type)
		Display_Code[0] = reversebit(val,Xbit);//��ת
}
void ComX_Enable(LogicType type,Hc595Bit Xbit)
{
	u8 val;
	val = Display_Code[1];//����ʹ�ܼ�����1-6λ
	if(type==ON)
		Display_Code[1] = clrbit(val,Xbit);//�͵�ƽ��ͨ
	else if(type==OFF)
		Display_Code[1] = setbit(val,Xbit);	
	else if(ON_ALL == type)	
		Display_Code[1] = 0x00;
	else if(OFF_ALL == type)
		Display_Code[1] = 0xff;
	else if(OFF_ON == type)
		Display_Code[1] = reversebit(val,Xbit);//��ת
}
#define LED_MASK 0x87   //1000 0111
void LedX_Set(LogicType type,Hc595Bit Xbit)
{
	u8 val;
	val = Display_Code[0];//����λÿ�����㣬���������λɨ��
	if(type==ON)
		Display_Code[0] = clrbit(val,Xbit);//�͵�ƽ��ͨ
	else if(type==OFF)
		Display_Code[0] = setbit(val,Xbit);	
	else if(ON_ALL == type)	
		Display_Code[0] = (~LED_MASK) | val;//LEDλ��1
	else if(OFF_ALL == type)
		Display_Code[0] = LED_MASK & val;//LEDλ��0
	else if(OFF_ON == type)
		Display_Code[0] = reversebit(val,Xbit);//��ת
}
void Date_Transform(u8 num1,u8 num2)
{	
	if(SysRun.Step==IDLE_MODE)//idleģʽֻ����ƣ�����������ʾ
	{
		LedX_Set(OFF_ALL,(Hc595Bit)7);//�ر�����LED;
		return ;
	}
	LED8[0] = t_display[mod(num1,10)];
	LED8[1] = t_display[rem(num1,10)];
	LED8[2] = t_display[mod(num2,10)];	
	LED8[3] = t_display[rem(num2,10)];
	
	if(DotFlag && (E2promDis == FALSE)){//����״̬����˸С����
		if(SysRun.Step==TURN_ON_MODE)
			LED8[1] = t_display1[rem(num1,10)];//��ͨ������˸
		else
			LED8[3] = t_display1[rem(num2,10)];//�ر�������˸
	}
	if(SysRun.Mode == ONLY_MINU_MODE)//����ǵ���ģʽ�رչض���ʾ
	{
		LED8[2] = t_display2[6];	
		LED8[3] = t_display2[6];
	}
}
void TaskDisplayScan(void)//10ms ˢ��һ��
{ 
	if(E2promDis)
		Date_Transform(E2PROM_Strings[ON_TIME_CFG],E2PROM_Strings[OFF_TIME_CFG]);
	else 
		Date_Transform(RunTime.on,RunTime.off);
	TM1650_Set(DIG1,LED8[0]);
	TM1650_Set(DIG2,LED8[1]);
	TM1650_Set(DIG3,LED8[2]);
	TM1650_Set(DIG4,LED8[3]);
	//vDataIn595(Display_Code[1]);//���LED����
	vDataIn595(Display_Code[0]);//����������
	vDataOut595();				//�����������
}
void vTaskfFlashLed(void)
{ 
	if(!E2promDis)
	{
		LedX_Set(OFF,RUN_MODE_LED);//��������ʱ���õƹر�;	
		return;
	}	
	if(LedFlash)
		LedX_Set(OFF_ON,RUN_MODE_LED);//���õ���˸;	
		//key_led_reverse();
	//print_char(vGu8KeySec);//��ӡ����ֵ
	//puts_to_SerialPort(E2PROM_Strings);
	
}
void TaskLedRunScan(void)
{
	static u8 LED_RUN_STEP=0;
	
	LED_RUN_STEP++;
	DotFlag = ~DotFlag;
	switch(LED_RUN_STEP)
	{
		case 1:
			LED8[3] = 0x01;
			LED8[2] = 0x00;
			break;
		case 2:
			LED8[3] = 0x02;
			LED8[2] = 0x00;
			break;
		case 3:
			LED8[3] = 0x04;
			LED8[2] = 0x00;
			break;
		case 4:
			LED8[3] = 0x08;
			LED8[2] = 0x00;
			break;
		case 5:
			LED8[3] = 0x00;
			LED8[2] = 0x08;
			break;
		case 6:
			LED8[3] = 0x00;
			LED8[2] = 0x10;
			break;
		case 7:
			LED8[3] = 0x00;
			LED8[2] = 0x20;
			break;
		case 8:
			LED8[3] = 0x00;
			LED8[2] = 0x01;
			LED_RUN_STEP = 0;
			break;
	}
	LED8[0] = LED8[2];
	LED8[1] = LED8[3];
}
void vEepromUpdate(void)
{
	if(!E2promErase)//���±�־Ϊ�ٲ�����
		return;
	switch(EraseStep) //�ֽ�д����������
	{
		case 0:
			vGu8TimeFlag_2 = 1;
			if(vGu32TimeCnt_2>=KEY_TIME_2S)//�ɿ�����2s��ʼд��EEPROM
			{
				vGu8TimeFlag_2 = 0;
				vGu32TimeCnt_2 = 0;
				EraseStep++;
			}
			break;
		case 1: 	
			EEPROM_SectorErase(IAP_ADDRESS);
			EraseStep++;
			break;
		case 2: 
			EEPROM_write_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);	
			EraseStep++;
		break;
		case 3: 	
			EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);
			RunTime.on = E2PROM_Strings[ON_TIME_CFG];
			RunTime.off = E2PROM_Strings[OFF_TIME_CFG];
			SysRun.Mode = E2PROM_Strings[RUN_MODE_CFG];
			EraseStep = 0;
			E2promErase = FALSE;
			break;
		default: 
			break;
	}
}

void KeyProcess(KeyEnum key)
{
	//print_char(key);
	if(0==key)
	{
		return; //�����Ĵ��������0��ζ���ް�����������ִ�д˺�������Ĵ���
	}
	switch(key) //���ݲ�ͬ�İ����������ִ�ж�Ӧ�Ĵ���
	{
		case KeyOffTime:     //���ض�ʱ�����á�
			key=KeyNull;
			if(!E2promDis||SysRun.Mode == ONLY_MINU_MODE)//���ֺ�����ģʽ�����ùض�ʱ��
			{
				break;
			}
			//LedX_Set(OFF_ON,RUN_MODE_LED);//�����õơ�ȡ��;	
			if(++SetTime.off>SET_TIME_MAX)
				SetTime.off = 1;
			E2promErase = TRUE;
			break;
		case KeyOnTime:     //������ʱ�����á�
			key=KeyNull;
			if(!E2promDis)//����ģʽ������ʱ��
			{
				break;
			}
			//LedX_Set(OFF_ON,RUN_MODE_LED);//�����õơ�ȡ��;
			if(++SetTime.on>SET_TIME_MAX)
				SetTime.on = 1;  
			E2promErase = TRUE;
			break;
		case KeyRunMode:     //������ģʽ��
			key=KeyNull; 
			if(++SysRun.Mode>DUAL_SEC_MODE)
				SysRun.Mode = DUAL_MINU_MODE;
			if(SysRun.Mode == ONLY_MINU_MODE)//����ǵ���ģʽ�ض�ʱ��Ϊ1;
				SetTime.off = 1;
			E2promErase = TRUE;
			break;

		case KeyBright:     //���������á�
			//LedX_Set(OFF_ON,RUN_MODE_LED);//�����õơ�ȡ��;
			key=KeyNull; 
			if(--Brightness<BLMIM)
				Brightness = 8; 
			Tube_CMD(P7_MODE,Brightness);
			
			break;
		case KeySetMode:     //������ģʽ��
			key=KeyNull; 
			//LedX_Set(OFF_ON,RUN_MODE_LED);//�����õơ�ȡ��;	
			E2promDis = ~E2promDis;
			break;
		case KeyCom1:
			E2promErase = TRUE;
			ComX_Enable(OFF_ON,COM1);
			break;
		case KeyCom2:
			E2promErase = TRUE;
			ComX_Enable(OFF_ON,COM2);
			break;
		case KeyCom3:
			E2promErase = TRUE;
			ComX_Enable(OFF_ON,COM3);
			break;
		case KeyCom4:
		case KeyCom5:
		case KeyCom6:
			break;
		default:     
			break;
	}
	if(E2promErase)
	{
		vGu8TimeFlag_2 = 0;
		vGu32TimeCnt_2 = 0;//����������ʱeeprom��������ʱ����ʼ
		EraseStep = 0;//���������豼��
		vGu8TimeFlag_1 = 0;//eprom��Ҫ����ʱ����ִ�е���ʱ
		vGu8TimeFlag_2 = 0;
		SysRun.Step = TURN_ON_MODE;//��������
	}
	E2PROM_Strings[ON_TIME_CFG] = SetTime.on;
	E2PROM_Strings[OFF_TIME_CFG] = SetTime.off;
	E2PROM_Strings[RUN_MODE_CFG] = SysRun.Mode;
	E2PROM_Strings[COM_CFG] = Display_Code[1];
	Key_EventProtect = FALSE;
}
#define COM_MAX 3//��չ��·
void Com_Match(BOOL en,u8 val)//�˿�ʹ�ܿ���
{
	u8 i=0;
	for(i=0;i<COM_MAX;i++)
	{
		if(en)
			ComX_Set(getbit(val,i)?ON:OFF,i); 
		else
			ComX_Set(OFF,i); 
	}
}
void Channle_Sw(void)
{
	switch(SysRun.Mode)
	{
		case DUAL_MINU_MODE:
			RunTime.tMode = KEY_TIME_60S;//˫��,60s
			LedX_Set(ON,DUAL_MIN_LED);  //��LED5��˫�ֵ���;
			LedX_Set(OFF,DUAL_SEC_LED); //��LED5��ȡ��;
			LedX_Set(OFF,ONLY_MIN_LED); //��LED5��ȡ��;
			break;
		case ONLY_MINU_MODE:
			RunTime.tMode = KEY_TIME_60S;//����,60s
			LedX_Set(ON,ONLY_MIN_LED);  //��LED5�����ֵ���;
			LedX_Set(OFF,DUAL_MIN_LED); //��LED5��ȡ��;
			LedX_Set(OFF,DUAL_SEC_LED); //��LED5��ȡ��;
			break;
		case DUAL_SEC_MODE:
			RunTime.tMode = KEY_TIME_1S;//˫��,1s
			LedX_Set(ON,DUAL_SEC_LED);  //��LED5��˫�����;
			LedX_Set(OFF,DUAL_MIN_LED); //��LED5��ȡ��;
			LedX_Set(OFF,ONLY_MIN_LED); //��LED5��ȡ��;
			break;
		default:	
			break;
	}
	switch(SysRun.Step)
	{
		case TURN_ON_MODE://��ͨ����ʱ
			vGu8TimeFlag_1 = 1;
			ComX_Set(ON,OUT1); //����·����;
			Com_Match(ON,Display_Code[1]);//ʹ��������
			if(vGu32TimeCnt_1>=RunTime.tMode)
			{
				vGu32TimeCnt_1 = 0;
				RunTime.on--;
				if(RunTime.on<=0){
					SysRun.Step++;
					vGu8TimeFlag_1 = 0;
					RunTime.off = E2PROM_Strings[OFF_TIME_CFG];//��ֵEEPROMֵ
					if(SysRun.Mode == ONLY_MINU_MODE)//����ǵ���ģʽ��ʱ�䵽�ر����С�
						SysRun.Step = IDLE_MODE;
				}
			}	
			break;
		case TURN_OFF_MODE://�ضϵ���ʱ
			vGu8TimeFlag_1 = 1;
			ComX_Set(OFF,OUT1);//����·����;
			Com_Match(OFF,Display_Code[1]);//ʹ��������
			if(vGu32TimeCnt_1>=RunTime.tMode)
			{
				vGu32TimeCnt_1 = 0;
				RunTime.off--;
				if(RunTime.off <= 0){
					SysRun.Step = TURN_ON_MODE;
					vGu8TimeFlag_1 = 0;
					RunTime.on = E2PROM_Strings[ON_TIME_CFG];//��ֵEEPROMֵ
				}
			}
			break;
		case IDLE_MODE://���ֹض�ģʽ״̬
			ComX_Set(OFF_ALL,COMX);//���ر����ж˿ڡ�;
			vGu8TimeFlag_1 = 0;	
			break;
			
		default:	
			break;
	}
}
int Get_Pt2272State(void)
{
	u8 i = 0,val;
	u8 m,j,k,l;
	m = PT2272_D0;
	j = PT2272_D1*2;
	k = PT2272_D2*4;
	l = PT2272_D3*8;
	for(i=0;i<15;i++)
	{
		val = m+j+k+l;
		
		if(val == pt2272[i]) 
		{
			val = i+1;
			break;
		}
		else 
			val = 0;
	}
  //print_char(val);
	if(val == 0x0d)//����ε�ң����GPIOȫΪ�߲���ʹ�õ����⡣
		val = 0;
	return val;
}
int Get_KeyVal(void)
{
	u8 i = 0,val,Key_Code;
	static unsigned char Su8KeyLock1;
	static unsigned int  Su16KeyCnt1;
	static unsigned int  uiKeyCtntyCnt1;
	static unsigned char Su8KeyLock2;
	static unsigned int  Su16KeyCnt2;
	static unsigned int  uiKeyCtntyCnt2;
	
	if(Key_EventProtect)
		return 0;
if( Support_Pt2272){
	
	Key_Code = Get_Pt2272State();
	//print_char(Key_Code);
	if(!Key_Code)//���Ϊ0��δ����
	{ 
		Su8KeyLock2=0;
		Su16KeyCnt2=0;
		uiKeyCtntyCnt2=0;
	}
	else if(0==Su8KeyLock2)
	{
		Su16KeyCnt2++;
		if(Su16KeyCnt2>=KEY_FILTER_TIME)
		{
			
			Su8KeyLock2=1;
			Su16KeyCnt2=0;	
			if(Key_Code == KeyOnTime || Key_Code == KeyOffTime)
				Key_EventProtect = FALSE;//����Ҫ��������
			else
				Key_EventProtect = TRUE;//��Ҫ��������
			return (Key_Code);
		}
	}
	else if(Su8KeyLock2)//�����������
	{
		if(Key_Code!=KeyOnTime && Key_Code!=KeyOffTime)//����S1,S2���ٴ���
			return 0;
		else
		{	
			if(Su16KeyCnt2 < KEY_TIME_1S)
				 Su16KeyCnt2++;
			else//��ס�ۼӵ�1�����Ȼ�����֣����ʱ������н������������
			{
				uiKeyCtntyCnt2++; //����������ʱ�������ۼ�
				if(uiKeyCtntyCnt2>KEY_TIME_025S)  //��סû���֣�ÿ0.25��ʹ���һ��
				{
					uiKeyCtntyCnt2=0; 
					return Key_Code;
				}
			}
		}
	}
}
	for(i=0;i<8;i++)
	{
		if(Scan_Key()==KeyVal[i]) 
		{
			val = i+1;
			break;
		}
		else 
			val = 0;
	}
	//print_char(val);
	if(val==0){
		Su8KeyLock1=0;
		Su16KeyCnt1=0; 
		uiKeyCtntyCnt1=0; //�����ۼӵ�ʱ������ʱ����������	
		return 0;
	}
	else if(0==Su8KeyLock1)
	{
		Su8KeyLock1 = 1;
		if(val == KeyOnTime || val == KeyOffTime)
			Key_EventProtect = FALSE;//����Ҫ��������
		else
			Key_EventProtect = TRUE;//��Ҫ��������
		return val;
	}
	else if(val!=KeyOnTime && val!=KeyOffTime)
	{
		return 0;
	}
	else if(Su16KeyCnt1 < KEY_TIME_1S)
	{
		Su16KeyCnt1++;
	}
	else//��ס�ۼӵ�1�����Ȼ�����֣����ʱ������н������������
	{
		uiKeyCtntyCnt1++; //����������ʱ�������ۼ�
		 if(uiKeyCtntyCnt1>KEY_TIME_025S)  //��סû���֣�ÿ0.25��ʹ���һ��
		 {
			uiKeyCtntyCnt1=0; 
			return val;
		 }
	}		
	return 0;	
}
void InitData(void)
{
	SysRun.Mode = DUAL_MINU_MODE;
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
	InitData();
	stc15x_hw_init();
	vDataIn595(0xff);
	//vDataIn595(0xff);
	vDataOut595();	//����Ĭ�Ϲر�ͨ����ʾLED
	puts_to_SerialPort("I am Timer_Controller_only!\n");
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);
	SetTime.on  = RunTime.on = E2PROM_Strings[ON_TIME_CFG];
	SetTime.off = RunTime.off = E2PROM_Strings[OFF_TIME_CFG];
	SysRun.Mode = E2PROM_Strings[RUN_MODE_CFG];
	Support_Pt2272 = E2PROM_Strings[PT_CFG];
	Display_Code[1] = E2PROM_Strings[COM_CFG];
	
	Init_Tm1650();//����ܿ���ʾ
	//Tube_CMD(P7_MODE,3);
	
	TM1650_Set(DIG1,t_display[0]);
	TM1650_Set(DIG2,t_display[1]);
	TM1650_Set(DIG3,t_display[2]);
	TM1650_Set(DIG4,t_display[3]);
	WDT_reset(D_WDT_SCALE_128);
	while(1)
	{
		if(B_1ms)	//1ms��
		{
			B_1ms = 0;	
			vGu8KeySec = Get_KeyVal();//��ȡTM1650����ֵ
			if(IDLE_MODE==SysRun.Step)//�������idle ģʽ��������Ч��ֻ�������ϵ����Ч��
				vGu8KeySec=0;
			Channle_Sw();
			KeyProcess((KeyEnum)vGu8KeySec);    //������������
			TaskProcess();//1��led 2���������˸ 3��e2prom��д
		}
		TaskDisplayScan();
	}
}

//========================================================================
// ����: Timer0 1ms�жϺ�����
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	static volatile u8 count=0;
	B_1ms = 1;		//1ms��־
	TaskRemarks();
	count++;
	if(count>=10){
		WDT_reset(D_WDT_SCALE_128);
		count = 0 ;
	}
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