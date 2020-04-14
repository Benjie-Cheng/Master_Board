/******************************************************************************/
/*                                                                            */
/*                   RTX_EX1.C:  The first RTX-51 Program                     */
/*                                                                            */
/******************************************************************************/
/*************	XXX_SLC1K08_M3_V10	**************/
/*
��ҵƻ������˵����
1��·�����ã���������2��ͬʱ��,����·������ģʽ��������ɺ������ϵ缴�ɡ��ٶȰ���+·����ģʽ��-·����
2����������������Խ����ݴ洢
2������ģʽ���Դ��ڹ۲����ݡ�
3��
4��

Ӳ����Դ��
1��WS2811+����ң��+ʵ�尴��2��
2��
3��
4��



�Ѿ������
1��
2��
3��

����ȱ�ݣ�
1��
2��
*/
//#include <rtx51tny.h>                 /* RTX-51 tiny functions & defines      */
#include "system.h"
#include "led_lib.h"
#include "eeprom.h"

#define SysTick 10   //10ms
#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //���մ���1��������
u8 RX_CONT = 0; 
BOOL B_TX1_Busy;  //����æ��־
KeyEnum KeyCode;
BOOL Key_EventProtect = FALSE;
BOOL E2promErase = FALSE;//e2prom ���±�־
BOOL LedFlash = FALSE;   //��˸���������б�־
SysRunStruct SysRun;     //����ϵͳ�ṹ��
typedef struct{
	u16 Second; //��
	u8  Minute; //��
	u8  Hour; //ʱ
	u16 Day; //��
	u8  Flag;
}RtcStruct;
RtcStruct idata Rtc; //����RTC �ṹ��
//--------------------------------------
static u8 	E2PROM_Strings[MAX_CFG] = {0x03,0x01,0x0a};//data0��LED num��data1��mode data2��speed
/*
data0��LED·��
data1������ģʽ
data2�������ٶ�
*/
void print_string(LogLevel level,u8 *puts);
void puts_to_SerialPort(LogLevel level,u8 *puts);
void print_char(u8 dat);
void printss(LogLevel level,u8 *puts,u8 num1);
void Sys_init (void)  {
	u8 i;
	/*104S-SOP8 ֻ��P3*/
	P0n_standard(0xff);	//����Ϊ׼˫���
	P1n_standard(0xff);	//����Ϊ׼˫���
	P2n_push_pull(0xff);//����Ϊ׼˫���
	P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���
	//P3n_pure_input(0x11);	//����Ϊ׼˫���
	SPEED_GPIO = 1;
	MODE_GPIO = 1;
	SysRun.Mode = RUN_MODE;
	while((!SPEED_GPIO&&(!MODE_GPIO)))//˫���밴����0.5S,��������ģʽ
	{
		uDelayMs(10);
		if(i++>50){
			SysRun.Mode = SET_MODE;
			i=0;
			break;
		}
	}
}
//------------------------------------------------
void RtcSystickRoutine(void)
{
	static u16 idata Counter = 0;
	if (++Counter == SysTick*100)//����10*100ms
	{
		Counter = 0;
		if(Rtc.Second < 59) //
		{
			Rtc.Second++;
			Rtc.Flag = TRUE;
		}
		else
		{
			Rtc.Second = 0;
			if(Rtc.Minute < 59) //
			Rtc.Minute++;
			else
			{
				Rtc.Minute = 0;
				if(Rtc.Hour < 23) //
				Rtc.Hour++;
				else
				{
					Rtc.Hour = 0;
					Rtc.Day++;
				}
			}
		}
	}
}
#define KEY_FILTER_TIME 2 //�˲��ġ� �ȶ�ʱ�䡱 20ms
#define KEY_TIME_1S     10
#define KEY_TIME_030S   3
void Gpio_KeyScan(u8 *KeyVal)
{
	static unsigned char Su8KeyLock1 = 0; //1 �Ű���������
	static unsigned char Su8KeyCnt1; //1 �Ű����ļ�ʱ��	
	static unsigned char uiKeyCtntyCnt1;
	static unsigned char Su8KeyLock2; //1 �Ű���������
	static unsigned char Su8KeyCnt2; //1 �Ű����ļ�ʱ��	
	static unsigned char uiKeyCtntyCnt2;
		
	if(Key_EventProtect)
		return ;
    //���ٶȡ�������ɨ��ʶ��
	if(0!=SPEED_GPIO)
	{
		Su8KeyLock1=0;
		Su8KeyCnt1=0;
		uiKeyCtntyCnt1=0;	
		*KeyVal = (u8)KeyNull;	
	}
	else if(0==Su8KeyLock1)
	{
		Su8KeyCnt1++;
		if(Su8KeyCnt1>=1)
		{
			Su8KeyLock1=0; 
			Su8KeyCnt1=0;	
			*KeyVal = (u8)KeySpeed;    //����1�ż�
			//if(SysRun.Mode != SET_MODE)//�����������ģʽ��֧����������
			{
				Key_EventProtect = TRUE;
				Su8KeyLock1 = 2;
				return;
			}
		}
	}
	else if(Su8KeyLock1==1)//�����������
	{
		if(Su8KeyCnt1 < KEY_TIME_1S)
			Su8KeyCnt1++;
		else//��ס�ۼӵ�1�����Ȼ�����֣����ʱ������н������������
		{
			uiKeyCtntyCnt1++; //����������ʱ�������ۼ�
			if(uiKeyCtntyCnt1>KEY_TIME_030S)  //��סû���֣�ÿ0.25��ʹ���һ��
			{
				uiKeyCtntyCnt1=0; 
				*KeyVal = (u8)KeySpeed;
			}			
			
		}
	}
	//��ģʽ��������ɨ��ʶ��
	if(0!=MODE_GPIO)
	{
		Su8KeyLock2=0;
		Su8KeyCnt2=0; 
		uiKeyCtntyCnt2=0;
		*KeyVal = (u8)KeyNull;		
	}
	else if(0==Su8KeyLock2)
	{
		Su8KeyCnt2++;
		if(Su8KeyCnt2>=KEY_FILTER_TIME)
		{
			Su8KeyLock2=1;  
			Su8KeyCnt2=0;
			*KeyVal = (u8)KeyRunMode;    //����1�ż�
			//if(SysRun.Mode != SET_MODE)//�����������ģʽ��֧����������
			{
				Key_EventProtect = TRUE;
				Su8KeyLock2 =2;
				return ;
			}
		}
	}
	else if(Su8KeyLock2==1)//�����������
	{
		if(Su8KeyCnt2 < KEY_TIME_1S)
			Su8KeyCnt2++;
		else//��ס�ۼӵ�1�����Ȼ�����֣����ʱ������н������������
		{
			uiKeyCtntyCnt2++; //����������ʱ�������ۼ�
			if(uiKeyCtntyCnt2>KEY_TIME_030S)  //��סû���֣�ÿ0.25��ʹ���һ��
			{
				uiKeyCtntyCnt2=0; 
				*KeyVal = (u8)KeyRunMode;
			}			
		}
	}	
}
void vEepromUpdate(void)
{
	if(!E2promErase)//���±�־Ϊ�ٲ�����
		return;
	EEPROM_SectorErase(IAP_ADDRESS);
	os_wait2 (K_SIG|K_TMO, 1);	
	EEPROM_write_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);	
	os_wait2 (K_SIG|K_TMO, 1);	
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);
	SysRun.LedNum   = E2PROM_Strings[LED_NUM_CFG];//��ȡled·��
	SysRun.LedMode  = E2PROM_Strings[RUN_TYPE_CFG];//��ȡ����ģʽ
	SysRun.LedSpeed = E2PROM_Strings[SPEED_CFG];//��ȡ�����ٶ�
	E2promErase = FALSE;
}
void TaskDisplayScan(void)//10ms ˢ��һ��
{ 	
	static u8 time = 0;
	if(LedFlash){
		key_led_reverse();
		time++;
		if(time>6)
		{
			time = 0;
			LedFlash = FALSE;//�а���������Ҫ��˸һ��LED��ʾ
			key_led_on(TRUE);
		}	
	}
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
//========================================================================
// ����: void print_string(u8 *puts)
// ����: ����1�����ַ���������
// ����: puts:  �ַ���ָ��.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================

void print_string(LogLevel level,u8 *puts)
{
	if(level<=LOG_LEVEL)
		return;
	for (; *puts != 0;	puts++)   	//����ֹͣ��*����
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
void puts_to_SerialPort(LogLevel level,u8 *puts)
{
	if(level<=LOG_LEVEL)
		return;
  for (; *puts != 0;	puts++)   	//����ֹͣ��0����
	{
		SBUF = *puts;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
	}
}
void printss(LogLevel level,u8 *puts,u8 num1)
{
	u8 temp;
	temp = rem(num1,100);
	if(level<=LOG_LEVEL)
		return;
	print_string(level,puts);
	print_char(mod(num1,100)+0x30);
	print_char(mod(temp,10)+0x30);
	print_char(rem(temp,10)+0x30);
	print_char('\n');
}
/*******************************************************************************
* ��ʼ������
*******************************************************************************/
static void InitData(void)
{
	
}
/******************************************************************************/
/*       Task 0 'job0':  RTX-51 tiny starts execution with task 0             */
/******************************************************************************/

void init (void) _task_ INIT_0{  	
	Sys_init();	
	Clear_WS2811();//��ʼ��WS2811
	uart1_config();
	puts_to_SerialPort(INFO,"I AM SLC8F1K08_M3!\n");
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);
	/*·����3��ģʽ��1���ٶȣ�10*/
	SysRun.LedNum   = E2PROM_Strings[LED_NUM_CFG]; //��ȡled·��
	SysRun.LedMode  = E2PROM_Strings[RUN_TYPE_CFG];//��ȡ����ģʽ
	SysRun.LedSpeed = E2PROM_Strings[SPEED_CFG];   //��ȡ�����ٶ�
	os_create_task (DISPLAY);                   
	os_create_task (KEY_SCAN_1);                               
	os_create_task (LIGHTS_2);                  
	os_delete_task (INIT_0);
}


/******************************************************************************/
/*    Task 1 'job1':  RTX-51 tiny starts this task with os_create_task (1)    */
/******************************************************************************/
void Dispaly (void) _task_ DISPLAY{  
  while (1)  {                        /* endless loop                         */
		RtcSystickRoutine();
		//key_led_reverse();
		//puts_to_SerialPort("I AM SLC104S_M3!\n");
		//print_char(0x02);
		if(Rtc.Flag){
			Rtc.Flag = FALSE;
			//key_led_reverse();
			//print_string("Runing time is :");
			//print_char(mod(Rtc.Hour,10)+0x30);
			//print_char(rem(Rtc.Hour,10)+0x30);
			//print_char(58);
			//print_char(mod(Rtc.Minute,10)+0x30);
			//print_char(rem(Rtc.Minute,10)+0x30);
			//print_char(58);
			//print_char(mod(Rtc.Second,10)+0x30);
			//print_char(rem(Rtc.Second,10)+0x30);
			//print_char('\n');
		}
		TaskDisplayScan();
		os_wait2(K_TMO, 50);
		//os_reset_interval (100);
  }
}

/******************************************************************************/
/*    Task 2 'job2':  RTX-51 tiny starts this task with os_create_task (2)    */
/******************************************************************************/
void Key_Scan (void) _task_ KEY_SCAN_1{
  while (1)  {                        /* endless loop                         */
		//os_wait2(K_TMO, 10);//10ms һ��
		Gpio_KeyScan(&KeyCode);
		switch(KeyCode)
		{
			case KeySpeed:
				if(SysRun.Mode == SET_MODE)
				{
					//LED++
					SysRun.LedNum = SysRun.LedNum+LED_NUM_STEP;
					SysRun.LedNum = is_max(LED_NUM_MIN,LED_NUM_MAX,SysRun.LedNum);//�������ָ���С
					printss(DEBUG,"LED num:",SysRun.LedNum);
				}
				else
				{
					//Speed++;
					SysRun.LedSpeed = SysRun.LedSpeed-SPEED_STEP;	
					SysRun.LedSpeed = is_min(SPEED_MIN,SPEED_MAX,SysRun.LedSpeed);      //С����С�ָ����	
					printss(DEBUG,"LED Speed:",SysRun.LedSpeed);					
				}
				E2promErase = TRUE;//��Ҫ�洢
				break;
			case KeyRunMode:
				if(SysRun.Mode == SET_MODE)
				{
					//LED--
					SysRun.LedNum = SysRun.LedNum-LED_NUM_STEP;
					SysRun.LedNum = is_min(LED_NUM_MIN,LED_NUM_MAX,SysRun.LedNum);        //С����С�ָ����
					printss(DEBUG,"LED num:",SysRun.LedNum);
				}
				else
				{
					//Mode++;
					SysRun.LedMode = SysRun.LedMode+LED_TYPE_STEP;
					SysRun.LedMode = is_max(LED_TYPE_MIN,LED_TYPE_MAX,SysRun.LedMode);		//�������ָ���С
					printss(DEBUG,"LED mode:",SysRun.LedMode);
				}
				E2promErase = TRUE;//��Ҫ�洢
				break;
			default: 
				break;
		}
			if(KeyCode != KeyNull){			               
				os_delete_task (LIGHTS_2);
				Clear_WS2811();//��ʼ��WS2811
				os_create_task (LIGHTS_2);   
				LedFlash = TRUE;
				KeyCode = KeyNull;
			}
		E2PROM_Strings[LED_NUM_CFG]  = SysRun.LedNum;  //��ȡled·��
		E2PROM_Strings[RUN_TYPE_CFG] = SysRun.LedMode; //��ȡ����ģʽ
		E2PROM_Strings[SPEED_CFG]    = SysRun.LedSpeed;//��ȡ�����ٶ�
		vEepromUpdate();
		Key_EventProtect = FALSE;//��ֹ��ֵδ����ʱ�ִ����°���	
		os_wait2 (K_SIG|K_TMO, 50);	
  }
}
/******************************************************************************/
/*    Task 3 'job3':  RTX-51 tiny starts this task with os_create_task (3)    */
/******************************************************************************/
void WS2811_LedTaks (void) _task_ LIGHTS_2{
  while (1)  {                        /* endless loop                         */
	switch((SysRun.Mode == SET_MODE)?0:SysRun.LedMode)
		{
			case SET_LINE://����ģʽ
				TurnOn(SysRun.LedNum,0);
				break;
			case Mode1:
				liushui123(SysRun.LedNum,1);
				//ChangeHigh(SysRun.LedNum,1);
				//ChangeLose(SysRun.LedNum,1);
				break;
			case Mode2:
				ChangeHigh(SysRun.LedNum,0);
				ChangeLose(SysRun.LedNum,0);
				break;
			case Mode3:
				BreathingAdd_Two(SysRun.LedNum);
			//	liushui123(SysRun.LedNum);
			//liushui123(SysRun.LedNum,1);
			liushui123(SysRun.LedNum,1);
				//liushui(SysRun.LedNum,0);
			//  liushui(SysRun.LedNum,1);
				//BreathingDel_Two(SysRun.LedNum);
				//BreathingAdd_Two(SysRun.LedNum);
				break;
			case Mode4:
				break;
			default:
				TurnOn(SysRun.LedNum,0);
				break;	
			}		
	}
}
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
	EA =0;
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
	EA =1;
}