/******************************************************************************/
/*                                                                            */
/*                   RTX_EX1.C:  The first RTX-51 Program                     */
/*                                                                            */
/******************************************************************************/
/*************	XXX_SLC104S_M3_V10	**************/
/*
��ҵƻ������˵����
1��·�����ã���������3��ͬʱ��,����·������ģʽ��������ɺ������ϵ缴�ɡ��ٶȰ���-·����ģʽ��+·����
2����������������Խ����ݴ洢
2������ģʽ���Դ��ڹ۲����ݡ�
3��
4��

Ӳ����Դ��
1��WS2811+����ң��+ʵ�尴��3��
2��
3��
4��



�Ѿ������
1��������ư�ͬһ�������ܿ�������Ҫ���-PT2272��ң��Ӳ����ַ��������
2������ʱ���������������������ɡ�
3�����ң�ؿ��ƶ˿�ʹ�ܹ��ܣ�д��eeprom���棬bin�ļ��������ö˿��Ƿ�ʹ�á�

����ȱ�ݣ�
1��
2��
*/
//#include <rtx51tny.h>                 /* RTX-51 tiny functions & defines      */
#include "system.h"
#include "eeprom.h"

#define SysTick 10   //10ms
#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //���մ���1��������
u8 RX_CONT = 0; 
BOOL B_TX1_Busy;  //����æ��־
static u8 KeyCode = 0;
BOOL Key_EventProtect = FALSE;
BOOL E2promErase = FALSE;//e2prom ���±�־
BOOL LedFlash = FALSE;//��˸���������б�־
BOOL DebugMode = FALSE;//����ģʽ��Ҫ��ӡlog
typedef struct{
	u16 Second; //��
	u8  Minute; //��
	u8  Hour; //ʱ
	u16 Day; //��
	u8 Flag;
} RtcStruct;
RtcStruct idata Rtc; //����RTC �ṹ��

//--------------------------------------
static u8 	E2PROM_Strings[MAX_CFG] = {0x03,0x01,0x05};//data0��LED num��data1��mode data2��speed
/*
data0��LED·��
data1������ģʽ
data2�������ٶ�
*/
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void printf(u8 *puts,u8 num1);
void Sys_init (void)  {
	/*104S-SOP8 ֻ��P3*/
	//P0n_standard(0xff);	//����Ϊ׼˫���
	//P1n_standard(0xff);	//����Ϊ׼˫���
	//P2n_push_pull(0xff);//����Ϊ׼˫���
	P3n_standard(0xff);	//����Ϊ׼˫���
	//P4n_standard(0xff);	//����Ϊ׼˫���
	//P5n_standard(0xff);	//����Ϊ׼˫���
	if(!SPEED_GPIO&&(!MODE_GPIO)&&(!NLED_GPIO))//����������������밴�����������led����ģʽ��
	{
		SysRun.Mode = SET_MODE;
		DebugMode = TRUE;
	}
	else
		SysRun.Mode = RUN_MODE;
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
	static unsigned char Su8KeyLock1; //1 �Ű���������
	static unsigned char Su8KeyCnt1; //1 �Ű����ļ�ʱ��	
	static unsigned char uiKeyCtntyCnt1;
	static unsigned char Su8KeyLock2; //1 �Ű���������
	static unsigned char Su8KeyCnt2; //1 �Ű����ļ�ʱ��	
	static unsigned char uiKeyCtntyCnt2;
	
	//*KeyVal = (u8)value;
	if(Key_EventProtect)
		return 0;
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
		if(Su8KeyCnt1>=KEY_FILTER_TIME)
		{
			Su8KeyLock1=1; 
			Su8KeyCnt1=0;		
			*KeyVal = (u8)KeySpeed;    //����1�ż�
			if(SysRun.Mode != SET_MODE)//�����������ģʽ��֧����������
			{
				Key_EventProtect = TRUE;
				Su8KeyLock1 = 2;
				return 0;
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
	if(0!=KeyRunMode)
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
			Su8KeyCnt2=0��
			*KeyVal = (u8)KeyRunMode    //����1�ż�
			if(SysRun.Mode != SET_MODE)//�����������ģʽ��֧����������
			{
				Key_EventProtect = TRUE;
				Su8KeyLock2 =2;
				return 0;
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
	SysRun.LedNum = E2PROM_Strings[LED_NUM_CFG];//��ȡled·��
	SysRun.LedMode = E2PROM_Strings[RUN_MODE_CFG];//��ȡ����ģʽ
	SysRun.LedSpeed = E2PROM_Strings[SPEED_CFG];//��ȡ�����ٶ�
	E2promErase = FALSE;
}
void TaskDisplayScan(void)//10ms ˢ��һ��
{ 	
	Gpio_KeyScan(&KeyCode);
	if(!KeyCode){
		LedFlash = TRUE;//�а���������Ҫ��˸һ��LED��ʾ
		printf("KeyScan task:",KeyCode);
	}
	switch(KeyCode)
	{
		case KeySpeed:
			KeyCode = KeyNull;
			if(SysRun.Mode == SET_MODE)
			{
				//LED++
				SysRun.LedNum++;
				SysRun.LedNum = is_max(1,254,SysRun.LedNum);//�������ָ���С
				printf("LED num:",SysRun.LedNum);
			}
			else
			{
				//Speed++;
				SysRun.LedSpeed++;	
				SysRun.LedSpeed = is_max(1,10,SysRun.LedSpeed);//�������ָ���С				
			}
			E2promErase = TRUE;//��Ҫ�洢
			break;
		case KeyRunMode:
			KeyCode = KeyNull;
			if(SysRun.Mode == SET_MODE)
			{
				//LED--
				SysRun.LedNum--;
				SysRun.LedNum = is_min(1,255,SysRun.LedNum);//С����С�ָ����
				printf("LED num:",SysRun.LedNum);
			}
			else
			{
				//Mode++;
				SysRun.LedMode++;
				is_max(0,10,SysRun.LedMode);		//�������ָ���С
				
			}
			E2promErase = TRUE;//��Ҫ�洢
			break;
		default: 
		break;
	}
	E2PROM_Strings[LED_NUM_CFG] = SysRun.LedNum;//��ȡled·��
	E2PROM_Strings[RUN_MODE_CFG] = SysRun.LedMode;//��ȡ����ģʽ
	E2PROM_Strings[SPEED_CFG] = SysRun.LedSpeed;//��ȡ�����ٶ�
	vEepromUpdate();
	Key_EventProtect = FALSE;//��ֹ��ֵδ����ʱ�ִ����°���	
	os_wait2 (K_SIG|K_TMO, 1);	
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

void print_string(u8 *puts)
{
	if(!DebugMode)
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
	if(!DebugMode)
		return;
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
void printf(u8 *puts,u8 num1)
{
	u8 temp;
	temp = rem(num1,100)
	print_string(puts);
	print_char(mod(num1,100)+0x30);
	print_char(mod(temp,10)+0x30);
	print_char(rem(temp,10)+0x30);
	print_char('\n');

}
/******************************************************************************/
/*       Task 0 'job0':  RTX-51 tiny starts execution with task 0             */
/******************************************************************************/
void init (void) _task_ INIT_0{      
	Sys_init();	
	Clear_WS2811();//��ʼ��WS2811
	uart1_config();
	puts_to_SerialPort("I AM SLC104S_M3!\n");
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);
	SysRun.LedNum = E2PROM_Strings[LED_NUM_CFG];//��ȡled·��
	SysRun.LedMode = E2PROM_Strings[RUN_MODE_CFG];//��ȡ����ģʽ
	SysRun.LedSpeed = E2PROM_Strings[SPEED_CFG];//��ȡ�����ٶ�
	os_create_task (DISPLAY);                   /* start task 1                         */
	os_create_task (KEY_SCAN_1);                /* start task 2                         */
	os_create_task (KEY_DONE_2);                /* start task 3                         */
	os_create_task (LIGHTS_3);                  /* start task 4                         */
	os_delete_task (INIT_0);
}

/******************************************************************************/
/*    Task 1 'job1':  RTX-51 tiny starts this task with os_create_task (1)    */
/******************************************************************************/
void Dispaly (void) _task_ DISPLAY{  
  while (1)  {                        /* endless loop                         */
		RtcSystickRoutine();
		if(Rtc.Flag){
			Rtc.Flag = FALSE;
			print_string("Runing time is :");
			print_char(mod(Rtc.Hour,10)+0x30);
			print_char(rem(Rtc.Hour,10)+0x30);
			print_char(58);
			print_char(mod(Rtc.Minute,10)+0x30);
			print_char(rem(Rtc.Minute,10)+0x30);
			print_char(58);
			print_char(mod(Rtc.Second,10)+0x30);
			print_char(rem(Rtc.Second,10)+0x30);
			print_char('\n');
		}
		TaskDisplayScan();//10ms ˢ��һ��
		os_wait2(K_TMO, 10);
		os_reset_interval (10);
  }
}

/******************************************************************************/
/*    Task 2 'job2':  RTX-51 tiny starts this task with os_create_task (2)    */
/******************************************************************************/
void Key_Scan (void) _task_ KEY_SCAN_1{
  while (1)  {                        /* endless loop                         */
		os_wait2(K_TMO, 1);
		if(KeyCode !=0)
			isr_set_ready(KEY_DONE_2);
		//os_wait2(K_TMO, 1);
		//os_reset_interval (1);
  }
}
/******************************************************************************/
/*    Task 3 'job3':  RTX-51 tiny starts this task with os_create_task (3)    */
/******************************************************************************/
void KeyTask (void) _task_ KEY_DONE_2{
  while (1)  {                        /* endless loop                         */
	//os_wait2(K_TMO, 100);
	os_wait2 (K_SIG|K_TMO, 1);
	switch(KeyCode){
		case KeySpeed:
			//print_char(KeyCode);
			break;
		case KeyRunMode:
			//print_char(KeyCode);
			break;
		case KeyLedNum:
			//print_char(KeyCode);
			break;
		default:     
			break;
	}
  }
	//os_wait2 (K_SIG, 0);
}
/******************************************************************************/
/*    Task 3 'job3':  RTX-51 tiny starts this task with os_create_task (3)    */
/******************************************************************************/
void WS2811_LedTaks (void) _task_ LIGHTS_3{
  while (1)  {                        /* endless loop                         */

		RgbChange();
		//led_test();
		os_wait2(K_TMO, 1);
		//led_test();
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