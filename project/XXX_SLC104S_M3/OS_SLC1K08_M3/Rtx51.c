/******************************************************************************/
/*                                                                            */
/*                   RTX_EX1.C:  The first RTX-51 Program                     */
/*                                                                            */
/******************************************************************************/
/*************	XXX_SLC104S_M3_V10	**************/
/*
Íò¼ÒµÆ»ğ¿ØÖÆÆ÷ËµÃ÷£º
1¡¢Â·ÊıÉèÖÃ£º¿ª»ú¹ı³Ì2¼üÍ¬Ê±°´,½øÈëÂ·ÊıÉèÖÃÄ£Ê½£¬ÉèÖÃÍê³Éºó£¬ÖØĞÂÉÏµç¼´¿É¡£ËÙ¶È°´¼ü-Â·Êı£¬Ä£Ê½¼ü+Â·Êı¡£
2¡¢°´¼ü´¥·¢ºó¾ù¿ÉÒÔ½«Êı¾İ´æ´¢
2¡¢ÉèÖÃÄ£Ê½¿ÉÒÔ´®¿Ú¹Û²ìÊı¾İ¡£
3¡¢
4¡¢

Ó²¼ş×ÊÔ´£º
1¡¢WS2811+ºìÍâÒ£¿Ø+ÊµÌå°´¼ü3¸ö
2¡¢
3¡¢
4¡¢



ÒÑ¾­½â¾ö£º
1¡¢¶à¸ö¿ØÖÆ°åÍ¬Ò»¸ö°´¼üÊÜ¿ØÎÊÌâĞèÒª½â¾ö-PT2272ºÍÒ£¿ØÓ²¼şµØÖ·±àÂë½â¾ö¡£
2¡¢³¤°´Ê±¼äÉèÖÃÁ¬Ğø´¥·¢¡£Èí¼şÍê³É¡£
3¡¢Ìí¼ÓÒ£¿Ø¿ØÖÆ¶Ë¿ÚÊ¹ÄÜ¹¦ÄÜ£¬Ğ´Èëeeprom±£´æ£¬binÎÄ¼ş¿ÉÒÔÉèÖÃ¶Ë¿ÚÊÇ·ñÊ¹ÓÃ¡£

´æÔÚÈ±Ïİ£º
1¡¢
2¡¢
*/
//#include <rtx51tny.h>                 /* RTX-51 tiny functions & defines      */
#include "system.h"
#include "led_lib.h"
#include "eeprom.h"

#define SysTick 10   //10ms
#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //½ÓÊÕ´®¿Ú1»º´æÊı×é
u8 RX_CONT = 0; 
BOOL B_TX1_Busy;  //·¢ËÍÃ¦±êÖ¾
//static u8 KeyCode = 0;
KeyEnum KeyCode;
BOOL Key_EventProtect = FALSE;
BOOL E2promErase = FALSE;//e2prom ¸üĞÂ±êÖ¾
BOOL LedFlash = FALSE;//ÉÁË¸µÆÔÊĞíÔËĞĞ±êÖ¾
BOOL DebugMode = FALSE;//ÉèÖÃÄ£Ê½ĞèÒª´òÓ¡log
//u8 nWs=2;
SysRunStruct SysRun;
typedef struct{
	u16 Second; //Ãë
	u8  Minute; //·Ö
	u8  Hour; //Ê±
	u16 Day; //Ìì
	u8 Flag;
} RtcStruct;
RtcStruct idata Rtc; //¶¨ÒåRTC ½á¹¹Ìå
//--------------------------------------
static u8 	E2PROM_Strings[MAX_CFG] = {0x03,0x01,0x0a};//data0£ºLED num£¬data1£ºmode data2£ºspeed
/*
data0£ºLEDÂ·Êı
data1£º»¨ÑùÄ£Ê½
data2£ºÔËĞĞËÙ¶È
*/
void print_string(LogLevel level,u8 *puts);
void puts_to_SerialPort(LogLevel level,u8 *puts);
void print_char(u8 dat);
void printss(LogLevel level,u8 *puts,u8 num1);
void Sys_init (void)  {
	/*104S-SOP8 Ö»ÓĞP3*/
	P0n_standard(0xff);	//ÉèÖÃÎª×¼Ë«Ïò¿Ú
	P1n_standard(0xff);	//ÉèÖÃÎª×¼Ë«Ïò¿Ú
	P2n_push_pull(0xff);//ÉèÖÃÎª×¼Ë«Ïò¿Ú
	P3n_standard(0xff);	//ÉèÖÃÎª×¼Ë«Ïò¿Ú
	P4n_standard(0xff);	//ÉèÖÃÎª×¼Ë«Ïò¿Ú
	P5n_standard(0xff);	//ÉèÖÃÎª×¼Ë«Ïò¿Ú
	//P3n_pure_input(0x11);	//ÉèÖÃÎª×¼Ë«Ïò¿Ú
	SPEED_GPIO = 1;
	MODE_GPIO = 1;
	if(!SPEED_GPIO&&(!MODE_GPIO))//Èç¹û¿ª»ú¹ı³ÌÈı¼üÆë°´£¬Ôò½øÈëÉèÖÃled¸öÊıÄ£Ê½¡£
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
	if (++Counter == SysTick*100)//ĞÄÌø10*100ms
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
#define KEY_FILTER_TIME 2 //ÂË²¨µÄ¡° ÎÈ¶¨Ê±¼ä¡± 20ms
#define KEY_TIME_1S     10
#define KEY_TIME_030S   3
void Gpio_KeyScan(u8 *KeyVal)
{
	static unsigned char Su8KeyLock1 = 0; //1 ºÅ°´¼üµÄ×ÔËø
	static unsigned char Su8KeyCnt1; //1 ºÅ°´¼üµÄ¼ÆÊ±Æ÷	
	static unsigned char uiKeyCtntyCnt1;
	static unsigned char Su8KeyLock2; //1 ºÅ°´¼üµÄ×ÔËø
	static unsigned char Su8KeyCnt2; //1 ºÅ°´¼üµÄ¼ÆÊ±Æ÷	
	static unsigned char uiKeyCtntyCnt2;
	
	
	if(Key_EventProtect)
		return ;
    //¡¾ËÙ¶È¡¿°´¼üµÄÉ¨ÃèÊ¶±ğ
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
			*KeyVal = (u8)KeySpeed;    //´¥·¢1ºÅ¼ü
			//if(SysRun.Mode != SET_MODE)//Èç¹û²»ÊÇÉèÖÃÄ£Ê½²»Ö§³ÖÁ¬Ğø´¥·¢
			{
				Key_EventProtect = TRUE;
				Su8KeyLock1 = 2;
				return;
			}
		}
	}
	else if(Su8KeyLock1==1)//Èç¹ûÁ¬Ğø°´ÏÂ
	{
		if(Su8KeyCnt1 < KEY_TIME_1S)
			Su8KeyCnt1++;
		else//°´×¡ÀÛ¼Óµ½1ÃëºóÈÔÈ»²»·ÅÊÖ£¬Õâ¸öÊ±ºò½øÈëÓĞ½Ú×àµÄÁ¬Ğø´¥·¢
		{
			uiKeyCtntyCnt1++; //Á¬Ğø´¥·¢ÑÓÊ±¼ÆÊıÆ÷ÀÛ¼Ó
			if(uiKeyCtntyCnt1>KEY_TIME_030S)  //°´×¡Ã»ËÉÊÖ£¬Ã¿0.25Ãë¾Í´¥·¢Ò»´Î
			{
				uiKeyCtntyCnt1=0; 
				*KeyVal = (u8)KeySpeed;
			}			
			
		}
	}
	//¡¾Ä£Ê½¡¿°´¼üµÄÉ¨ÃèÊ¶±ğ
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
			*KeyVal = (u8)KeyRunMode;    //´¥·¢1ºÅ¼ü
			//if(SysRun.Mode != SET_MODE)//Èç¹û²»ÊÇÉèÖÃÄ£Ê½²»Ö§³ÖÁ¬Ğø´¥·¢
			{
				Key_EventProtect = TRUE;
				Su8KeyLock2 =2;
				return ;
			}
		}
	}
	else if(Su8KeyLock2==1)//Èç¹ûÁ¬Ğø°´ÏÂ
	{
		if(Su8KeyCnt2 < KEY_TIME_1S)
			Su8KeyCnt2++;
		else//°´×¡ÀÛ¼Óµ½1ÃëºóÈÔÈ»²»·ÅÊÖ£¬Õâ¸öÊ±ºò½øÈëÓĞ½Ú×àµÄÁ¬Ğø´¥·¢
		{
			uiKeyCtntyCnt2++; //Á¬Ğø´¥·¢ÑÓÊ±¼ÆÊıÆ÷ÀÛ¼Ó
			if(uiKeyCtntyCnt2>KEY_TIME_030S)  //°´×¡Ã»ËÉÊÖ£¬Ã¿0.25Ãë¾Í´¥·¢Ò»´Î
			{
				uiKeyCtntyCnt2=0; 
				*KeyVal = (u8)KeyRunMode;
			}			
		}
	}	
}
void vEepromUpdate(void)
{
	if(!E2promErase)//¸üĞÂ±êÖ¾Îª¼Ù²»¸üĞÂ
		return;
	EEPROM_SectorErase(IAP_ADDRESS);
	os_wait2 (K_SIG|K_TMO, 1);	
	EEPROM_write_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);	
	os_wait2 (K_SIG|K_TMO, 1);	
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);
	SysRun.LedNum = E2PROM_Strings[LED_NUM_CFG];//»ñÈ¡ledÂ·Êı
	SysRun.LedMode = E2PROM_Strings[RUN_MODE_CFG];//»ñÈ¡ÔËĞĞÄ£Ê½
	SysRun.LedSpeed = E2PROM_Strings[SPEED_CFG];//»ñÈ¡ÔËĞĞËÙ¶È
	E2promErase = FALSE;
}
void TaskDisplayScan(void)//10ms Ë¢ĞÂÒ»´Î
{ 	
	static u8 time = 0;
	if(LedFlash){
		key_led_reverse();
		time++;
		if(time>6)
		{
			time = 0;
			LedFlash = FALSE;//ÓĞ°´¼ü°´ÏÂĞèÒªÉÁË¸Ò»´ÎLEDÌáÊ¾
			key_led_on(TRUE);
		}	
	}

}
//========================================================================
// º¯Êı: set_timer2_baudraye(u16 dat)
// ÃèÊö: ÉèÖÃTimer2×ö²¨ÌØÂÊ·¢ÉúÆ÷¡£
// ²ÎÊı: dat: Timer2µÄÖØ×°Öµ.
// ·µ»Ø: none.
// °æ±¾: VER1.0
// ÈÕÆÚ: 2014-11-28
// ±¸×¢: 
//========================================================================

void set_timer2_baudraye(u16 dat)
{
	AUXR &= ~(1<<4);	//Timer stop
	AUXR &= ~(1<<3);	//Timer2 set As Timer
	AUXR |=  (1<<2);	//Timer2 set as 1T mode
	TH2 = dat / 256;
	TL2 = dat % 256;
	IE2  &= ~(1<<2);	//½ûÖ¹ÖĞ¶Ï
	AUXR |=  (1<<4);	//Timer run enable
}

void uart1_config()
{
	/*********** ²¨ÌØÂÊÊ¹ÓÃ¶¨Ê±Æ÷2 *****************/
	AUXR |= 0x01;		//S1 BRT Use Timer2;
	set_timer2_baudraye(65536UL - (MAIN_Fosc / 4) / Baudrate1);

	SCON = (SCON & 0x3f) | 0x40;	//UART1Ä£Ê½, 0x00: Í¬²½ÒÆÎ»Êä³ö, 0x40: 8Î»Êı¾İ,¿É±ä²¨ÌØÂÊ, 0x80: 9Î»Êı¾İ,¹Ì¶¨²¨ÌØÂÊ, 0xc0: 9Î»Êı¾İ,¿É±ä²¨ÌØÂÊ
	S1_USE_P30P31();//UART1 Ê¹ÓÃP30 P31¿Ú	Ä¬ÈÏ
	ES  = 1;	//ÔÊĞíÖĞ¶Ï
	REN = 1;	//ÔÊĞí½ÓÊÕ
	EA = 1;

	B_TX1_Busy = 0;
}
//========================================================================
// º¯Êı: void print_string(u8 *puts)
// ÃèÊö: ´®¿Ú1·¢ËÍ×Ö·û´®º¯Êı¡£
// ²ÎÊı: puts:  ×Ö·û´®Ö¸Õë.
// ·µ»Ø: none.
// °æ±¾: VER1.0
// ÈÕÆÚ: 2014-11-28
// ±¸×¢: 
//========================================================================

void print_string(LogLevel level,u8 *puts)
{
	if(level<=LOG_LEVEL)
		return;
	for (; *puts != 0;	puts++)   	//Óöµ½Í£Ö¹·û*½áÊø
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
  for (; *puts != 0;	puts++)   	//Óöµ½Í£Ö¹·û0½áÊø
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
* ³õÊ¼»¯Êı¾İ
*******************************************************************************/
static void InitData(void)
{
	
}
/******************************************************************************/
/*       Task 0 'job0':  RTX-51 tiny starts execution with task 0             */
/******************************************************************************/

void init (void) _task_ INIT_0{  	
		Sys_init();	
		Clear_WS2811();//³õÊ¼»¯WS2811
		uart1_config();
		puts_to_SerialPort(INFO,"I AM SLC8F1K08_M3!\n");
		EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);
		SysRun.LedNum = E2PROM_Strings[LED_NUM_CFG];//»ñÈ¡ledÂ·Êı
		SysRun.LedMode = E2PROM_Strings[RUN_MODE_CFG];//»ñÈ¡ÔËĞĞÄ£Ê½
		SysRun.LedSpeed = E2PROM_Strings[SPEED_CFG];//»ñÈ¡ÔËĞĞËÙ¶È
		os_create_task (DISPLAY);                   
		os_create_task (KEY_SCAN_1);                               
		os_create_task (LIGHTS_3);                  
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
		//os_wait2(K_TMO, 10);//10ms Ò»´Î
		Gpio_KeyScan(&KeyCode);
		switch(KeyCode)
		{
			case KeySpeed:
				
				if(SysRun.Mode == SET_MODE)
				{
					//LED++
					SysRun.LedNum++;
					SysRun.LedNum = is_max(1,255,SysRun.LedNum);//´óÓÚ×î´ó»Ö¸´×îĞ¡
					printss(DEBUG,"LED num:",SysRun.LedNum);
				}
				else
				{
					//Speed++;
					SysRun.LedSpeed = SysRun.LedSpeed-2;	
					SysRun.LedSpeed = is_min(1,20,SysRun.LedSpeed);//´óÓÚ×î´ó»Ö¸´×îĞ¡	
					printss(DEBUG,"LED Speed:",SysRun.LedSpeed);					
				}
				E2promErase = TRUE;//ĞèÒª´æ´¢
				break;
			case KeyRunMode:
				//nWs--;
				if(SysRun.Mode == SET_MODE)
				{
					//LED--
					SysRun.LedNum--;
					SysRun.LedNum = is_min(0,255,SysRun.LedNum);//Ğ¡ÓÚ×îĞ¡»Ö¸´×î´ó
					printss(DEBUG,"LED num:",SysRun.LedNum);
					SysRun.LedMode = 0;
				}
				else
				{
					//Mode++;
					SysRun.LedMode++;
					SysRun.LedMode = is_max(0,3,SysRun.LedMode);		//´óÓÚ×î´ó»Ö¸´×îĞ
					printss(DEBUG,"LED mode:",SysRun.LedMode);
					
				}
				E2promErase = TRUE;//ĞèÒª´æ´¢
				break;
			default: 
				break;
		}
			if(KeyCode != KeyNull){
						               
				os_delete_task (LIGHTS_3);
				Clear_WS2811();//³õÊ¼»¯WS2811
				os_create_task (LIGHTS_3);   
				LedFlash = TRUE;
				KeyCode = KeyNull;
			}
		E2PROM_Strings[LED_NUM_CFG] = SysRun.LedNum;//»ñÈ¡ledÂ·Êı
		E2PROM_Strings[RUN_MODE_CFG] = SysRun.LedMode;//»ñÈ¡ÔËĞĞÄ£Ê½
		E2PROM_Strings[SPEED_CFG] = SysRun.LedSpeed;//»ñÈ¡ÔËĞĞËÙ¶È
		vEepromUpdate();
		Key_EventProtect = FALSE;//·ÀÖ¹¼üÖµÎ´´¦ÀíÊ±ÓÖ´¥·¢ĞÂ°´¼ü	
		os_wait2 (K_SIG|K_TMO, 50);	
  }
}
/******************************************************************************/
/*    Task 3 'job3':  RTX-51 tiny starts this task with os_create_task (3)    */
/******************************************************************************/
void KeyTask (void) _task_ KEY_DONE_2{
  while (1)  {                        /* endless loop                         */
	//os_wait2(K_TMO, 100);
	//os_wait2 (K_SIG|K_TMO, 10);
		
		switch(KeyCode){
			case KeySpeed:
				//key_led_reverse();
				//print_char(KeyCode);
				break;
			case KeyRunMode:
				//key_led_reverse();
				//print_char(KeyCode);
				break;
			case KeyLedNum:
				//print_char(KeyCode);
				break;
			default:     
				break;
		}
		KeyCode = 0;
  }
	//os_wait2 (K_SIG, 0);
}
/******************************************************************************/
/*    Task 3 'job3':  RTX-51 tiny starts this task with os_create_task (3)    */
/******************************************************************************/
void WS2811_LedTaks (void) _task_ LIGHTS_3{
  while (1)  {                        /* endless loop                         */
		
		switch((SysRun.Mode == SET_MODE)?0:SysRun.LedMode)
		{
			case SET_LINE:
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
			
		}
  }
}
//========================================================================
// º¯Êı: void uart1_int (void) interrupt UART1_VECTOR
// ÃèÊö: UART1ÖĞ¶Ïº¯Êı¡£
// ²ÎÊı: nine.
// ·µ»Ø: none.
// °æ±¾: VER1.0
// ÈÕÆÚ: 2014-11-28
// ±¸×¢: 
//========================================================================
void uart1_int (void) interrupt UART1_VECTOR
{
	EA =0;
	if(RI)
	{
		RI = 0;
		Rec_Buf[RX_CONT] = SBUF;                    //°Ñ´®¿Ú1»º´æSBUF¼Ä´æÆ÷Êı¾İÒÀ´Î´æ·Åµ½Êı×éRec_BufÖĞ
		RX_CONT++;                                   
		if(RX_CONT>Buf_Max)                          //½ÓÊÕÊı´óÓÚ¶¨Òå½ÓÊÕÊı×é×î´ó¸öÊıÊ±£¬¸²¸Ç½ÓÊÕÊı×éÖ®Ç°Öµ
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