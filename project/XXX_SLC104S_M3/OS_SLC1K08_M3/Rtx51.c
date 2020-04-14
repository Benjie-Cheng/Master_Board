/******************************************************************************/
/*                                                                            */
/*                   RTX_EX1.C:  The first RTX-51 Program                     */
/*                                                                            */
/******************************************************************************/
/*************	XXX_SLC1K08_M3_V10	**************/
/*
万家灯火控制器说明：
1、路数设置：开机过程2键同时按,进入路数设置模式，设置完成后，重新上电即可。速度按键+路数，模式键-路数。
2、按键触发后均可以将数据存储
2、设置模式可以串口观察数据。
3、
4、

硬件资源：
1、WS2811+红外遥控+实体按键2个
2、
3、
4、



已经解决：
1、
2、
3、

存在缺陷：
1、
2、
*/
//#include <rtx51tny.h>                 /* RTX-51 tiny functions & defines      */
#include "system.h"
#include "led_lib.h"
#include "eeprom.h"

#define SysTick 10   //10ms
#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //接收串口1缓存数组
u8 RX_CONT = 0; 
BOOL B_TX1_Busy;  //发送忙标志
KeyEnum KeyCode;
BOOL Key_EventProtect = FALSE;
BOOL E2promErase = FALSE;//e2prom 更新标志
BOOL LedFlash = FALSE;   //闪烁灯允许运行标志
SysRunStruct SysRun;     //定义系统结构体
typedef struct{
	u16 Second; //秒
	u8  Minute; //分
	u8  Hour; //时
	u16 Day; //天
	u8  Flag;
}RtcStruct;
RtcStruct idata Rtc; //定义RTC 结构体
//--------------------------------------
static u8 	E2PROM_Strings[MAX_CFG] = {0x03,0x01,0x0a};//data0：LED num，data1：mode data2：speed
/*
data0：LED路数
data1：花样模式
data2：运行速度
*/
void print_string(LogLevel level,u8 *puts);
void puts_to_SerialPort(LogLevel level,u8 *puts);
void print_char(u8 dat);
void printss(LogLevel level,u8 *puts,u8 num1);
void Sys_init (void)  {
	u8 i;
	/*104S-SOP8 只有P3*/
	P0n_standard(0xff);	//设置为准双向口
	P1n_standard(0xff);	//设置为准双向口
	P2n_push_pull(0xff);//设置为准双向口
	P3n_standard(0xff);	//设置为准双向口
	P4n_standard(0xff);	//设置为准双向口
	P5n_standard(0xff);	//设置为准双向口
	//P3n_pure_input(0x11);	//设置为准双向口
	SPEED_GPIO = 1;
	MODE_GPIO = 1;
	SysRun.Mode = RUN_MODE;
	while((!SPEED_GPIO&&(!MODE_GPIO)))//双键齐按超过0.5S,进入设置模式
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
	if (++Counter == SysTick*100)//心跳10*100ms
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
#define KEY_FILTER_TIME 2 //滤波的“ 稳定时间” 20ms
#define KEY_TIME_1S     10
#define KEY_TIME_030S   3
void Gpio_KeyScan(u8 *KeyVal)
{
	static unsigned char Su8KeyLock1 = 0; //1 号按键的自锁
	static unsigned char Su8KeyCnt1; //1 号按键的计时器	
	static unsigned char uiKeyCtntyCnt1;
	static unsigned char Su8KeyLock2; //1 号按键的自锁
	static unsigned char Su8KeyCnt2; //1 号按键的计时器	
	static unsigned char uiKeyCtntyCnt2;
		
	if(Key_EventProtect)
		return ;
    //【速度】按键的扫描识别
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
			*KeyVal = (u8)KeySpeed;    //触发1号键
			//if(SysRun.Mode != SET_MODE)//如果不是设置模式不支持连续触发
			{
				Key_EventProtect = TRUE;
				Su8KeyLock1 = 2;
				return;
			}
		}
	}
	else if(Su8KeyLock1==1)//如果连续按下
	{
		if(Su8KeyCnt1 < KEY_TIME_1S)
			Su8KeyCnt1++;
		else//按住累加到1秒后仍然不放手，这个时候进入有节奏的连续触发
		{
			uiKeyCtntyCnt1++; //连续触发延时计数器累加
			if(uiKeyCtntyCnt1>KEY_TIME_030S)  //按住没松手，每0.25秒就触发一次
			{
				uiKeyCtntyCnt1=0; 
				*KeyVal = (u8)KeySpeed;
			}			
			
		}
	}
	//【模式】按键的扫描识别
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
			*KeyVal = (u8)KeyRunMode;    //触发1号键
			//if(SysRun.Mode != SET_MODE)//如果不是设置模式不支持连续触发
			{
				Key_EventProtect = TRUE;
				Su8KeyLock2 =2;
				return ;
			}
		}
	}
	else if(Su8KeyLock2==1)//如果连续按下
	{
		if(Su8KeyCnt2 < KEY_TIME_1S)
			Su8KeyCnt2++;
		else//按住累加到1秒后仍然不放手，这个时候进入有节奏的连续触发
		{
			uiKeyCtntyCnt2++; //连续触发延时计数器累加
			if(uiKeyCtntyCnt2>KEY_TIME_030S)  //按住没松手，每0.25秒就触发一次
			{
				uiKeyCtntyCnt2=0; 
				*KeyVal = (u8)KeyRunMode;
			}			
		}
	}	
}
void vEepromUpdate(void)
{
	if(!E2promErase)//更新标志为假不更新
		return;
	EEPROM_SectorErase(IAP_ADDRESS);
	os_wait2 (K_SIG|K_TMO, 1);	
	EEPROM_write_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);	
	os_wait2 (K_SIG|K_TMO, 1);	
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);
	SysRun.LedNum   = E2PROM_Strings[LED_NUM_CFG];//获取led路数
	SysRun.LedMode  = E2PROM_Strings[RUN_TYPE_CFG];//获取运行模式
	SysRun.LedSpeed = E2PROM_Strings[SPEED_CFG];//获取运行速度
	E2promErase = FALSE;
}
void TaskDisplayScan(void)//10ms 刷新一次
{ 	
	static u8 time = 0;
	if(LedFlash){
		key_led_reverse();
		time++;
		if(time>6)
		{
			time = 0;
			LedFlash = FALSE;//有按键按下需要闪烁一次LED提示
			key_led_on(TRUE);
		}	
	}
}
//========================================================================
// 函数: set_timer2_baudraye(u16 dat)
// 描述: 设置Timer2做波特率发生器。
// 参数: dat: Timer2的重装值.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================

void set_timer2_baudraye(u16 dat)
{
	AUXR &= ~(1<<4);	//Timer stop
	AUXR &= ~(1<<3);	//Timer2 set As Timer
	AUXR |=  (1<<2);	//Timer2 set as 1T mode
	TH2 = dat / 256;
	TL2 = dat % 256;
	IE2  &= ~(1<<2);	//禁止中断
	AUXR |=  (1<<4);	//Timer run enable
}

void uart1_config()
{
	/*********** 波特率使用定时器2 *****************/
	AUXR |= 0x01;		//S1 BRT Use Timer2;
	set_timer2_baudraye(65536UL - (MAIN_Fosc / 4) / Baudrate1);

	SCON = (SCON & 0x3f) | 0x40;	//UART1模式, 0x00: 同步移位输出, 0x40: 8位数据,可变波特率, 0x80: 9位数据,固定波特率, 0xc0: 9位数据,可变波特率
	S1_USE_P30P31();//UART1 使用P30 P31口	默认
	ES  = 1;	//允许中断
	REN = 1;	//允许接收
	EA = 1;

	B_TX1_Busy = 0;
}
//========================================================================
// 函数: void print_string(u8 *puts)
// 描述: 串口1发送字符串函数。
// 参数: puts:  字符串指针.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================

void print_string(LogLevel level,u8 *puts)
{
	if(level<=LOG_LEVEL)
		return;
	for (; *puts != 0;	puts++)   	//遇到停止符*结束
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
  for (; *puts != 0;	puts++)   	//遇到停止符0结束
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
* 初始化数据
*******************************************************************************/
static void InitData(void)
{
	
}
/******************************************************************************/
/*       Task 0 'job0':  RTX-51 tiny starts execution with task 0             */
/******************************************************************************/

void init (void) _task_ INIT_0{  	
	Sys_init();	
	Clear_WS2811();//初始化WS2811
	uart1_config();
	puts_to_SerialPort(INFO,"I AM SLC8F1K08_M3!\n");
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);
	/*路数：3，模式：1，速度：10*/
	SysRun.LedNum   = E2PROM_Strings[LED_NUM_CFG]; //获取led路数
	SysRun.LedMode  = E2PROM_Strings[RUN_TYPE_CFG];//获取运行模式
	SysRun.LedSpeed = E2PROM_Strings[SPEED_CFG];   //获取运行速度
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
		//os_wait2(K_TMO, 10);//10ms 一次
		Gpio_KeyScan(&KeyCode);
		switch(KeyCode)
		{
			case KeySpeed:
				if(SysRun.Mode == SET_MODE)
				{
					//LED++
					SysRun.LedNum = SysRun.LedNum+LED_NUM_STEP;
					SysRun.LedNum = is_max(LED_NUM_MIN,LED_NUM_MAX,SysRun.LedNum);//大于最大恢复最小
					printss(DEBUG,"LED num:",SysRun.LedNum);
				}
				else
				{
					//Speed++;
					SysRun.LedSpeed = SysRun.LedSpeed-SPEED_STEP;	
					SysRun.LedSpeed = is_min(SPEED_MIN,SPEED_MAX,SysRun.LedSpeed);      //小于最小恢复最大	
					printss(DEBUG,"LED Speed:",SysRun.LedSpeed);					
				}
				E2promErase = TRUE;//需要存储
				break;
			case KeyRunMode:
				if(SysRun.Mode == SET_MODE)
				{
					//LED--
					SysRun.LedNum = SysRun.LedNum-LED_NUM_STEP;
					SysRun.LedNum = is_min(LED_NUM_MIN,LED_NUM_MAX,SysRun.LedNum);        //小于最小恢复最大
					printss(DEBUG,"LED num:",SysRun.LedNum);
				}
				else
				{
					//Mode++;
					SysRun.LedMode = SysRun.LedMode+LED_TYPE_STEP;
					SysRun.LedMode = is_max(LED_TYPE_MIN,LED_TYPE_MAX,SysRun.LedMode);		//大于最大恢复最小
					printss(DEBUG,"LED mode:",SysRun.LedMode);
				}
				E2promErase = TRUE;//需要存储
				break;
			default: 
				break;
		}
			if(KeyCode != KeyNull){			               
				os_delete_task (LIGHTS_2);
				Clear_WS2811();//初始化WS2811
				os_create_task (LIGHTS_2);   
				LedFlash = TRUE;
				KeyCode = KeyNull;
			}
		E2PROM_Strings[LED_NUM_CFG]  = SysRun.LedNum;  //获取led路数
		E2PROM_Strings[RUN_TYPE_CFG] = SysRun.LedMode; //获取运行模式
		E2PROM_Strings[SPEED_CFG]    = SysRun.LedSpeed;//获取运行速度
		vEepromUpdate();
		Key_EventProtect = FALSE;//防止键值未处理时又触发新按键	
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
			case SET_LINE://设置模式
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
// 函数: void uart1_int (void) interrupt UART1_VECTOR
// 描述: UART1中断函数。
// 参数: nine.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
void uart1_int (void) interrupt UART1_VECTOR
{
	EA =0;
	if(RI)
	{
		RI = 0;
		Rec_Buf[RX_CONT] = SBUF;                    //把串口1缓存SBUF寄存器数据依次存放到数组Rec_Buf中
		RX_CONT++;                                   
		if(RX_CONT>Buf_Max)                          //接收数大于定义接收数组最大个数时，覆盖接收数组之前值
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