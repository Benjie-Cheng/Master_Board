/******************************************************************************/
/*                                                                            */
/*                   RTX_EX1.C:  The first RTX-51 Program                     */
/*                                                                            */
/******************************************************************************/
/*************	XXX_SLC104S_M3_V10	**************/
/*
万家灯火控制器说明：
1、路数设置：开机过程3键同时按,进入路数设置模式，设置完成后，重新上电即可。速度按键-路数，模式键+路数。
2、按键触发后均可以将数据存储
2、设置模式可以串口观察数据。
3、
4、

硬件资源：
1、WS2811+红外遥控+实体按键3个
2、
3、
4、



已经解决：
1、多个控制板同一个按键受控问题需要解决-PT2272和遥控硬件地址编码解决。
2、长按时间设置连续触发。软件完成。
3、添加遥控控制端口使能功能，写入eeprom保存，bin文件可以设置端口是否使用。

存在缺陷：
1、
2、
*/
//#include <rtx51tny.h>                 /* RTX-51 tiny functions & defines      */
#include "system.h"
#include "eeprom.h"

#define SysTick 10   //10ms
#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //接收串口1缓存数组
u8 RX_CONT = 0; 
BOOL B_TX1_Busy;  //发送忙标志
static u8 KeyCode = 0;
BOOL Key_EventProtect = FALSE;
BOOL E2promErase = FALSE;//e2prom 更新标志
BOOL LedFlash = FALSE;//闪烁灯允许运行标志
BOOL DebugMode = FALSE;//设置模式需要打印log
typedef struct{
	u16 Second; //秒
	u8  Minute; //分
	u8  Hour; //时
	u16 Day; //天
	u8 Flag;
} RtcStruct;
RtcStruct idata Rtc; //定义RTC 结构体

//--------------------------------------
static u8 	E2PROM_Strings[MAX_CFG] = {0x03,0x01,0x05};//data0：LED num，data1：mode data2：speed
/*
data0：LED路数
data1：花样模式
data2：运行速度
*/
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void printf(u8 *puts,u8 num1);
void Sys_init (void)  {
	/*104S-SOP8 只有P3*/
	//P0n_standard(0xff);	//设置为准双向口
	//P1n_standard(0xff);	//设置为准双向口
	//P2n_push_pull(0xff);//设置为准双向口
	P3n_standard(0xff);	//设置为准双向口
	//P4n_standard(0xff);	//设置为准双向口
	//P5n_standard(0xff);	//设置为准双向口
	if(!SPEED_GPIO&&(!MODE_GPIO)&&(!NLED_GPIO))//如果开机过程三键齐按，则进入设置led个数模式。
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
	static unsigned char Su8KeyLock1; //1 号按键的自锁
	static unsigned char Su8KeyCnt1; //1 号按键的计时器	
	static unsigned char uiKeyCtntyCnt1;
	static unsigned char Su8KeyLock2; //1 号按键的自锁
	static unsigned char Su8KeyCnt2; //1 号按键的计时器	
	static unsigned char uiKeyCtntyCnt2;
	
	//*KeyVal = (u8)value;
	if(Key_EventProtect)
		return 0;
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
		if(Su8KeyCnt1>=KEY_FILTER_TIME)
		{
			Su8KeyLock1=1; 
			Su8KeyCnt1=0;		
			*KeyVal = (u8)KeySpeed;    //触发1号键
			if(SysRun.Mode != SET_MODE)//如果不是设置模式不支持连续触发
			{
				Key_EventProtect = TRUE;
				Su8KeyLock1 = 2;
				return 0;
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
			Su8KeyCnt2=0；
			*KeyVal = (u8)KeyRunMode    //触发1号键
			if(SysRun.Mode != SET_MODE)//如果不是设置模式不支持连续触发
			{
				Key_EventProtect = TRUE;
				Su8KeyLock2 =2;
				return 0;
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
	SysRun.LedNum = E2PROM_Strings[LED_NUM_CFG];//获取led路数
	SysRun.LedMode = E2PROM_Strings[RUN_MODE_CFG];//获取运行模式
	SysRun.LedSpeed = E2PROM_Strings[SPEED_CFG];//获取运行速度
	E2promErase = FALSE;
}
void TaskDisplayScan(void)//10ms 刷新一次
{ 	
	Gpio_KeyScan(&KeyCode);
	if(!KeyCode){
		LedFlash = TRUE;//有按键按下需要闪烁一次LED提示
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
				SysRun.LedNum = is_max(1,254,SysRun.LedNum);//大于最大恢复最小
				printf("LED num:",SysRun.LedNum);
			}
			else
			{
				//Speed++;
				SysRun.LedSpeed++;	
				SysRun.LedSpeed = is_max(1,10,SysRun.LedSpeed);//大于最大恢复最小				
			}
			E2promErase = TRUE;//需要存储
			break;
		case KeyRunMode:
			KeyCode = KeyNull;
			if(SysRun.Mode == SET_MODE)
			{
				//LED--
				SysRun.LedNum--;
				SysRun.LedNum = is_min(1,255,SysRun.LedNum);//小于最小恢复最大
				printf("LED num:",SysRun.LedNum);
			}
			else
			{
				//Mode++;
				SysRun.LedMode++;
				is_max(0,10,SysRun.LedMode);		//大于最大恢复最小
				
			}
			E2promErase = TRUE;//需要存储
			break;
		default: 
		break;
	}
	E2PROM_Strings[LED_NUM_CFG] = SysRun.LedNum;//获取led路数
	E2PROM_Strings[RUN_MODE_CFG] = SysRun.LedMode;//获取运行模式
	E2PROM_Strings[SPEED_CFG] = SysRun.LedSpeed;//获取运行速度
	vEepromUpdate();
	Key_EventProtect = FALSE;//防止键值未处理时又触发新按键	
	os_wait2 (K_SIG|K_TMO, 1);	
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

void print_string(u8 *puts)
{
	if(!DebugMode)
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
	if(!DebugMode)
		return;
	SBUF = dat;
	B_TX1_Busy = 1;
	while(B_TX1_Busy);
}
void puts_to_SerialPort(u8 *puts)
{
    for (; *puts != 0;	puts++)   	//遇到停止符0结束
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
	Clear_WS2811();//初始化WS2811
	uart1_config();
	puts_to_SerialPort("I AM SLC104S_M3!\n");
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,MAX_CFG);
	SysRun.LedNum = E2PROM_Strings[LED_NUM_CFG];//获取led路数
	SysRun.LedMode = E2PROM_Strings[RUN_MODE_CFG];//获取运行模式
	SysRun.LedSpeed = E2PROM_Strings[SPEED_CFG];//获取运行速度
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
		TaskDisplayScan();//10ms 刷新一次
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