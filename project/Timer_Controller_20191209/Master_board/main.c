/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了版权 ---------------   */
/*---------------------------------------------------------------------*/
/*************	本程序功能说明	**************
8K8R:8个key控制8个Relay，具体功能见spec

用STC的MCU的IO方式矩阵键盘交互和串口收发数据。
1、8*8 矩阵键暂时只调试并用了一行。
2、按键板有按键触发时，串口发送K1-K8按键控制的led 状态给主机。主机暂时无反馈状态给从机（掉包时状态会错乱，但是从新按一次能恢复）。
3、从机根据按键值，通过74HC595控制板上LED状态。
4、有按键按下时有一颗公用的LED指示亮，松手灭，不按时正常1Hz闪烁。

存在缺陷：
1、8*8键盘的LED灯在按键右侧，每次按下均被手指遮挡，难以分辨继电器状态。改在右侧或者上侧。
2、LED驱动都为高电平驱动，存在驱动能力不住，可采用外部上拉，低电平点亮方式

v1.1：
1、修改波特率115200->9600           20190929
2、统一通信数据格式                 20190929
******************************************/
#include "config.h"
#include "debug.h"
#include "eeprom.h"

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 中断频率, 1000次/秒
#define	Baudrate1	115200UL                                //通信波特率115200


	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd
	#define RED_LED     2
	#define YELLOW_LED  1
	#define GREEN_LED   0
	#define CAR_LED     3
	

volatile unsigned char vGu8KeySec=0;  //按键的触发序号
//主板5个按键定义
#define	KEY1_GPIO P12
#define	KEY2_GPIO P13
#define	KEY3_GPIO P14
#define	KEY4_GPIO P15
#define	KEY5_GPIO P33

//========================================================================
/*************	本地变量声明	**************/
/********************** 公用量 ************************/
BOOL B_1ms;	 //1ms标志
BOOL B_TX1_Busy;  //发送忙标志

enum Gu8Step{
	TURN_ON_MODE=0,
	TURN_OFF_MODE,
	IDLE_MODE,
};
enum Run_Mode{
	DUAL_MINU_MODE=0,
	ONLY_MINU_MODE,
	DUAL_SEC_MODE,
};

//“软件定时器 1” 的相关变量
volatile unsigned char vGu8TimeFlag_1=0;
volatile u32 vGu32TimeCnt_1=0;	
volatile unsigned char vGu8TimeFlag_2=0;
volatile u32 vGu32TimeCnt_2=0;
	
static u8 Gu8Step = 0;   //软件定时器 1 的 switch 切换步骤
static u8 Run_Mode = DUAL_MINU_MODE;  //运行模式 switch 切换步骤，掉电需要记录到EEPROM中
static u8 EraseStep = 0; //扇区擦除步骤


#define     SET_TIME_MAX 20  //最大值
#define     E2PROM_LENGTH 3
static u8 	E2PROM_Strings[E2PROM_LENGTH] = {0x00,0x00,0x00};//data0：通时间，data1：关时间.data2：运行模式

BOOL led_flash_flag = TRUE;//闪烁灯允许运行标志
BOOL e2prom_update_flag = FALSE;//e2prom 更新标志
BOOL dot_flag = TRUE;//数码管点闪烁灯允许运行标志
BOOL e2prom_display = TRUE;//显示EEPROM 时间
BOOL Key_EventProtect = FALSE;

static u16  Delay_Time = 0;       //用于双秒，双分切换的中间变量
static u8   on_time = 0,off_time = 0;//运行时间
static u8   on_time_set = 0,off_time_set = 0;//需要写入EEPROM中的时间


#define KEY_FILTER_TIME 20 //滤波的“ 稳定时间” 20ms

#define KEY_TIME_1S  1000  //时间是 10000ms
#define KEY_TIME_2S  1000  //时间是 10000ms
#define KEY_TIME_025S 250  //0.25秒钟的时间需要的定时中断次数
#define KEY_TIME_60S 60000 //时间是 60000ms


#define	INDEX_MAX 4	//显示位索引
static u8 	Display_Code[2]={0x00,0x00};		//数码管段码和位码，继电器和led状态。
static u8 	LED8[4] = {0x00,0x00,0x00,0x00};		//显示缓冲支持四位
static u8	display_index = 0;						//显示位索引	

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //接收串口1缓存数组
u8 RX_CONT = 0; 
u8 code t_display[]={						//共阴极标准字库，共阳取反
//	 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,
//black	 -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e,
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	//0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1


/********************** A 给指示灯用的595 ************************/
sbit	A_HC595_SER   = P1^1;	//pin 34	SER		data input
sbit	A_HC595_RCLK  = P3^2;	//pin 37	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P1^0;	//pin 35	SRCLK	Shift data clock
//sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		低电平 使能enable pin
//sbit	A_HC595_MR    = P3^6;	//pin 36	低电平复位	

void uart1_config();	// 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void KeyScan(void);
void KeyTask(void);
void TaskDisplayScan(void);
void vTaskfFlashLed(void);
void TaskLedRunScan(void);
void vEepromUpdate(void);
void Date_Transform(u8 num1,u8 num2);
	
typedef struct _TASK_COMPONENTS
{
	u8 Run;                 // 程序运行标记：0-不运行，1运行
	u16 Timer;               // 计时器
	u16 ItvTime;             // 任务运行间隔时间
	void (*TaskHook)(void); // 要运行的任务函数
} TASK_COMPONENTS;   
static TASK_COMPONENTS TaskComps[] =
{
	{0, 1000,  1000, vTaskfFlashLed},           // 按键扫描1s
	{0, 100, 100, TaskLedRunScan},         		// 跑马灯
	{0, 50, 50, vEepromUpdate},					// e2prom写操作，50ms
//	{0, 10, 10, TaskRTC},				        // RTC倒计时
//	{0, 30, 30, TaskDispStatus},
	// 这里添加你的任务
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // 运行LED
	TAST_LED_RUN,          //跑马灯
	TAST_E2PROM_RUN,        //E2PROM 运行
//	TASK_KEY_SERV,
//	TASK_RTC,
//	TASK_DISP_WS,             // 工作状态显示// 这里添加你的任务。。。。
	// 这里添加你的任务
	TASKS_MAX                 // 总的可供分配的定时任务数目
} TASK_LIST;

//========================================================================
// 描述: u8 ms延时函数。
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
	EA = 1;		//打开总中断
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

//========================================================================
// 描述: 硬件初始化 。
//========================================================================
void stc15x_hw_init(void)
{
	P0n_standard(0xff);	//设置为准双向口
	P1n_standard(0xff);	//设置为准双向口
	P2n_push_pull(0xff);	//设置为准双向口
	P3n_standard(0xff);	//设置为准双向口
	P4n_standard(0xff);	//设置为准双向口
	P5n_standard(0xff);	//设置为准双向口	
	timer0_init();
	uart1_config();
	//A_HC595_MR = 1;//复位禁止
	//A_HC595_OE = 0;//使能芯片
	KEY1_GPIO = 1;//按键引脚，默认拉高电平
	KEY2_GPIO = 1;//按键引脚，默认拉高电平
	KEY3_GPIO = 1;//按键引脚，默认拉高电平
	KEY4_GPIO = 1;//按键引脚，默认拉高电平
	KEY5_GPIO = 1;//按键引脚，默认拉高电平
}
/**************** 向HC595发送一个字节函数 ******************/
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
/**************** HC595数据锁存函数 ******************/
void vDataOut595()
{		
	A_HC595_RCLK = 0;
	NOP2();
	A_HC595_RCLK = 1;
}

void TaskRemarks(void)
{
	u8 i;    
	for (i=0; i<TASKS_MAX; i++)           // 逐个任务时间处理
	{
		if (TaskComps[i].Timer)           // 时间不为0
		{
			TaskComps[i].Timer--;         // 减去一个节拍
			if (TaskComps[i].Timer == 0)  // 时间减完了
			{
				TaskComps[i].Timer = TaskComps[i].ItvTime;       // 恢复计时器值，从新下一次
				TaskComps[i].Run = 1;           // 任务可以运行	
			}
		}
	}
}
void TaskProcess(void)
{
	u8 i;    
	for (i=0; i<TASKS_MAX; i++)           // 逐个任务时间处理
	{
		if (TaskComps[i].Run)           // 时间不为0
		{
			TaskComps[i].TaskHook();         // 运行任务
			TaskComps[i].Run = 0;          // 标志清0
		}
	}
}
void BitX_Set(int status,u8 Xbit)
{
	u8 val;
	val = Display_Code[1]&0xf0;//低四位每次清零，用于数码管位扫描
	if(status==ON)
		Display_Code[1] = setbit(val,Xbit);//高电平导通
	else if(status==OFF)
		Display_Code[1] = clrbit(val,Xbit);	
	else if(ON_ALL == status)	
		Display_Code[1] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[1] = 0x00;
	else if(OFF_ON == status)
		Display_Code[1] = reversebit(val,Xbit);//翻转
}
void TaskDisplayScan(void)//10ms 刷新一次
{ 
	for(display_index = 0;display_index < INDEX_MAX;display_index++){
		//DisplayChange();//赋值缓存数值，并改变数码管com位
		BitX_Set(ON,display_index);//com 口扫描，点亮该位
#if 0
	_nop_();
	_nop_();
	BitX_Set(OFF,display_index);//改变亮度
#endif	
		if(e2prom_display)
			Date_Transform(E2PROM_Strings[0],E2PROM_Strings[1]);
		else 
			Date_Transform(on_time,off_time);
		Display_Code[0] = LED8[display_index];

		vDataIn595(Display_Code[1]);//输出位码
		vDataIn595(Display_Code[0]);//输出断码
		vDataOut595();				//锁存输出数据
	}	
}
void vTaskfFlashLed(void)
{ 
	if(led_flash_flag)
		key_led_reverse();
	puts_to_SerialPort(E2PROM_Strings);
	
}
void TaskLedRunScan(void)
{
	static u8 LED_RUN_STEP=0;
	
	LED_RUN_STEP++;
	dot_flag = ~dot_flag;
	LED8[0] = ~t_display[16];
	LED8[1] = ~t_display[16];
	switch(LED_RUN_STEP)
	{
	case 1:
		LED8[3] = ~0x01;
		LED8[2] = ~0x00;
		break;
	case 2:
		LED8[3] = ~0x02;
		LED8[2] = ~0x00;
		break;
	case 3:
		LED8[3] = ~0x04;
		LED8[2] = ~0x00;
		break;
	case 4:
		LED8[3] = ~0x08;
		LED8[2] = ~0x00;
		break;
	case 5:
		LED8[3] = ~0x00;
		LED8[2] = ~0x08;
		break;
	case 6:
		LED8[3] = ~0x00;
		LED8[2] = ~0x10;
		break;
	case 7:
		LED8[3] = ~0x00;
		LED8[2] = ~0x20;
		break;
	case 8:
		LED8[3] = ~0x00;
		LED8[2] = ~0x01;
		LED_RUN_STEP = 0;
		break;
	}
}
void vEepromUpdate(void)
{
	if(!e2prom_update_flag)//更新标志为假不更新
		return;
	switch(EraseStep) //分解写读扇区步骤
	{
		case 0:
			vGu8TimeFlag_2 = 1;
			if(vGu32TimeCnt_2>=KEY_TIME_2S)//松开按键2s后开始写入EEPROM
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
			EEPROM_write_n(IAP_ADDRESS,E2PROM_Strings,E2PROM_LENGTH);	
			EraseStep++;
		break;
		case 3: 	
			EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,E2PROM_LENGTH);
			on_time = E2PROM_Strings[0];
			off_time = E2PROM_Strings[1];
			Run_Mode = E2PROM_Strings[2];
			EraseStep = 0;
			e2prom_update_flag = FALSE;
			break;
		default: 
			break;
	}
}

void Date_Transform(u8 num1,u8 num2)
{	
	if(Gu8Step==IDLE_MODE)//停止状态让其跑马灯，不做数据显示
		return ;
	LED8[0] = ~t_display[mod(num1,10)];
	LED8[1] = ~t_display[rem(num1,10)];
	LED8[2] = ~t_display[mod(num2,10)];	
	LED8[3] = ~t_display[rem(num2,10)];
	
	if(dot_flag){
		if(Gu8Step==TURN_ON_MODE)
			LED8[1] &=0x7f;
		else
			LED8[3] &=0x7f;
	}
	if(Run_Mode == ONLY_MINU_MODE)//如果是单分模式关闭关断显示
	{
		LED8[2] = ~t_display[16];	
		LED8[3] = ~t_display[16];
	}
}
void Kled_Set(int status,u8 Nled)
{
	u8 val;
	val = Display_Code[1];
	if(status==ON)
		Display_Code[1] = setbit(val,Nled);//高电平导通
	else if(status==OFF)
		Display_Code[1] = clrbit(val,Nled);	
	else if(ON_ALL == status)	
		Display_Code[1] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[1] = 0x00;
	else if(OFF_ON == status)
		Display_Code[1] = reversebit(val,Nled);//翻转
}
void KeyScan(void)
{	
	static unsigned char Su8KeyLock1;
	static unsigned int  Su16KeyCnt1;
	static unsigned int  uiKeyCtntyCnt1;
	static unsigned char Su8KeyLock2;
	static unsigned int  Su16KeyCnt2;
	static unsigned int  uiKeyCtntyCnt2; 	
	static unsigned char Su8KeyLock3;
	static unsigned int  Su16KeyCnt3; 
	static unsigned char Su8KeyLock4;
	static unsigned int  Su16KeyCnt4; 
	static unsigned char Su8KeyLock5;
	static unsigned int  Su16KeyCnt5;

	if(Key_EventProtect)
		return;
    //【启动】按键K1的扫描识别
	if(0!=KEY1_GPIO)
	{
		Su8KeyLock1=0;
		Su16KeyCnt1=0; 
		uiKeyCtntyCnt1=0; //连续累加的时间间隔延时计数器清零		
	}
	else if(0==Su8KeyLock1)
	{
		Su16KeyCnt1++;
		if(Su16KeyCnt1>=KEY_FILTER_TIME)
		{
			Su8KeyLock1=1;  
			Su16KeyCnt1=0;
			vGu8KeySec=1;    //触发1号键
			Key_EventProtect = FALSE;//不需要按键保护
		}
	}
	else if(Su16KeyCnt1 < KEY_TIME_1S)//如果小于1s,继续+
	{
		Su16KeyCnt1++;
	}
	else//按住累加到1秒后仍然不放手，这个时候进入有节奏的连续触发
	{
		uiKeyCtntyCnt1++; //连续触发延时计数器累加
		 if(uiKeyCtntyCnt1>KEY_TIME_025S)  //按住没松手，每0.25秒就触发一次
		 {
			uiKeyCtntyCnt1=0; 
			vGu8KeySec=1;    //触发1号键
		 }
	}
   //【停止】按键K2的扫描识别
	if(0!=KEY2_GPIO)
	{
		Su8KeyLock2=0;
		Su16KeyCnt2=0; 
		uiKeyCtntyCnt2=0; //连续累加的时间间隔延时计数器清零
	}
	else if(0==Su8KeyLock2)
	{
		Su16KeyCnt2++;
		if(Su16KeyCnt2>=KEY_FILTER_TIME)
		{
			Su8KeyLock2=1;  
			vGu8KeySec=2;    //触发2号键
			Key_EventProtect = FALSE;
		}
	}
	else if(Su16KeyCnt2 < KEY_TIME_1S)//如果小于1s,继续+
	{
		Su16KeyCnt2++;
	}
	else//按住累加到1秒后仍然不放手，这个时候进入有节奏的连续触发
	{
		uiKeyCtntyCnt2++; //连续触发延时计数器累加
		 if(uiKeyCtntyCnt2>KEY_TIME_025S)  //按住没松手，每0.25秒就触发一次
		 {
			uiKeyCtntyCnt2=0; 
			vGu8KeySec=2;    //触发2号键
		 }
	}
   //【模式+】按键K3的扫描识别
	if(0!=KEY3_GPIO)
	{
		Su8KeyLock3=0;
		Su16KeyCnt3=0; 	
	}
	else if(0==Su8KeyLock3)
	{
		Su16KeyCnt3++;
		if(Su16KeyCnt3>=KEY_FILTER_TIME)
		{
			Su8KeyLock3=1;  
			vGu8KeySec=3;    //触发3号键
			Key_EventProtect = TRUE;
		}
	}
   //【模式-】按键K4的扫描识别
	if(0!=KEY4_GPIO)
	{
		Su8KeyLock4=0;
		Su16KeyCnt4=0; 	
	}
	else if(0==Su8KeyLock4)
	{
		Su16KeyCnt4++;
		if(Su16KeyCnt4>=KEY_FILTER_TIME)
		{
			Su8KeyLock4=1;  
			vGu8KeySec=4;    //触发4号键
			Key_EventProtect = TRUE;
		}
	}
   //【模式-】按键K4的扫描识别
	if(0!=KEY5_GPIO)
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
			vGu8KeySec=5;    //触发5号键
			Key_EventProtect = TRUE;
		}
	}	
}	
void KeyTask(void)
{
	//print_char(vGu8KeySec);
	if(0==vGu8KeySec)
	{
		return; //按键的触发序号是0意味着无按键触发，不执行此函数下面的代码
	}
	switch(vGu8KeySec) //根据不同的按键触发序号执行对应的代码
	{
		case 1:     //按键K1【关断时间设置】
			vGu8KeySec=0;	
			if(++off_time_set>SET_TIME_MAX)
				off_time_set = 0;
			e2prom_update_flag = TRUE;
			break;
		case 2:     //按键K2【开启时间设置】
			vGu8KeySec=0;
			if(++on_time_set>SET_TIME_MAX)
				on_time_set = 0;  
			e2prom_update_flag = TRUE;
			break;
		case 3:     //按键K3【模式设置】
			vGu8KeySec=0; 
			BitX_Set(OFF_ON,6);//【继电器3】取反;	
			if(++Run_Mode>DUAL_SEC_MODE)
				Run_Mode = DUAL_MINU_MODE;
			if(Run_Mode == ONLY_MINU_MODE)//如果是单分模式关断时间为0;
				off_time_set = 0;
			e2prom_update_flag = TRUE;
			break;
		case 4:     //按键K4
			vGu8KeySec=0; 
			BitX_Set(OFF_ON,5);//【继电器2】取反;
			BitX_Set(OFF_ON,7);//【LED状态】取反;
			break;
		case 5:     //按键K5【显示模式】
			vGu8KeySec=0; 
			e2prom_display = ~e2prom_display;
			break;
		default:     
			 
			break;
	}
	if(e2prom_update_flag)
	{
		EraseStep = 0;
		vGu8TimeFlag_1 = 0;//eprom需要操作时，不执行倒计时
		vGu8TimeFlag_2 = 0;
		Gu8Step = TURN_ON_MODE;//从新运行
	}
	E2PROM_Strings[0] = on_time_set;
	E2PROM_Strings[1] = off_time_set;
	E2PROM_Strings[2] = Run_Mode;
	Key_EventProtect = FALSE;
}	
void Channle_Sw(void)
{
	switch(Run_Mode)
	{
		case DUAL_MINU_MODE:
			Delay_Time = KEY_TIME_60S;//双分
			break;
		case ONLY_MINU_MODE:
			Delay_Time = KEY_TIME_60S;//单分
			break;
		case DUAL_SEC_MODE:
			Delay_Time = KEY_TIME_1S;//双秒
			break;
		default:	
			break;
	}
	switch(Gu8Step)
	{
		case TURN_ON_MODE://导通倒计时
			vGu8TimeFlag_1 = 1;
			BitX_Set(ON,4);//【继电器1】开;
			if(vGu32TimeCnt_1>=Delay_Time)
			{
				vGu32TimeCnt_1 = 0;
				on_time--;
				if(on_time<=0){
					Gu8Step++;
					vGu8TimeFlag_1 = 0;
					off_time = E2PROM_Strings[1];//赋值EEPROM值
					if(Run_Mode == ONLY_MINU_MODE)//如果是单分模式则，时间到关闭运行。
						Gu8Step = IDLE_MODE;
				}
			}	
			break;
		case TURN_OFF_MODE://关断倒计时
			vGu8TimeFlag_1 = 1;
			BitX_Set(OFF,4);//【继电器1】开;
			if(vGu32TimeCnt_1>=Delay_Time)
			{
				vGu32TimeCnt_1 = 0;
				off_time--;
				if(off_time <= 0){
					Gu8Step = TURN_ON_MODE;
					vGu8TimeFlag_1 = 0;
					on_time = E2PROM_Strings[0];//赋值EEPROM值
				}
			}
			break;
		case IDLE_MODE://单分关断模式状态
			//on_time = E2PROM_Strings[0];//赋值EEPROM，开启时间
				
			break;
			
		default:	
			break;
	}
}
//========================================================================
// 函数: void main(void)
// 描述: 主程序.
// 参数: 无.
// 返回: 无.
// 版本: V1.0, 2012-10-22
//========================================================================
void main(void)
{
	stc15x_hw_init();
	vDataIn595(0x00);
	vDataIn595(0x00);
	vDataOut595();	//开机默认关闭通道显示LED
	puts_to_SerialPort("I am Traffic Lights!\n");
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,E2PROM_LENGTH);
	on_time_set = on_time = E2PROM_Strings[0];
	off_time_set = off_time = E2PROM_Strings[1];
	Run_Mode = E2PROM_Strings[2];
	while(1)
	{
		if(B_1ms)	//1ms到
		{
			B_1ms = 0;	
			Channle_Sw();
			KeyTask();    //按键的任务函数
			TaskProcess();//1：led 2：数码管闪烁 3、e2prom读写
		}
		TaskDisplayScan();
	}
}

//========================================================================
// 描述: Timer0 1ms中断函数。
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	B_1ms = 1;		//1ms标志
	KeyScan();
	TaskRemarks();
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
  for (; *puts != '*';	puts++)   	//遇到停止符*结束
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
    for (; *puts != 0;	puts++)   	//遇到停止符0结束
	{
		SBUF = *puts;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
	}
}
//========================================================================
// 函数: void	uart1_config()
// 描述: UART1初始化函数。
// 参数: brt: 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
#if 1
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
#endif
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
}