/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了版权 ---------------   */
/*---------------------------------------------------------------------*/
/*************	XXX_MC204S_K8_V10	**************
可控硅延时控制器:具体功能见spec

1、TM1650 驱动四位共阴极数码管，实现S1-S5 键盘。
2、595控制按键LED灯显示和光耦控制。
3、PT2272 遥控功能接口。
4、BT05蓝牙uart接口。

存在缺陷：
1、时间设置为0的时候，会乱码，范围应该在1~99。
2、长按连续触发。

v1.0：软件设定
1、波特率115200 ，晶振35M          
2、
******************************************/
#include "config.h"
#include "debug.h"
#include "eeprom.h"
#include "TM1650_I2C.h"

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 中断频率, 1000次/秒
#define	Baudrate1	115200UL                                //通信波特率115200


	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd

typedef enum {
	KeyOnTime=1,
	KeyOffTime,
	KeyRunMode,
	KeySetMode,
	KeyBright,
}KeyEnum;
typedef enum {
	ON       =0x01,
	OFF      =0x00,
	ON_ALL   =0xff,
	OFF_ALL  =0xfe,
	OFF_ON   =0xfd,	
}LogicType;
typedef enum {
	DualMinBit=2,
	DualSecBit=4,
	OnlyMinBit=3,
	SetModeBit=1,
	RunModeBit=0,
}LedBit;
typedef enum {
	COM1=0,
	COM2=1,
	COM3=2,
	COM4=3,
	COM5=4,
	COM6=5,
	OUT1=6,
	OUT2=7,
	COMX=8
}Port;

volatile unsigned char vGu8KeySec=0;  //按键的触发序号


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
	
typedef struct{
	static u8 Step = 0;               //运行步骤,switch 切换
	static u8 Mode = DUAL_MINU_MODE;  //运行模式 switch 切换，掉电需要记录到EEPROM中	
}SysRun;
static u8 EraseStep = 0; //扇区擦除步骤
static u8 Brightness = 8;

#define     SET_TIME_MAX 99  //最大值
#define     BLMAX 8//背光最大
#define     BLMIM 1//背光最小
#define     E2PROM_LENGTH 4
static u8 	E2PROM_Strings[E2PROM_LENGTH] = {0x00,0x00,0x00,0x01};
/*
data0：通时间
data1：关时间
data2：运行模式
data3:支持PT2272
*/

BOOL LedFlash = TRUE;//闪烁灯允许运行标志
BOOL E2promErase = FALSE;//e2prom 更新标志
BOOL DotFlag = TRUE;//数码管点闪烁灯允许运行标志
BOOL E2promDis = TRUE;//显示EEPROM 时间
BOOL Key_EventProtect = FALSE;

typedef struct{
	static u8 on = 0;
	static u8 off = 0;		
}SetTime;//设置通断的时间结构体，需要写入EEPROM中的时间
typedef struct{
	static u8 on = 0;
	static u8 off = 0;
	static u16 tMode = 0;//用于双秒，双分切换的中间变量	
}RunTime;//运行时间结构体
static u8   Support_Pt2272 = 0;

#define KEY_FILTER_TIME 20 //滤波的“ 稳定时间” 20ms

#define KEY_TIME_1S  1000  //时间是 10000ms
#define KEY_TIME_2S  500  //时间是 10000ms
#define KEY_TIME_025S 80  //0.25秒钟的时间需要的定时中断次数
#define KEY_TIME_60S 60000 //时间是 60000ms


static u8 	Display_Code[2]={0xff,0xff};		    //595，光耦和led状态，低电平点亮。
static u8 	LED8[4] = {0x00,0x00,0x00,0x00};		//显示缓冲支持四位

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //接收串口1缓存数组
u8 RX_CONT = 0; 
u8 code t_display[]={						//共阴极标准字库，共阳取反
//	 0    1    2    3    4    5    6    7    8    9  
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};	
u8 code t_display1[]={						//共阴极标准字库，共阳取反
//	0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	
u8 code t_display2[]={						//共阴极标准字库，共阳取反
    //A    B    C    D    E    F   black -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x77,0x7C,0x39,0x5E,0x79,0x71,0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e};
u8 code KeyVal[] = {0xDF,0xD7,0xC7,0xCF};
u8 code pt2272[] = {
//   1     2    3    4    5    6    7    8   9    10   11   12   13   14   15
	0x01,0x02,0x04,0x0a,0x0b,0x06,0x07,0x0c,0x09,0x03,0x0e,0x0d,0x0f,0x08,0x05	
};
/********************** A 给指示灯用的595 ************************/
sbit	A_HC595_SER   = P1^3;	//pin 55	SER		data input
sbit	A_HC595_RCLK  = P1^4;	//pin 15	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P1^2;	//pin 54	SRCLK	Shift data clock
//sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		低电平 使能enable pin
sbit	A_HC595_MR    = P1^5;	//pin 32	低电平复位

#define PT2272_D0      P11
#define PT2272_D1      P10
#define PT2272_D2      P37
#define PT2272_D3      P36
//#define PT2272_DATA    (PT2272_D0 + PT2272_D1*2)// + PT2272_D2*4 + PT2272_D3*8)

void vDataIn595(u8 dat);
void vDataOut595(void);
void uart1_config();	// 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void KeyProcess(KeyEnum key);
void TaskDisplayScan(void);//显示扫描
void vTaskfFlashLed(void);//led 闪烁
void TaskLedRunScan(void);//跑马灯
void vEepromUpdate(void);//EEPROM擦写
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
	{0, 11, 11, vEepromUpdate},					// e2prom写操作，50ms
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
	A_HC595_MR = 1;     //先清零数据输出
	timer0_init();
	uart1_config();
	A_HC595_MR = 1;     //复位禁止
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
void vDataOut595(void)
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
void ComX_Set(LogicType type,Port Xbit)
{
	u8 val;
	val = Display_Code[0];//低四位每次清零，用于数码管位扫描
	if(type==ON)
		Display_Code[0] = clrbit(val,Xbit);//低电平导通
	else if(type==OFF)
		Display_Code[0] = setbit(val,Xbit);	
	else if(ON_ALL == type)	
		Display_Code[0] = 0x00;
	else if(OFF_ALL == type)
		Display_Code[0] = 0xff;
	else if(OFF_ON == type)
		Display_Code[0] = reversebit(val,Xbit);//翻转
}
void LedX_Set(LogicType type,LedBit Xbit)
{
	u8 val;
	val = Display_Code[1];//低四位每次清零，用于数码管位扫描
	if(type==ON)
		Display_Code[1] = clrbit(val,Xbit);//低电平导通
	else if(type==OFF)
		Display_Code[1] = setbit(val,Xbit);	
	else if(ON_ALL == type)	
		Display_Code[1] = 0x00;
	else if(OFF_ALL == type)
		Display_Code[1] = 0xff;
	else if(OFF_ON == type)
		Display_Code[1] = reversebit(val,Xbit);//翻转
}
void Date_Transform(u8 num1,u8 num2)
{	
	if(SysRun.Step==IDLE_MODE)//idle模式只跑马灯，不做数据显示
	{
		LedX_Set(OFF_ALL,7);//关闭所有LED;
		return ;
	}
	LED8[0] = t_display[mod(num1,10)];
	LED8[1] = t_display[rem(num1,10)];
	LED8[2] = t_display[mod(num2,10)];	
	LED8[3] = t_display[rem(num2,10)];
	
	if(DotFlag && (E2promDis == FALSE)){//运行状态，闪烁小数点
		if(SysRun.Step==TURN_ON_MODE)
			LED8[1] = t_display1[rem(num1,10)];//导通运行闪烁
		else
			LED8[3] = t_display1[rem(num2,10)];//关闭运行闪烁
	}
	if(SysRun.Mode == ONLY_MINU_MODE)//如果是单分模式关闭关断显示
	{
		LED8[2] = t_display2[6];	
		LED8[3] = t_display2[6];
	}
}
void TaskDisplayScan(void)//10ms 刷新一次
{ 
	if(E2promDis)
		Date_Transform(E2PROM_Strings[0],E2PROM_Strings[1]);
	else 
		Date_Transform(RunTime.on,RunTime.off);
	TM1650_Set(DIG1,LED8[0]);
	TM1650_Set(DIG2,LED8[1]);
	TM1650_Set(DIG3,LED8[2]);
	TM1650_Set(DIG4,LED8[3]);
	vDataIn595(Display_Code[1]);//输出LED控制
	vDataIn595(Display_Code[0]);//输出光耦控制
	vDataOut595();				//锁存输出数据
}
void vTaskfFlashLed(void)
{ 
	if(!E2promDis)
	{
		LedX_Set(OFF,RunModeBit);//正常运行时设置灯关闭;	
		return;
	}	
	if(LedFlash)
		LedX_Set(OFF_ON,RunModeBit);//设置灯闪烁;	
		//key_led_reverse();
	//print_char(vGu8KeySec);//打印按键值
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
	if(!E2promErase)//更新标志为假不更新
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
			RunTime.on = E2PROM_Strings[0];
			RunTime.off = E2PROM_Strings[1];
			SysRun.Mode = E2PROM_Strings[2];
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
		return; //按键的触发序号是0意味着无按键触发，不执行此函数下面的代码
	}
	switch(key) //根据不同的按键触发序号执行对应的代码
	{
		case KeyOffTime:     //【关断时间设置】
			key=0;
			if(!E2promDis||SysRun.Mode == ONLY_MINU_MODE)//单分和运行模式不设置关断时间
			{
				break;
			}
			LedX_Set(OFF_ON,SetModeBit);//【设置灯】取反;	
			if(++SetTime.off>SET_TIME_MAX)
				SetTime.off = 1;
			E2promErase = TRUE;
			break;
		case KeyOnTime:     //【开启时间设置】
			key=0;
			if(!E2promDis)//运行模式不设置时间
			{
				break;
			}
			LedX_Set(OFF_ON,SetModeBit);//【设置灯】取反;
			if(++SetTime.on>SET_TIME_MAX)
				SetTime.on = 1;  
			E2promErase = TRUE;
			break;
		case KeyRunMode:     //【运行模式】
			key=0; 
			if(++SysRun.Mode>DUAL_SEC_MODE)
				SysRun.Mode = DUAL_MINU_MODE;
			if(SysRun.Mode == ONLY_MINU_MODE)//如果是单分模式关断时间为1;
				SetTime.off = 1;
			E2promErase = TRUE;
			break;

		case KeyBright:     //【背光设置】
			//LedX_Set(OFF_ON,SetModeBit);//【设置灯】取反;
			key=0; 
			if(--Brightness<BLMIM)
				Brightness = 8; 
			//Tube_CMD(P7_MODE,Brightness);
			
			break;
		case KeySetMode:     //【设置模式】
			key=0; 
			LedX_Set(OFF_ON,SetModeBit);//【设置灯】取反;	
			E2promDis = ~E2promDis;
			break;
		default:     
			break;
	}
	if(E2promErase)
	{
		vGu8TimeFlag_2 = 0;
		vGu32TimeCnt_2 = 0;//有连续触发时eeprom操作倒计时不开始
		EraseStep = 0;//按擦除步骤奔跑
		vGu8TimeFlag_1 = 0;//eprom需要操作时，不执行倒计时
		vGu8TimeFlag_2 = 0;
		SysRun.Step = TURN_ON_MODE;//重新运行
	}
	E2PROM_Strings[0] = SetTime.on;
	E2PROM_Strings[1] = SetTime.off;
	E2PROM_Strings[2] = SysRun.Mode;
	Key_EventProtect = FALSE;
}		
	
void Channle_Sw(void)
{
	switch(SysRun.Mode)
	{
		case DUAL_MINU_MODE:
			RunTime.tMode = KEY_TIME_60S;//双分,60s
			LedX_Set(ON,DualMinBit);  //【LED5】双分灯亮;
			LedX_Set(OFF,DualSecBit); //【LED5】取反;
			LedX_Set(OFF,OnlyMinBit); //【LED5】取反;
			break;
		case ONLY_MINU_MODE:
			RunTime.tMode = KEY_TIME_60S;//单分,60s
			LedX_Set(ON,OnlyMinBit);  //【LED5】取反;
			LedX_Set(OFF,DualMinBit); //【LED5】取反;
			LedX_Set(OFF,DualSecBit); //【LED5】取反;
			break;
		case DUAL_SEC_MODE:
			RunTime.tMode = KEY_TIME_1S;//双秒,1s
			LedX_Set(ON,DualSecBit);  //【LED5】取反;
			LedX_Set(OFF,DualMinBit); //【LED5】取反;
			LedX_Set(OFF,OnlyMinBit); //【LED5】取反;
			break;
		default:	
			break;
	}
	switch(SysRun.Step)
	{
		case TURN_ON_MODE://导通倒计时
			vGu8TimeFlag_1 = 1;
			ComX_Set(ON,OUT2); //【OUT2】开;
			ComX_Set(OFF,OUT1);//【OUT1】关;
			ComX_Set(ON,0);    //【】开;
			ComX_Set(OFF,1);   //【】关;
			if(vGu32TimeCnt_1>=RunTime.tMode)
			{
				vGu32TimeCnt_1 = 0;
				RunTime.on--;
				if(RunTime.on<=0){
					SysRun.Step++;
					vGu8TimeFlag_1 = 0;
					RunTime.off = E2PROM_Strings[1];//赋值EEPROM值
					if(SysRun.Mode == ONLY_MINU_MODE)//如果是单分模式则，时间到关闭运行。
						SysRun.Step = IDLE_MODE;
				}
			}	
			break;
		case TURN_OFF_MODE://关断倒计时
			vGu8TimeFlag_1 = 1;
			ComX_Set(ON,OUT1); //【OUT1】开;
			ComX_Set(OFF,OUT2);//【OUT2】关;
			ComX_Set(ON,1);    //【】开;
			ComX_Set(OFF,0);   //【】关;
			if(vGu32TimeCnt_1>=RunTime.tMode)
			{
				vGu32TimeCnt_1 = 0;
				RunTime.off--;
				if(RunTime.off <= 0){
					SysRun.Step = TURN_ON_MODE;
					vGu8TimeFlag_1 = 0;
					RunTime.on = E2PROM_Strings[0];//赋值EEPROM值
				}
			}
			break;
		case IDLE_MODE://单分关断模式状态
			ComX_Set(OFF_ALL,COMX);//【关闭所有端口】;
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
	if(!Key_Code)//如果为0则未触发
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
			if(Key_Code == S1 || Key_Code == S2)
				Key_EventProtect = FALSE;//不需要按键保护
			else
				Key_EventProtect = TRUE;//需要按键保护
			return (Key_Code);
		}
	}
	else if(Su8KeyLock2)//如果连续按下
	{
		if(Key_Code!=S1 && Key_Code!=S2)//不是S1,S2不再触发
			return 0;
		else
		{	
			if(Su16KeyCnt2 < KEY_TIME_1S)
				 Su16KeyCnt2++;
			else//按住累加到1秒后仍然不放手，这个时候进入有节奏的连续触发
			{
				uiKeyCtntyCnt2++; //连续触发延时计数器累加
				if(uiKeyCtntyCnt2>KEY_TIME_025S)  //按住没松手，每0.25秒就触发一次
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
	
	if(val==0){
		Su8KeyLock1=0;
		Su16KeyCnt1=0; 
		uiKeyCtntyCnt1=0; //连续累加的时间间隔延时计数器清零	
		return 0;
	}
	else if(0==Su8KeyLock1)
	{
		Su8KeyLock1 = 1;
		if(val == S1 || val == S2)
			Key_EventProtect = FALSE;//不需要按键保护
		else
			Key_EventProtect = TRUE;//需要按键保护
		return val;
	}
	else if(val!=S1 && val!=S2)
	{
		return 0;
	}
	else if(Su16KeyCnt1 < KEY_TIME_1S)
	{
		Su16KeyCnt1++;
	}
	else//按住累加到1秒后仍然不放手，这个时候进入有节奏的连续触发
	{
		uiKeyCtntyCnt1++; //连续触发延时计数器累加
		 if(uiKeyCtntyCnt1>KEY_TIME_025S)  //按住没松手，每0.25秒就触发一次
		 {
			uiKeyCtntyCnt1=0; 
			return val;
		 }
	}		
	return 0;	
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
	vDataIn595(0xff);
	vDataIn595(0xff);
	vDataOut595();	//开机默认关闭通道显示LED
	puts_to_SerialPort("I am MC204S_K8!\n");
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,E2PROM_LENGTH);
	SetTime.on  = RunTime.on = E2PROM_Strings[0];
	SetTime.off = RunTime.off = E2PROM_Strings[1];
	SysRun.Mode = E2PROM_Strings[2];
	Support_Pt2272 = E2PROM_Strings[3];
	
	Init_Tm1650();//数码管开显示
	TM1650_Set(DIG1,t_display[0]);
	TM1650_Set(DIG2,t_display[1]);
	TM1650_Set(DIG3,t_display[2]);
	TM1650_Set(DIG4,t_display[3]);

	while(1)
	{
		if(B_1ms)	//1ms到
		{
			B_1ms = 0;	
			vGu8KeySec = Get_KeyVal();//获取TM1650按键值
			if(IDLE_MODE==SysRun.Step)//如果进入idle 模式，按键无效，只能重启断电才生效。
				vGu8KeySec=0;
			Channle_Sw();
			KeyProcess(KeyEnum)vGu8KeySec)    //按键的任务函数
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
	//KeyScan();
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