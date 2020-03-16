/*---------------------------------------------------------------------*/
/* --- Fuctin: Mini traffic lights- -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了版权 ---------------   */
/*---------------------------------------------------------------------*/
/*************	本程序功能说明	**************
交通灯：0.28寸共阳极数码管，TXD,RXD P32、P55用于自定义开发。
单片机：STC15W201AS-SOP16,采用动态扫描方式节约IO
波特率：115200
调试日志：
1、
2、
3、
4、
******************************************/
#include "config.h"
#include "debug.h"

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))	//Timer 0 中断频率, 1000次/秒
#define	Baudrate1	115200UL                            //通信波特率115200


/*************	IO定义	**************/
sbit	LED8_A = P1^1;
sbit	LED8_B = P1^0;
sbit	LED8_C = P1^5;
sbit	LED8_D = P1^4;
sbit	LED8_E = P1^3;
sbit	LED8_F = P1^2;
sbit	LED8_G = P5^4;

sbit	P_COM0 = P3^6;
sbit	P_COM1 = P3^7;
sbit	P_COM2 = P3^3;

enum{
	LEDL_RED=5, 
	LEDL_GREEN=1,
	LEDR_RED=3, 
	LEDR_GREEN=2, 
	LEDG_RED=0,
	LEDG_GREEN=4,  	
};
typedef enum{
	ON=1,
	OFF=0,
	ON_ALL=0xff,
	OFF_ALL=0xfe,
	OFF_ON=0xfd	
}LogicType;
	
#define GPIO_CHECK_PIN P30
#define GPIO_OUT_PIN   P31

//========================================================================
/********************** 公用量 ************************/
BOOL B_1ms;	 //1ms标志
BOOL B_TX1_Busy;  //发送忙标志

//“软件定时器 1” 的相关变量
volatile unsigned char vGu8TimeFlag_1=0;
volatile u32 vGu32TimeCnt_1=0;	
volatile unsigned char vGu8TimeFlag_2=0;
volatile unsigned int vGu16TimeCnt_2=0;	
volatile unsigned char vGu8LeveCheckFlag=0;

#define GPIO_FILTER_TIME 50 //滤波的“ 稳定时间” 50ms
#define LED_TIME_1S  1000  //时间是 10000ms
/*-----------------自定义时间区域---------------------*/
#define RIGHT_LED_TIME 15
#define FRONT_LED_TIME 25
#define LEFT_LED_TIME 35	
#define LowTimeS 5
/*----------------------------------------------------*/
volatile unsigned int g_count = FRONT_LED_TIME;
volatile unsigned int y_count = RIGHT_LED_TIME;
volatile unsigned int r_count = LEFT_LED_TIME;
BOOL FlashFlag = FALSE;
BOOL LowTime = FALSE;

static u8 Gu8Step = 0; //软件定时器 1 的 switch 切换步骤

#define	INDEX_MAX 3	//显示位索引
static u8 	Display_Code[1]={0x00};		    //1：com 口状态;
static u8 	LED8[3] = {0x00,0x00,0x00};		//显示缓冲支持2位,十位、个位,LED状态。
static u8	display_index = 0;				//显示位索引	

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //接收串口1缓存数组
u8 RX_CONT = 0; 
u8 code t_display[]={						//共阳极
//	 0    1    2    3    4    5    6    7    8    9
	0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,
//black
	0xff,
};	

void uart1_config();	// 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void TaskDisplayScan(void);
void vTaskfFlashLed(void);
void TaskLedRunScan(void);

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
//	{0, 80, 80, TaskLedRunScan},         		// 跑马灯	
//	{0, 50, 50, vKey_Service}					// 按键服务程序50ms
//	{0, 10, 10, TaskRTC}				        // RTC倒计时
//	{0, 30, 30, TaskDispStatus}
	// 这里添加你的任务
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // 运行LED
//	TAST_LED_RUN,        	//跑马灯
//	TASK_KEY_SERV,
//	TASK_RTC,
//	TASK_DISP_WS,             // 工作状态显示// 这里添加你的任务
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
	P2n_standard(0xff);//设置为准双向口
	P0n_push_pull(0xc8);//设置为强推挽
	P4n_standard(0xff);	//设置为准双向口
	P5n_standard(0xff);	//设置为准双向口	
	timer0_init();
	uart1_config();
	//GPIO_OUT_PIN = 0;//输出低，让P26检测
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
void Kled_Set(LogicType type,u8 Nled)//LED设置
{
	u8 val;
	val = LED[2];
	if(type==ON)
		LED[2] = setbit(val,Nled);//高电平导通
	else if(type==OFF)
		LED[2] = clrbit(val,Nled);	
	else if(ON_ALL == type)	
		LED[2] = 0xff;
	else if(OFF_ALL == type)
		LED[2] = 0x00;
	else if(OFF_ON == type)
		LED[2] = reversebit(val,Nled);//翻转
}
void BitX_Set(LogicType type,u8 Xbit)
{
	u8 val=0;
	if(type==ON)
		Display_Code[0] = setbit(val,Xbit);//高电平导通
	else if(type==OFF)
		Display_Code[0] = clrbit(val,Xbit);	
	else if(ON_ALL == type)	
		Display_Code[0] = 0xff;
	else if(OFF_ALL == type)
		Display_Code[0] = 0x00;
	else if(OFF_ON == type)
		Display_Code[0] = reversebit(val,Xbit);//翻转
	
}
void TaskDisplayScan(void)//10ms 刷新一次
{ 
	if(FlashFlag)
	{
		P_COM0 = 0;
		P_COM1 = 0;
		P_COM3 = 0;
		return;
	}
	for(display_index = 0;display_index < INDEX_MAX;display_index++){
		BitX_Set(ON,display_index);//com 口扫描，点亮该位
#if 0
	_nop_();
	_nop_();
	BitX_Set(OFF,display_index);//改变亮度
#endif	
		P_COM0=P_COM1=P_COM2=0;//显示段码前先关闭位码消影;
		_nop_();
		//段码显示，交通灯显示
		LED8_A = getbit(LED8[display_index],0);
		LED8_B = getbit(LED8[display_index],1);
		LED8_C = getbit(LED8[display_index],2);
		LED8_D = getbit(LED8[display_index],3);
		LED8_E = getbit(LED8[display_index],4);
		LED8_F = getbit(LED8[display_index],5);
		LED8_G = getbit(LED8[display_index],6);

		//数码管显示
		P_COM0 = getbit(Display_Code[0],0);
		P_COM1 = getbit(Display_Code[0],1);
		P_COM2 = getbit(Display_Code[0],2);
	}
}
void vTaskfFlashLed(void)
{ 
	if(!LowTime)
		return;
	FlashFlag = ~FlashFlag;//0.5s
		//key_led_reverse();
	switch(Gu8Step)
	{
		case 0:	
			print_char(mod(g_count,10));
			print_char(rem(g_count,10));
		break;
		case 1:
			print_char(mod(y_count,10));
			print_char(rem(y_count,10));
		break;			
		case 2:
			print_char(mod(r_count,10));
			print_char(rem(r_count,10));
		break;	
	}
}
void Gpio_ValRead(void)	
{	
	static unsigned char Su8KeyLock1; //1 号按键的自锁

	if(GPIO_CHECK_PIN==1)
	{
		Su8KeyLock1=0; //按键解锁
		vGu8TimeFlag_2=0;
		vGu16TimeCnt_2=0;
	}
	else if(0==Su8KeyLock1)
	{
		vGu8TimeFlag_2=1;//启动定时器2
		if(vGu16TimeCnt_2>=GPIO_FILTER_TIME) //滤波的“ 稳定时间” GPIO_FILTER_TIME， 长度是 50ms。
		{
			vGu8TimeFlag_2=0;//滤波时间到，定时器请0
			Su8KeyLock1=1; //按键的自锁,避免一直触发
		}		
	}
}	
void Turn_Direction(u8 status)
{
	switch(status)
	{
		case 0://左转
			Kled_Set(ON,LEDL_GREEN);
			Kled_Set(OFF,LEDL_RED);
			Kled_Set(ON,LEDG_RED);
			Kled_Set(OFF,LEDG_GREEN);
			Kled_Set(ON,LEDR_RED);
			Kled_Set(OFF,LEDR_GREEN);
			break;
		case 1://向前
			Kled_Set(OFF,LEDL_GREEN);
			Kled_Set(ON,LEDL_RED);
			Kled_Set(ON,LEDG_GREEN);
			Kled_Set(OFF,LEDG_RED);
			Kled_Set(OFF,LEDR_GREEN);
			Kled_Set(ON,LEDR_GREEN);
			break;
		case 2://向右
			Kled_Set(OFF,LEDL_GREEN);
			Kled_Set(ON,LEDL_RED);
			Kled_Set(OFF,LEDG_GREEN);
			Kled_Set(ON,LEDG_RED);
			Kled_Set(ON,LEDR_GREEN);
			Kled_Set(OFF,LEDR_RED);
		break;
	}
}
void Traffic_Led(void)
{
	switch(Gu8Step)
	{
		case 0://左转，35s
			vGu8TimeFlag_1 = 1;
			Turn_Direction(Gu8Step);
			vGu8LeveCheckFlag = 0;//关闭电平检测
			r_count = LEFT_LED_TIME;
			LED8[0] = t_display[mod(g_count,10)];
			LED8[1] = t_display[rem(g_count,10)];
			if(g_count<=LowTimeS)
				LowTime = TRUE;
			else
				LowTime = FALSE;
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				g_count--;
				vGu32TimeCnt_1 = 0;
				if(g_count == 0)
					Gu8Step++;
			}	
			break;
		case 1://向前,25s
			g_count = FRONT_LED_TIME;
			Turn_Direction(Gu8Step);
			LED8[0] = t_display[mod(y_count,10)];
			LED8[1] = t_display[rem(y_count,10)];	
			vGu8LeveCheckFlag = 1;//启动电平检测
			if(y_count<=LowTimeS)
				LowTime = TRUE;
			else
				LowTime = FALSE;
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				y_count--;	
				vGu32TimeCnt_1 = 0;
				if(y_count == 0)
					Gu8Step++;
			}
			break;
		case 2://向右,15s
			y_count = RIGHT_LED_TIME;
			Turn_Direction(Gu8Step);
			LED8[0] = t_display[mod(r_count,10)];
			LED8[1] = t_display[rem(r_count,10)];	
			if(r_count<=LowTimeS)
				LowTime = TRUE;
			else
				LowTime = FALSE;
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				r_count--;		
				vGu32TimeCnt_1 = 0;
				if(r_count == 0)
				{
					Gu8Step=0;		
					vGu32TimeCnt_1 = 0;
					vGu8TimeFlag_1 = 0;
				}
			}
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
	puts_to_SerialPort("I am Traffic Lights!\n");
	while(1)
	{
		if(B_1ms)	//1ms到
		{
			B_1ms = 0;	
			Traffic_Led();//指示灯状态切换		
			if(vGu8LeveCheckFlag)
				Gpio_ValRead();//低电平检测
			else
			{
				vGu8TimeFlag_2=0;//不检测，定时器请0
			}
		}
		TaskDisplayScan();
		TaskProcess();//显示和闪烁led
	}
}

//========================================================================
// 描述: Timer0 1ms中断函数。
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	B_1ms = 1;		//1ms标志
	TaskRemarks();
	if(vGu8TimeFlag_1){
		vGu32TimeCnt_1++;
	}
	else
		vGu32TimeCnt_1 = 0;
	if(vGu8TimeFlag_2)
		vGu16TimeCnt_2++;
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