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

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 中断频率, 1000次/秒
#define	Baudrate1	115200UL                                //通信波特率115200

#define KEY_LED_GPIO P55

	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd
	#define RED_LED     2
	#define YELLOW_LED  1
	#define GREEN_LED   0
	#define CAR_LED     3
	
#define GPIO_CHECK_PIN P26
#define GPIO_OUT_PIN   P10

//========================================================================
/*************	本地变量声明	**************/
/********************** 公用量 ************************/
BOOL B_1ms;	 //1ms标志
u16	msecond; //1s计数
u8 cnt50ms;  //50ms计数
u8 cnt10ms;  //10ms计数
BOOL B_TX1_Busy;  //发送忙标志

//“软件定时器 1” 的相关变量
volatile unsigned char vGu8TimeFlag_1=0;
volatile u32 vGu32TimeCnt_1=0;	
volatile unsigned char vGu8TimeFlag_2=0;
volatile unsigned int vGu16TimeCnt_2=0;	
volatile unsigned char vGu8LeveCheckFlag=0;

#define G_LED_TIME 59
#define Y_LED_TIME 4
#define R_LED_TIME 29	
volatile unsigned int g_count = G_LED_TIME;
volatile unsigned int y_count = Y_LED_TIME;
volatile unsigned int r_count = R_LED_TIME;
BOOL flash_flag = TRUE;

#define GPIO_FILTER_TIME 50 //滤波的“ 稳定时间” 50ms

#define LED_TIME_1S  1000  //时间是 10000ms
#define LED_TIME_60S 60000 //时间是 60000ms
#define LED_TIME_65S 65000 //时间是 65000ms
#define LED_TIME_95S 95000 //时间是 95000ms

static u8 Gu8Step = 0; //软件定时器 1 的 switch 切换步骤
#define	INDEX_MAX 2	//显示位索引
static u8 	Display_Code[3]={0x00,0x00,0x00};		//1个595控制按键板LED灯,两个595数码管倒计时。
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
sbit	A_HC595_SER   = P3^4;	//pin 34	SER		data input
sbit	A_HC595_RCLK  = P3^7;	//pin 37	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P3^5;	//pin 35	SRCLK	Shift data clock
sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		低电平 使能enable pin
sbit	A_HC595_MR    = P3^6;	//pin 36	低电平复位	

void uart1_config();	// 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
int Get_KeyVal(int val);
void TaskDisplayScan(void);
void vTaskfFlashLed(void);

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
//	{0, 10, 10, TaskDisplayScan},         		// 显示时钟,LED 10ms刷新	
//	{0, 50, 50, vKey_Service}					// 按键服务程序50ms
//	{0, 10, 10, TaskRTC}				        // RTC倒计时
//	{0, 30, 30, TaskDispStatus}
	// 这里添加你的任务
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // 运行LED
//	TAST_DISP_TIME,        // 显示时钟
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
	GPIO_OUT_PIN = 0;//输出低，让P26检测
	A_HC595_MR = 1;//复位禁止
	A_HC595_OE = 0;//使能芯片
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
/*****************************************************
	行列键扫描程序
	使用XY查找8x8键的方法，只能单键，速度快

    Y  P20   P21   P22   P23   P24   P25   P26   P27
        |     |     |     |     |     |     |     |
X       |     |     |     |     |     |     |     |
P10 -- K00 - K01 - K02 - K03 - K04 - K05 - K06 - K07
        |     |     |     |     |     |     |     |
P11 -- K08 - K09 - K10 - K11 - K12 - K13 - K14 - K15
        |     |     |     |     |     |     |     |
P12 -- K16 - K17 - K18 - K19 - K20 - K21 - K22 - K23
        |     |     |     |     |     |     |     |
P13 -- K24 - K25 - K26 - K27 - K28 - K29 - K30 - K31
        |     |     |     |     |     |     |     |
P14 -- K32 - K33 - K34 - K35 - K36 - K37 - K38 - K39
        |     |     |     |     |     |     |     |
P15 -- K40 - K41 - K42 - K43 - K44 - K45 - K46 - K47
        |     |     |     |     |     |     |     |
P16 -- K48 - K49 - K50 - K51 - K52 - K53 - K54 - K55
        |     |     |     |     |     |     |     |
P17 -- K56 - K57 - K58 - K59 - K60 - K61 - K62 - K63
******************************************************/
void gpio_key_delay(void)
{
	u8 i;
	i = 60;
	while(--i)	;
}
void BitX_Set(int status,u8 Xbit)
{
/*
	u8 val;
	Display_Code[2] = 0x00;//将低四位先清0，再置位，每次只能亮一位
	val = Display_Code[2];
	if(En)
		Display_Code[2] = setbit(val,Xbit);//共阳极高电平导通三极管	
*/
	u8 val;
	//val = Display_Code[2];
	val =0x00;
	if(status==ON)
		Display_Code[2] = setbit(val,Xbit);//高电平导通
	else if(status==OFF)
		Display_Code[2] = clrbit(val,Xbit);	
	else if(ON_ALL == status)	
		Display_Code[2] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[2] = 0x00;
	else if(OFF_ON == status)
		Display_Code[2] = reversebit(val,Xbit);//翻转
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
		Display_Code[1] = LED8[display_index];
		//Display_Code[1] = LED8[1];
										//送显即可
		//delay_ms(254);
		//delay_ms(254);
		vDataIn595(Display_Code[2]);//输出位码//考虑消影
		vDataIn595(Display_Code[1]);//输出断码
		vDataIn595(Display_Code[0]);//输出红绿灯状态
		vDataOut595();				//锁存输出数据
	}
}
void vTaskfFlashLed(void)
{ 
	if(flash_flag)
		key_led_reverse();
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
void Kled_Set(int status,u8 Nled)
{
	u8 val;
	val = Display_Code[0];
	if(status==ON)
		Display_Code[0] = setbit(val,Nled);//高电平导通
	else if(status==OFF)
		Display_Code[0] = clrbit(val,Nled);	
	else if(ON_ALL == status)	
		Display_Code[0] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[0] = 0x00;
	else if(OFF_ON == status)
		Display_Code[0] = reversebit(val,Nled);//翻转
}
void Gpio_ValRead(void)	
{	
	static unsigned char Su8KeyLock1; //1 号按键的自锁

	if(GPIO_CHECK_PIN==1)
	{
		Su8KeyLock1=0; //按键解锁
		vGu8TimeFlag_2=0;
		vGu16TimeCnt_2=0;
		flash_flag = TRUE;
		Kled_Set(OFF,CAR_LED);//汽车停止
	}
	else if(0==Su8KeyLock1)
	{
		vGu8TimeFlag_2=1;//启动定时器2
		if(vGu16TimeCnt_2>=GPIO_FILTER_TIME) //滤波的“ 稳定时间” GPIO_FILTER_TIME， 长度是 50ms。
		{
			vGu8TimeFlag_2=0;//滤波时间到，定时器请0
			Su8KeyLock1=1; //按键的自锁,避免一直触发
			Kled_Set(ON,CAR_LED);//汽车停止
			key_led_on(TRUE);//按键提示led
			flash_flag = FALSE;
		}		
	}
}	
void Traffic_Led(void)
{
	switch(Gu8Step)
	{
		case 0:
			vGu8TimeFlag_1 = 1;
			Kled_Set(ON,GREEN_LED);//绿灯亮
			Kled_Set(OFF,RED_LED);//红灯灭
			vGu8LeveCheckFlag = 0;//关闭电平检测
			r_count = R_LED_TIME;
			LED8[0] = ~t_display[mod(g_count,10)];
			LED8[1] = ~t_display[rem(g_count,10)];
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				g_count--;
				vGu32TimeCnt_1 = 0;
				if(g_count == 0)
					Gu8Step++;
			}	
			break;
		case 1:
			g_count = G_LED_TIME;
			Kled_Set(OFF,GREEN_LED);//绿灯灭
			Kled_Set(ON,YELLOW_LED);//黄灯亮
			LED8[0] = ~t_display[mod(y_count,10)];
			LED8[1] = ~t_display[rem(y_count,10)];	
			vGu8LeveCheckFlag = 1;//启动电平检测
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				y_count--;	
				vGu32TimeCnt_1 = 0;
				if(y_count == 0)
					Gu8Step++;
			}
			break;
		case 2:
			y_count = Y_LED_TIME;
			Kled_Set(OFF,GREEN_LED);//绿灯灭
			Kled_Set(OFF,YELLOW_LED);//黄灯灭
			Kled_Set(ON,RED_LED);//红灯亮
			LED8[0] = ~t_display[mod(r_count,10)];
			LED8[1] = ~t_display[rem(r_count,10)];	
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
	vDataIn595(0x00);
	vDataOut595();	//开机默认关闭通道显示LED
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
				Kled_Set(OFF,CAR_LED);//汽车运行
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