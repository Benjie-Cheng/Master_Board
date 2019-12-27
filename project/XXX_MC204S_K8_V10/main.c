/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了版权 ---------------   */
/*---------------------------------------------------------------------*/
/*************	XXX_MC204S_K8_V10	**************
XXX_0008:该控制板主要用于小项目开发，具体功能见spec

1、TM1650 驱动四位共阴极数码管，实现4*7 键盘扫描，预留级联口。
2、595控制按键LED灯显示，预留有级联口。
3、PT2272 遥控功能接口。
4、BT05蓝牙uart接口。
5、两个WS2811 接口。
6、电源12V,5V，GND
7、GPIO扩展预留。

存在缺陷：
1、
2、

v1.0：软件设定
1、波特率115200 ，晶振35M          
2、
******************************************/
#include "config.h"
#include "debug.h"

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 中断频率, 1000次/秒
#define	Baudrate1	115200UL                                //通信波特率115200

	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd
	#define OFF_ON_ALL 0xfc
	#define RED_LED     2
	#define YELLOW_LED  1
	#define GREEN_LED   0
	
	#define RUN_STOP 0xff
	#define RUN_START 0x01
	
	#define KEY_START 0x01
	#define KEY_UP    0x02
	#define KEY_DOWN  0x03
	#define KEY_STOP  0x04
	#define KEY_NULL  0xff
	#define S1        1
	#define S2        2
	#define S3        3
	#define S4        4
	#define S5        5
	#define S6        6
	#define S7        7
	#define S8        8
	#define S9        9
	#define S10       10
	#define S11       11
	#define S12       12
	#define S13       13
	#define S14       14
	#define S15       15
	#define S16       16
	
	
#define KEY_FILTER_TIME 20 //滤波的“ 稳定时间” 20ms

/*
PT2272芯片控制管脚
#define GPIO_VT_CHECK  P14
#define PT2272_D0      P10
#define PT2272_D1      P11
#define PT2272_D2      P12
#define PT2272_D3      P13
*/
#define PT2272_DATA(x)  x>>0


#define SET_SCL_OUT_TM1650()    //{SCL_TM1650=1; PC_CR1_C17 = 1; PC_CR2_C27 = 0;}
#define SET_SDA_OUT_TM1650()    //{PC_DDR_DDR6=1; PC_CR1_C16 = 1; PC_CR2_C26 = 0;}
#define SET_SDA_IN_TM1650()     //{PC_DDR_DDR6=0; PC_CR1_C16 = 0; PC_CR2_C26 = 0;}


BOOL LED_OFF_All_Flag = FALSE;

BOOL update_flag = FALSE;
BOOL Key_EventProtect = FALSE;
//-----------------------------------------------------------------------------------------------------------------
//“软件定时器 1” 的相关变量
volatile unsigned char vGu8TimeFlag_1=0;//20分钟计数
volatile u32 vGu32TimeCnt_1=0;
volatile unsigned char vGu8TimeFlag_2=0;//5分钟计数
volatile u32 vGu32TimeCnt_2=0;
volatile unsigned char vGu8TimeFlag_3=0;//5分钟计数
volatile u32 vGu32TimeCnt_3=0;
unsigned int  uiKeyTimeCnt=0; //按键去抖动延时计数器 //按键去抖动延时计数器


//========================================================================
/*************	本地变量声明	**************/
BOOL B_1ms;	 		//1ms标志
BOOL B_TX1_Busy;  	//发送忙标志

static u8 Gu8Step = 0;				  //switch 切换步骤
static u8 ucKeyStep = 0;
volatile unsigned char Key_Code=0xff;//定时器中断中使用
#define DISP_LENGTH 2
static u8 	Display_Code[DISP_LENGTH]={0x00,0x00};		   //按键灯，L_MOS，H_MOS。
#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //接收串口1缓存数组
u8 RX_CONT = 0; 
u8 code TM1650_CODE[]={						//TM1650显示代码,共阴极标准字库，共阳取反
//	 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,
//black	 -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e,
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	//0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1
u8 code pt2272[] = {//PT2272解码数据
//   1     2    3    4    5    6    7    8   9    10   11   12   13   14   15
	0x01,0x02,0x04,0x0a,0x0b,0x06,0x07,0x0c,0x09,0x03,0x0e,0x0d,0x0f,0x08,0x05	
};
u8 code KeyVal[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};

/********************** A 给指示灯用的595 ************************/
sbit	A_HC595_SER   = P5^5;	//pin 55	SER		data input
sbit	A_HC595_RCLK  = P1^5;	//pin 15	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P5^4;	//pin 54	SRCLK	Shift data clock
//sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		低电平 使能enable pin
sbit	A_HC595_MR    = P3^2;	//pin 32	低电平复位	

void uart1_config();	// 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void TaskDisplayScan(void);
void vTaskfFlashLed(void);
void Date_EventProcess(void);

typedef struct _TASK_COMPONENTS
{
	u8 Run;                  //程序运行标记：0-不运行，1运行
	u16 Timer;               // 计时器
	u16 ItvTime;             // 任务运行间隔时间
	void (*TaskHook)(void); // 要运行的任务函数
} TASK_COMPONENTS;   
static TASK_COMPONENTS TaskComps[] =
{
	{0, 1000,  1000, vTaskfFlashLed},           // led闪烁1s
	{0, 10, 10, TaskDisplayScan},         		// 595控制刷新10ms一次
//	{0, 50, 50, vKey_Service}					// 按键服务程序50ms
//	{0, 10, 10, TaskRTC}				        // RTC倒计时
//	{0, 30, 30, TaskDispStatus}
	// 这里添加你的任务
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // 运行LED
	TAST_595_UPDATE,        // 显示时钟
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
	P2n_standard(0xff);	//设置为准双向口
	P3n_standard(0xff);	//设置为准双向口
	P4n_standard(0xff);	//设置为准双向口
	P5n_standard(0xff);	//设置为准双向口
	A_HC595_MR = 0;     //先清零数据输出	
	timer0_init();
	uart1_config();
	A_HC595_MR = 1;     //复位禁止
	//A_HC595_OE = 0;   //使能芯片,长低有效
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
	for (i=0; i<TASKS_MAX; i++)            // 逐个任务时间处理
	{
		if (TaskComps[i].Run)              // 时间不为0
		{
			TaskComps[i].TaskHook();       // 运行任务
			TaskComps[i].Run = 0;          // 标志清0
		}
	}
}
u16 Bit8_ToBit16(u8 high,u8 low)
{
	u16 val;
	val = (high << 8) | low;;
	return val;
}
u8 Bit16_ToBit8(u16 num,u8 high)
{
	u16 val;
	if(high)
		val = (num >> 8) & 0xff;//高8位
	else
		val = num & 0xff; //低8位
	return val;
}
void Kled_Set(int status,u8 Kled)
{
	u16 val;
	val = Bit8_ToBit16(Display_Code[1],Display_Code[0]);//H,L
	if(status==ON)
		setbit(val,Kled);//高电平导通，如果低电平导通，取反最后发送数据
	else if(status==OFF)
		clrbit(val,Kled);	
	else if(ON_ALL == status)	
		val = 0xffff;
	else if(OFF_ALL == status)
		val= 0x0000;
	else if(OFF_ON_ALL == status)
		val= ~val;
	else if(OFF_ON == status)
		reversebit(val,Kled);//翻转
	Display_Code[1] = Bit16_ToBit8(val,1);//高8位
	Display_Code[0] = Bit16_ToBit8(val,0);//低8位
}
void Nled_Set(int status,u8 Nled)
{
	u16 val;
	val = Bit8_ToBit16(Display_Code[1],Display_Code[0]);//H,L
	if(status==ON)
		setbit(val,Nled);//高电平导通
	else if(status==OFF)
		clrbit(val,Nled);	
	else if(ON_ALL == status)	
		val = 0xffff;
	else if(OFF_ALL == status)
		val= 0x0000;
	else if(OFF_ON == status)
		reversebit(val,Nled);//翻转
	Display_Code[1] = Bit16_ToBit8(val,1);//高8位
	Display_Code[0] = Bit16_ToBit8(val,0);//低8位
}
int Get_Pt2272State(void)
{
	u8 i = 0,val;
	for(i=0;i<15;i++)
	{
		if(PT2272_DATA(P1)==pt2272[i]) 
		{
			val = i+1;
			break;
		}
		else 
			val = 0;
	}
	return val;
}
void KeyTask(void)
{
	u8 led_bit=0;
	if(Key_Code==0)
	return;
	//print_char(Key_Code);
	led_bit = Key_Code;
	led_bit--;
	Gu8Step = 0;
	//print_char(led_bit);
	switch(Key_Code)
	{
		case S1:
		case S2:
		case S3:
		case S4:
		case S5:
		case S6:
		case S7:
		case S8:
			Kled_Set(OFF_ON,led_bit);
			break;
		case S9:
		case S10:
		case S11:
		case S12:
		case S13:
		case S14:
		case S15:
			Kled_Set(ON,led_bit);//该位开，开启一分钟计时
			break;
		default : 
		
			break;
	}	
	Key_Code = 0;//按键值清0，防止反复触发
	Key_EventProtect = FALSE;//解除按键保护，可以接受按键值。
}
void Date_EventProcess(void)
{

}
void State_EventProcess(void)
{
	switch(Gu8Step)
	{
		case 0:
			break;
		case 1:
			break;			
		case 2:
			break;			
		default : 
		
			break;			
	}
}
void KeyScan(void)
{
	static unsigned char Su8KeyLock5;
	static unsigned int  Su16KeyCnt5;

	if(Key_EventProtect)
		return;//如果有按键按下，不执行遥控检测。
	
	if(!(P1&0x0f))//如果不为0则未触发
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
			//key_led_on(1);			
			Key_EventProtect = TRUE;
			Key_Code = Get_Pt2272State();	
		}
	}
	
}
//------------------------------------------------------------
void TaskDisplayScan(void)//595控制刷新10ms一次
{ 
	if(LED_OFF_All_Flag)//如果全关
	{
		Display_Code[0] = 0x00;
		vDataIn595(~Display_Code[0]);//输出按键指示灯关
		vDataOut595();
		return;
	}
	vDataIn595(~Display_Code[0]);//输出按键指示灯低8位状态，低电平点亮
	vDataOut595();				//锁存输出数据
}
void vTaskfFlashLed(void)//1s 一次
{ 
	Kled_Set(OFF_ON,7);//最高位LED秒闪
	//print_char(GPIO_OUT_ROW);
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
	vDataOut595();	//开机默认关闭通道显示LED
	puts_to_SerialPort("I am LED controller XXX_MC204S_K8_V10!\n");
	puts_to_SerialPort("Contact me: 15901856750\n");
	key_led_on(0);
	Init_Tm1650()//数码管开显示
	while (1)
	{	
		//Led_StateUpdate();//LED状态更新
		KeyTask();
		Date_EventProcess();
		State_EventProcess();
		TaskProcess();//595更新和闪烁led任务执行		 
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