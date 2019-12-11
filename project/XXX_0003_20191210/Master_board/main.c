/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了版权 ---------------   */
/*---------------------------------------------------------------------*/
/*************	本程序功能说明	**************
8K8R:4个key控制12路，16种功能，具体功能见spec

用STC的MCU的IO方式矩阵键盘交互和串口收发数据。
1、开始按下或者运行灯一直亮，停止键按下亮一下灭掉，所有指示灯灭掉。
2、模式+，模式- 按下LED亮，松手灭，模式对应+，-。
3、PT2272 按键1开始，2+，3-，4暂停
4、

存在缺陷：
1、
2、

v1.1：
1、波特率115200                     20191210
2、
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
	
	#define RUN_STOP 0xff
	#define RUN_START 0x01
	
	#define KEY_START 0x01
	#define KEY_UP    0x02
	#define KEY_DOWN  0x03
	#define KEY_STOP  0x04
	
#define KEY_FILTER_TIME 20 //滤波的“ 稳定时间” 20ms

#define KEY_INPUT1 P23  //【启动】按键K1的输入口。
#define KEY_INPUT2 P20  //【停止】按键K2的输入口。
#define KEY_INPUT3 P22  //【模式+】按键K3的输入口。
#define KEY_INPUT4 P21  //【模式-】按键K4的输入口。
#define GPIO_OUT_PIN   P10
#define GPIO_VT_CHECK  P13
/*
#define PT2272_D0      P14
#define PT2272_D1      P15
#define PT2272_D2      P16
#define PT2272_D3      P17
*/
#define PT2272_DATA(x)  x>>4
BOOL update_flag = TRUE;
BOOL Key_EventProtect = FALSE;

#define LED_TIME_60S 60000 //时间是 60000ms
#define LED_TIME_65S 65000 //时间是 65000ms
#define LED_TIME_95S 95000 //时间是 95000ms

//========================================================================
/*************	本地变量声明	**************/
BOOL B_1ms;	 		//1ms标志
BOOL B_TX1_Busy;  	//发送忙标志

volatile unsigned char vGu8KeySec=0;  //按键的触发序号
static u8 Gu8Step = 1;				  //switch 切换步骤
int Key_Code=0xff;
static u8 Key_CodeOld = 0x00 ;
#define	INDEX_MAX 2	//显示位索引
static u8 	Display_Code[3]={0x00,0x00,0x00};		//按键灯，L_MOS，H_MOS。
//static u8 	LED8[4] = {0x00,0x00,0x00,0x00};		//显示缓冲支持四位
//static u8	display_index = 0;						//显示位索引	

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //接收串口1缓存数组
u8 RX_CONT = 0; 
u8 code t_display[]={						//共阴极标准字库，共阳取反
//	 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,
//black	 -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e,
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	//0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1
u8 code pt2272[] = {
//   1     2    3    4    5    6    7    8   9    10   11   12   13   14   15
	0x01,0x02,0x04,0x0a,0x0b,0x06,0x07,0x0c,0x09,0x03,0x0e,0x0d,0x0f,0x08,0x05	
};

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
	P2n_push_pull(0xff);	//设置为准双向口
	P3n_standard(0xff);	//设置为准双向口
	P4n_standard(0xff);	//设置为准双向口
	P5n_standard(0xff);	//设置为准双向口	
	timer0_init();
	uart1_config();
	GPIO_OUT_PIN = 0;//输出低，让按键检测
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
	u8 val;
	val = Display_Code[0];
	if(status==ON)
		setbit(val,Kled);//高电平导通
	else if(status==OFF)
		clrbit(val,Kled);	
	else if(ON_ALL == status)	
		val = 0xff;
	else if(OFF_ALL == status)
		val= 0x00;
	else if(OFF_ON == status)
		reversebit(val,Kled);//翻转
	Display_Code[0] = val;
}
void Nled_Set(int status,u8 Nled)
{
	u16 val;
	val = Bit8_ToBit16(Display_Code[2],Display_Code[1]);//H,L
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
	Display_Code[2] = Bit16_ToBit8(val,1);//高8位
	Display_Code[1] = Bit16_ToBit8(val,0);//低8位
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
void Transsion_Keycode(int key_code)
{
	if(key_code==Key_CodeOld)
		return;
	switch(key_code)
	{
		case KEY_START:
		vGu8KeySec=1;    //触发开启键
		break;
		case KEY_UP:
		vGu8KeySec=3;    //触发+号键
		Kled_Set(ON,2);//按键+灯亮	
		break;
		case KEY_DOWN:
		vGu8KeySec=4;    //触发-号键
		Kled_Set(ON,1);//按键-灯亮	
		break;
		case KEY_STOP:
		vGu8KeySec=2;    //触发4号键
		Kled_Set(ON,0);//暂停键灯亮
		break;
		default : 
		vGu8KeySec=0;    //触发4号键
		Key_EventProtect = FALSE;
		break;
		key_code = 0;
		Key_CodeOld = key_code;
	}
	
}
void KeyScan(void)
{
	static unsigned char Su8KeyLock1;
	static unsigned int  Su16KeyCnt1;
	static unsigned char Su8KeyLock2;
	static unsigned int  Su16KeyCnt2; 	
	static unsigned char Su8KeyLock3;
	static unsigned int  Su16KeyCnt3; 
	static unsigned char Su8KeyLock4;
	static unsigned int  Su16KeyCnt4; 
	static unsigned char Su8KeyLock5;
	static unsigned int  Su16KeyCnt5;
	
	if(Key_EventProtect)
		return;
    //【启动】按键K1的扫描识别
	if(0!=KEY_INPUT1)
	{
		Su8KeyLock1=0;
		Su16KeyCnt1=0;   
	}
	else if(0==Su8KeyLock1)
	{
		Su16KeyCnt1++;
		if(Su16KeyCnt1>=KEY_FILTER_TIME)
		{
			Su8KeyLock1=1;  
			vGu8KeySec=1;    //触发1号键
			Key_EventProtect = TRUE;
		}
	}
   //【停止】按键K2的扫描识别
	if(0!=KEY_INPUT2)
	{
		Su8KeyLock2=0;
		Su16KeyCnt2=0; 
		if(!Su8KeyLock5)
			Kled_Set(OFF,0);//按键灯灭			
	}
	else if(0==Su8KeyLock2)
	{
		Su16KeyCnt2++;
		if(Su16KeyCnt2>=KEY_FILTER_TIME)
		{
			Su8KeyLock2=1;  
			vGu8KeySec=2;    //触发2号键
			Key_EventProtect = TRUE;
			Kled_Set(ON,0);//按键灯亮
		}
	}
   //【模式+】按键K3的扫描识别
	if(0!=KEY_INPUT3)
	{
		Su8KeyLock3=0;
		Su16KeyCnt3=0; 
		if(!Su8KeyLock5)
			Kled_Set(OFF,2);//按键灯灭		
	}
	else if(0==Su8KeyLock3)
	{
		Su16KeyCnt3++;
		if(Su16KeyCnt3>=KEY_FILTER_TIME)
		{
			Su8KeyLock3=1;  
			vGu8KeySec=3;    //触发3号键
			Key_EventProtect = TRUE;
			Kled_Set(ON,2);//按键灯亮
		}
	}
   //【模式-】按键K4的扫描识别
	if(0!=KEY_INPUT4)
	{
		Su8KeyLock4=0;
		Su16KeyCnt4=0; 
		if(!Su8KeyLock5)
			Kled_Set(OFF,1);//按键灯灭		
	}
	else if(0==Su8KeyLock4)
	{
		Su16KeyCnt4++;
		if(Su16KeyCnt4>=KEY_FILTER_TIME)
		{
			Su8KeyLock4=1;  
			vGu8KeySec=4;    //触发4号键
			Key_EventProtect = TRUE;
			Kled_Set(ON,1);//按键灯亮
		}
	}
	
	if(Key_EventProtect)
		return;//如果有按键按下，不执行遥控检测。
	
	if(!(P1&0xf0))//如果不为0则未触发
	{ 
	
		Su8KeyLock5=0;
		Su16KeyCnt5=0;
		if(!Su8KeyLock2)
			Kled_Set(OFF,0);//暂停键灯灭
		else if(!Su8KeyLock3)
			Kled_Set(OFF,2);//按键+灯灭	
		else if(!Su8KeyLock4)
			Kled_Set(OFF,1);//按键-灯灭	
	}
	else if(0==Su8KeyLock5)
	{
		Su16KeyCnt5++;
		if(Su16KeyCnt5>=KEY_FILTER_TIME)
		{
			
			Su8KeyLock5=1;  
			Key_EventProtect = TRUE;
			Key_Code = Get_Pt2272State();
			Transsion_Keycode(Key_Code);	
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
	if((Gu8Step == RUN_STOP)&&(vGu8KeySec != 1))
	{
		vGu8KeySec = 0;
		Key_EventProtect = FALSE;
		return;//如果是停止键，且不是启动键,不执行
	}
	switch(vGu8KeySec) //根据不同的按键触发序号执行对应的代码
	{
		case 1:     //1号按键。【启动】按键K1
			Gu8Step = RUN_START;
			Kled_Set(ON,3);//按键灯亮
			vGu8KeySec=0; 
		break;
		case 2:     //2号按键。【停止】按键K2
			Gu8Step = RUN_STOP;
			Kled_Set(OFF,1);//按键灯灭
			Kled_Set(OFF,2);//按键灯灭
			Kled_Set(OFF,3);//按键灯灭
			vGu8KeySec=0;  
		break;
		case 3:     //3号按键。【模式+】按键K3
			Gu8Step++;
			vGu8KeySec=0;  
		break;
		case 4:     //4号按键。【模式-】按键K4
			Gu8Step--;
			vGu8KeySec=0; 
		break;
		default:     
			 
		break;
	}
	if(Gu8Step >16 && (Gu8Step != RUN_STOP))
		Gu8Step = 1;
	else if(Gu8Step < 1)
		Gu8Step = 16;
	update_flag = TRUE;
	Key_EventProtect = FALSE;
}	
void Led_StateUpdate(void)
{
	if(!update_flag)
		return ;
	Nled_Set(OFF_ALL,1);//发生改变时，先清0一次;
	switch(Gu8Step)
	{
		case 1:
			Kled_Set(ON,3);//按键灯亮
			Nled_Set(ON,0);
			Nled_Set(ON,2);
		break;
		case 2:
			Nled_Set(ON,4);
		break;
		case 3:
			Nled_Set(ON,1);
			Nled_Set(ON,3);
		break;	
		case 4:
			Nled_Set(ON,5);
		break;
		case 5:
			Nled_Set(ON,0);
			Nled_Set(ON,2);
		break;
		case 6:
			Nled_Set(ON,6);
		break;
		case 7:
			Nled_Set(ON,1);
			Nled_Set(ON,3);
		break;
		case 8:
			Nled_Set(ON,5);
		break;
		case 9:
			Nled_Set(ON,0);
			Nled_Set(ON,7);
		break;
		case 10:
			Nled_Set(ON,9);
		break;
		case 11:
			Nled_Set(ON,1);
			Nled_Set(ON,8);
		break;
		case 12:
			Nled_Set(ON,10);
		break;
		case 13:
			Nled_Set(ON,0);
			Nled_Set(ON,7);
		break;
		case 14:
			Nled_Set(ON,11);
		break;
		case 15:
			Nled_Set(ON,0);
			Nled_Set(ON,8);
		break;
		case 16:
			Nled_Set(ON,10);
		break;
		
		default:
		break;
		update_flag = FALSE;//更新后及时置位0;
	}
}

void TaskDisplayScan(void)//10ms 刷新一次
{ 
	vDataIn595(Display_Code[2]);//高8位MOS管的状态
	vDataIn595(Display_Code[1]);//低8位MOS管的状态
	vDataIn595(Display_Code[0]);//输出按键指示灯状态
	vDataOut595();				//锁存输出数据
}
void vTaskfFlashLed(void)
{ 
	key_led_reverse();
	//print_char(Get_Pt2272State());
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
	puts_to_SerialPort("I am LED controller XXX_0003_20191210!\n");
	puts_to_SerialPort("Contact me: 15901856750\n");
	Kled_Set(ON,3);//按键灯亮
	while (1)
	{	
		KeyTask();    //按键的任务函数
		Led_StateUpdate();//LED状态更新
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