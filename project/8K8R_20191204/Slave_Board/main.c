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
#define KEY_BOARD_GPIO_Y P2
	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd
	#define KEY1_VAL 0x10
	#define KEY2_VAL 0x11
	#define KEY3_VAL 0x12
	#define KEY4_VAL 0x13
	#define KEY5_VAL 0x14
	#define KEY6_VAL 0x15
	#define KEY7_VAL 0x16
	#define KEY8_VAL 0x17

#define CMD_ON_OFF_ALL 1
u8 twice_cmd = 0;
	
//========================================================================
/*************	本地变量声明	**************/
/********************** 公用量 ************************/
BOOL B_1ms;	 //1ms标志
u16	msecond; //1s计数
u8 cnt50ms;  //50ms计数
u8 cnt10ms;  //10ms计数
BOOL B_TX1_Busy;  //发送忙标志

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //接收串口1缓存数组
u8 RX_CONT = 0; 

#define	UART1_TX_LENGTH 11
static u8 Display_Code[1]={0x00};//1个595控制按键板LED灯。
static u8 To_Marster_Data[UART1_TX_LENGTH]={0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,'*'};//主控板继电器开关状态，帧头0x01,帧尾0xff;
static u8 code LED_CMD[3]={0x1,0xff,'*'};
/********************** 8*8矩阵键盘 ************************/

static u8 KeyCode= 0x00;	//给用户使用的键码	
static u8 Key8_status = 0;

//“软件定时器 1” 的相关变量
volatile unsigned char vGu8TimeFlag_1=0;
volatile unsigned int vGu16TimeCnt_1=0;
BOOL flash_flag = TRUE;


#define KEY_FILTER_TIME 50 //按键滤波的“ 稳定时间” 50ms

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

/*****************************************************
	行列键扫描程序
	使用XY查找8x8键的方法，只能单键，速度快

    Y  P10   P11   P12   P13   P14   P15   P16   P17
        |     |     |     |     |     |     |     |
X       |     |     |     |     |     |     |     |
P00 -- K00 - K01 - K02 - K03 - K04 - K05 - K06 - K07
        |     |     |     |     |     |     |     |
P01 -- K08 - K09 - K10 - K11 - K12 - K13 - K14 - K15
        |     |     |     |     |     |     |     |
P02 -- K16 - K17 - K18 - K19 - K20 - K21 - K22 - K23
        |     |     |     |     |     |     |     |
P03 -- K24 - K25 - K26 - K27 - K28 - K29 - K30 - K31
        |     |     |     |     |     |     |     |
P04 -- K32 - K33 - K34 - K35 - K36 - K37 - K38 - K39
        |     |     |     |     |     |     |     |
P05 -- K40 - K41 - K42 - K43 - K44 - K45 - K46 - K47
        |     |     |     |     |     |     |     |
P06 -- K48 - K49 - K50 - K51 - K52 - K53 - K54 - K55
        |     |     |     |     |     |     |     |
P07 -- K56 - K57 - K58 - K59 - K60 - K61 - K62 - K63
******************************************************/
void gpio_key_delay(void)
{
	u8 i;
	i = 60;
	while(--i)	;
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

void Gpio_Keyscan(void)	//50ms call
{	
	static unsigned char Su8KeyLock1; //1 号按键的自锁
	static unsigned int Su16KeyCnt1; //1 号按键的计时器
	P10 = 0;
	if(KEY_BOARD_GPIO_Y==0xff)
	{
		Su8KeyLock1=0; //按键解锁
		Su16KeyCnt1=0; //按键去抖动延时计数器清零
		flash_flag = TRUE;
		if(!flash_flag)
			key_led_on(FALSE);//熄灭按键提示led
	}
	else if(0==Su8KeyLock1)
	{
		Su16KeyCnt1++; //累加定时中断次数
		if(Su16KeyCnt1>=KEY_FILTER_TIME) //滤波的“ 稳定时间” KEY_FILTER_TIME， 长度是 50ms。
		{
			Su8KeyLock1=1; //按键的自锁,避免一直触发
			key_led_on(TRUE);//按键提示led
			flash_flag = FALSE;
			KeyCode = Get_KeyVal(KEY_BOARD_GPIO_Y);
		}		
	}
}	
int Get_KeyVal(int val)
{
	int temp;
	switch(val){
		case 0xfe:
				temp =0x10;
			break;
		case 0xfd:
				temp =0x11;
				break;
		case 0xfb:
				temp =0x12;
				break;
		case 0xf7:
				temp =0x13;
				break;
		case 0xef:
				temp =0x14;
				break;
		case 0xdf:
				temp =0x15;
				break;
		case 0xbf:
				temp =0x16;
				break;
		case 0x7f:
				temp =0x17;
				break;
		default:
			break;
	}
	return temp;
}
int Get_Led8Set(void)
{
	static u8 temp = 0;
	temp = Display_Code[0];
	if((temp == 0x80) || (temp == 0x00))
		return 1;
	else if((temp == 0x7f) || (temp ==0xff))
		return 2;
	else 
		return 0;
	
}
void Date_EventProcess(void)
{
	u8 i=0;

	for(i=0;i<8;i++)
	{
		if(getbit(Display_Code[0],i))
			To_Marster_Data[i+1] = 0xfe;
		else
			To_Marster_Data[i+1] = 0x00;
	}
#if defined CMD_ON_OFF_ALL
	if(KEY8_VAL == KeyCode)
		if(twice_cmd){
			print_string(LED_CMD);
			print_string(LED_CMD);
			twice_cmd = 0;
		}
		else
			print_string(LED_CMD);
	else
		print_string(To_Marster_Data);
#else
	print_string(To_Marster_Data);
#endif
		
}
void Key_EventProcess(int KeyCode)
{
	u8 LedBit = 0;
	static u8 LED_ST = 0;
	static u8 Odd_Flag = 1;//奇数标志
	switch(KeyCode){
	case KEY1_VAL:
	case KEY2_VAL:
	case KEY3_VAL:
	case KEY4_VAL:
	case KEY5_VAL:
	case KEY6_VAL:
	case KEY7_VAL:
		LedBit = KeyCode - 0x10;
		Kled_Set(OFF_ON,LedBit);
		if(getbit(Display_Code[0],LedBit))
			LED_ST++;
		else
			LED_ST--;
		break;
	case KEY8_VAL:
		if(LED_ST==7)//不管奇偶，全灭
		{
			Kled_Set(OFF_ALL,7);//全灭
			LED_ST = 0;//计数清0
			if(Odd_Flag==1)
				twice_cmd =1;
			Odd_Flag = 1;//奇数
			
		}
		else if(Odd_Flag)//奇数
		{
			Kled_Set(ON,7);//全亮
			LED_ST = 7;
			Odd_Flag = 0;//设置为偶数
		}
		else if(Odd_Flag==0)//是偶数
		{
			Kled_Set(OFF_ALL,7);//全灭	
			LED_ST = 0;	
			Odd_Flag = 1;//奇数		
		}
		break;
	default:
		break;	
	}
	if(LED_ST>0)
		Kled_Set(ON,7);//8灯亮
	else
		Kled_Set(OFF,7);//8灯灭
	
	vDataIn595(Display_Code[0]);
	vDataOut595();
	if(KeyCode){
		Date_EventProcess();//按键值转换成发给主机格式的值并串口发送;
		KeyCode = 0;//清除按键触发值
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
	puts_to_SerialPort("408as init uart");
	while (1)
	{
		if(B_1ms)	//1ms到
		{
			B_1ms = 0;	
			if(++msecond >= 1000)	//1秒到
			{	
				msecond = 0;
				if(flash_flag)
					key_led_reverse();
			}
			if(++cnt50ms >= 50)		//50ms到
			{
				cnt50ms = 0;
			}
			Key_EventProcess(KeyCode);
			if(KeyCode > 0)		//有键按下
			{		
				KeyCode = 0;
			}			
		}
	}
}

//========================================================================
// 描述: Timer0 1ms中断函数。
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	B_1ms = 1;		//1ms标志
	if(vGu8TimeFlag_1)
		vGu16TimeCnt_1++;
	else
		vGu16TimeCnt_1 = 0;
	Gpio_Keyscan();//按键扫描
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