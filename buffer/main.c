/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了版权 ---------------   */
/*---------------------------------------------------------------------*/
/*************	本程序功能说明	**************

用STC的MCU的IO方式矩阵键盘和串口收发数据。
1、8*8 矩阵键盘检测对应继电器的按键输入。，按一次状态翻转一次。
2、串口将扫描到的键值发送到主机，主机反馈当前继电器通断状态给从机。
3、从机根据主机接收到的数据，通过74HC595控制LED,指示通道开关状态。
4、有按键按下时有一颗公用的LED指示按键触发

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
#define	Baudrate1	9600UL                                //通信波特率115200

#define GPIO_KEY64_BOARD
#define KEY_LED_GPIO P55
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
/********************** 8*8矩阵键盘 ************************/
#if defined GPIO_KEY64_BOARD
#define KEY_BOARD_GPIO_X P1
#define KEY_BOARD_GPIO_Y P2
u8 code Line_KeyTable[]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};	//线行码
u8 IO_KeyState =0,IO_KeyState_x =0,IO_KeyState_y =0,IO_KeyState_old = 0xff;	//行列键盘变量
u8	KeyCode= 0xff;	//给用户使用的键码, 0~63有效
#endif


//“软件定时器 1” 的相关变量
volatile unsigned char vGu8TimeFlag_1=0;
volatile unsigned int vGu16TimeCnt_1=0;
#define LED_TIME_3S  3000  //时间是 3000ms
#define LED_TIME_7S  7000  //时间是 3000ms
#define LED_TIME_10S 10000 //时间是 3000ms
#define LED_TIME_15S 15000 //时间是 3000ms
#define LED_TIME_18S 18000 //时间是 3000ms
#define LED_TIME_23S 23000 //时间是 3000ms
#define LED_TIME_26S 26000 //时间是 3000ms
#define LED_TIME_31S 31000 //时间是 3000ms
#define LED_TIME_34S 34000 //时间是 3000ms
#define LED_TIME_39S 39000 //时间是 3000ms
#define LED_TIME_42S 42000 //时间是 3000ms
#define LED_TIME_60S 60000 //时间是 3000ms
#define KEY8_VAL 0x17
#define     E2PROM_LENGTH 8
u16 T0,T2,T3;
u16 U1_T1,U2_T1,U3_T1,U4_T1,U5_T1;			 //T0,  T2,  T3, U1_T1, U2_T1, U3_T1,  U4_T1,  U5_T1
static u8 	E2PROM_Strings[E2PROM_LENGTH] = {0x00,0x00,0x00,0x00,  0x00,   0x00,   0x00,   0x00};
#define GET_INIT_VAL()	{T0=E2PROM_Strings[0]*1000;T2=E2PROM_Strings[1]*1000;T3=E2PROM_Strings[2]*1000;\
U1_T1=E2PROM_Strings[3]*1000;U2_T1=E2PROM_Strings[4]*1000;U3_T1=E2PROM_Strings[5]*1000;U4_T1=E2PROM_Strings[6]*1000;U5_T1=E2PROM_Strings[7]*1000;}

static u8 Gu8Step = 0; //软件定时器 1 的 switch 切换步骤
static u8 vGu8Key1_lock = 0;
static u8 vGu8Key2_lock = 0;
static u8 vGu8Key3_lock = 0;
static u8 vGu8Key4_lock = 0;
static u8 vGu8Key5_lock = 0;
static u8 vGu8Key6_lock = 0;
static u8 vGu8Key7_lock = 0;
static u8 vGu8Key8_lock = 0;

#define KEY_FILTER_TIME 50 //按键滤波的“ 稳定时间” 25ms

/********************** A 给指示灯用的595 ************************/
sbit	A_HC595_SER   = P3^4;	//pin 34	SER		data input
sbit	A_HC595_RCLK  = P3^7;	//pin 37	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P3^5;	//pin 35	SRCLK	Shift data clock
sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		低电平 使能enable pin
sbit	A_HC595_MR    = P3^6;	//pin 36	低电平复位	


static u8 Display_Code[2]={0x00,0x00};//两个595数据。

void uart1_config();	// 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void Key1_Fun(void);
void Key2_Fun(void);
void Key3_Fun(void);
void Key4_Fun(void);
void Key5_Fun(void);
void Key6_Fun(void);
void Key7_Fun(void);
void Key8_Fun(void);
int Check_KeyStatus(void);
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
void Nmos_Set(BOOL en,u8 Nmos)
{
	u8 val;
	val = Display_Code[1];
	if(en)
		Display_Code[1] = setbit(val,Nmos);//高电平导通三极管，Mos管工作
	else
		Display_Code[1] = clrbit(val,Nmos);	
	if(0xff == Nmos)	
		Display_Code[1] &= 0x80;
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
#if defined GPIO_KEY64_BOARD
void gpio_key_delay(void)
{
	u8 i;
	i = 60;
	while(--i)	;
}
u16 compose(u8 x,u8 y)
{
	u16 val;
	val = y;
	return (val<<8) | x;
}	
u8 val_to_line(u8 key_val)
{
	u8 val = 0,i;
	
	for(i=0;i<8;i++)
	{
		if(key_val == Line_KeyTable[i])
			val = i;
	}
	return val;
}

#if 0
void Gpio_Keyscan(void)	//50ms call
{
	u16	j,x,y;
	
	x = val_to_line(IO_KeyState_x);			
	y = val_to_line(IO_KeyState_y);
	j = x * 8 + y + 16;
	
	KEY_BOARD_GPIO_Y = 0xff;
	KEY_BOARD_GPIO_X = 0x00;	//X低，读Y
	gpio_key_delay();
	IO_KeyState_y = KEY_BOARD_GPIO_Y & 0xff;
	
	KEY_BOARD_GPIO_X =0xff;
	KEY_BOARD_GPIO_Y = 0x00;	//Y低，读X
	gpio_key_delay();
	IO_KeyState_x= KEY_BOARD_GPIO_X & 0xff;
	
	IO_KeyState_x ^= 0xff;	//取反
	IO_KeyState_y ^= 0xff;	//取反
	

#if 1
	x = val_to_line(IO_KeyState_x);			
	y = val_to_line(IO_KeyState_y);
	//print_char(x);
	//print_char(y);
	IO_KeyState = x * 8 + y + 16;	//计算键码，(0~63) +16
#endif
	if(!IO_KeyState_x && !IO_KeyState_y){
		IO_KeyState_old = 0;
		key_led_on(FALSE);//熄灭按键提示led
	}
	else //if(j == IO_KeyState)	//连续两次读相等
	{
		if(IO_KeyState != 0 &&(IO_KeyState_old != IO_KeyState))	//有键按下
		{
			IO_KeyState_old = IO_KeyState;
			x = val_to_line(IO_KeyState_x);			
			y = val_to_line(IO_KeyState_y);
			KeyCode = x * 8 + y + 16;	//计算键码，(0~63) +16
			print_char(KeyCode);
		}
			key_led_on(TRUE);//熄灭按键提示led
	}
	KEY_BOARD_GPIO_X = 0xff;
	KEY_BOARD_GPIO_Y = 0xff;
}
#else
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
void Gpio_Keyscan(void)	//50ms call
{	
	static unsigned char Su8KeyLock1; //1 号按键的自锁
	static unsigned int Su16KeyCnt1; //1 号按键的计时器

		P10 = 0;
		if(KEY_BOARD_GPIO_Y==0xff)
		{
			Su8KeyLock1=0; //按键解锁
			Su16KeyCnt1=0; //按键去抖动延时计数器清零
			key_led_on(FALSE);//熄灭按键提示led
		}
		else if(0==Su8KeyLock1)
		{
			Su16KeyCnt1++; //累加定时中断次数
			if(Su16KeyCnt1>=KEY_FILTER_TIME) //滤波的“ 稳定时间” KEY_FILTER_TIME， 长度是 25ms。
			{
				Su8KeyLock1=1; //按键的自锁,避免一直触发
				key_led_on(TRUE);//按键提示led
				KeyCode = Get_KeyVal(KEY_BOARD_GPIO_Y);
				if(Check_KeyStatus()&&(KeyCode!=KEY8_VAL))
					KeyCode = 0;
			}		
		}
}	
#endif
#endif


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
	puts_to_SerialPort("init uart");
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,E2PROM_LENGTH);
	GET_INIT_VAL();
	while (1)
	{
		if(B_1ms)	//1ms到
		{
			B_1ms = 0;	
			if(++msecond >= 1000)	//1秒到
			{	
				msecond = 0;
				Display_Code[1]=reversebit(Display_Code[1],7);
			}
		  if(++cnt50ms >= 50)		//50ms扫描一次行列键盘
			{
				cnt50ms = 0;
		#if defined GPIO_KEY64_BOARD
				//Gpio_Keyscan();
				vDataIn595(Display_Code[1]);
				vDataIn595(Display_Code[1]);
				vDataOut595();
		#endif
			}
		#if defined GPIO_KEY64_BOARD
			if((Check_KeyStatus())&&(KeyCode==KEY8_VAL))
			{
				vGu8Key1_lock = 0;
				vGu8Key2_lock = 0;
				vGu8Key3_lock = 0;
				vGu8Key4_lock = 0;
				vGu8Key5_lock = 0;
				vGu8Key6_lock = 0;
				vGu8Key7_lock = 0;	
				vGu8Key8_lock = 0;
				vGu16TimeCnt_1 = 0;
				Gu8Step	= 0;	
			}
			/*
			else if((KeyCode == KEY8_VAL) && vGu8Key8_lock)
			{
				Key8_Fun();
				KeyCode = 0;
				print_char(0xcc);
			}
			*/
			if(KeyCode > 0)		//有键按下
			{	
					//Nmos_Set(0,6);//debug led
					switch(KeyCode)
					{		
						case 0x10:
							vGu8Key1_lock = 1;
							break; 
						case 0x11:
							vGu8Key2_lock = 1;
							break;
						case 0x12:
							vGu8Key3_lock = 1;
							break;		
						case 0x13:
							vGu8Key4_lock = 1;
							break;		
						case 0x14:
							vGu8Key5_lock = 1;
							break;	
						case 0x15:
							vGu8Key6_lock = 1;
							break;	
						case 0x16:
							vGu8Key7_lock = 1;
							break;	
						case 0x17:
							vGu8Key8_lock = 1;
							break;	
						default:
							break;
					}				
					KeyCode = 0;
					Gu8Step = 0;//step 清0
			}			
	
			if(vGu8Key8_lock)
			{
				KeyCode = 0;
				Key8_Fun();
				print_char(0x0a);
			}
			else if(vGu8Key1_lock){
				Key1_Fun();
				print_char(0x0b);
			}
			else if(vGu8Key2_lock){
				Key2_Fun();
				print_char(0x0c);
			}
			else if(vGu8Key3_lock){
				Key3_Fun();
				print_char(0x0d);
			}
			else if(vGu8Key4_lock){
				Key4_Fun();
				print_char(0x0e);
			}
			else if(vGu8Key5_lock){
				Key5_Fun();
				print_char(0x0f);
			}
			else if(vGu8Key6_lock){
				Key6_Fun();
				print_char(0xaa);
			}
			else if(vGu8Key7_lock){
				Key7_Fun();	
				print_char(0xbb);
			}
		#endif	
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
	Gpio_Keyscan();
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
		//TxSend(SBUF);
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
void Key1_Fun(void)
{
	
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			//vGu8Key1_lock = 1;//按键上锁
			vGu8TimeFlag_1 = 1;//定时器运行置位
			Nmos_Set(1,6);//debug led
			Nmos_Set(1,0);
			Nmos_Set(1,1);
			Nmos_Set(1,2);
			Nmos_Set(1,3);
			Nmos_Set(1,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T0) //T0时间到
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);	
			if(vGu16TimeCnt_1>=T3) //T3s时间到
				Gu8Step++;
			break;
		case 2:
			//Nmos_Set(1,6);//debug led
			Gu8Step = 0;//step 清0
			vGu8TimeFlag_1 = 0;//清定时器运行标志
			vGu8Key1_lock = 0;//解按键锁
			break;	
		default:
		
		break;
	}	
}
void Key2_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			Nmos_Set(1,6);//debug led
			//vGu8Key2_lock=1;//按键上锁
			vGu8TimeFlag_1 = 1;//定时器运行置位
			Nmos_Set(1,0);
			Nmos_Set(1,1);
			Nmos_Set(1,2);
			Nmos_Set(1,3);
			Nmos_Set(1,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T0) //3s时间到
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);//第1-5位灭
			if(vGu16TimeCnt_1>=(U5_T1+T0)) //第五路关断时间截止
				Gu8Step++;
			break;
		case 2:
			Nmos_Set(1,4);//第5位亮
			if(vGu16TimeCnt_1>=(U5_T1+T0+T2)) //第五路导通时间截止
				Gu8Step++;		
			break;	
		case 3:
			Nmos_Set(0,4);//第5位灭
			if(vGu16TimeCnt_1>=(U4_T1+T0)) //第5路关断时间截止
				Gu8Step++;
			break;
		case 4:
			Nmos_Set(1,3);//第4位亮
			if(vGu16TimeCnt_1>=(U4_T1+T0+T2)) //第4路导通时间截止
				Gu8Step++;
			break;
		case 5:
			Nmos_Set(0,3);//第4位灭
			if(vGu16TimeCnt_1>=(U3_T1+T0)) //第3路关断时间截止
				Gu8Step++;
			break;
		case 6:
			Nmos_Set(1,2);//第3位亮
			if(vGu16TimeCnt_1>=(U3_T1+T0+T2)) //第3路导通时间截止
				Gu8Step++;
			break;
		case 7:
			Nmos_Set(0,2);//第3位灭
			if(vGu16TimeCnt_1>=(U2_T1+T0)) //第2路关断时间截止
				Gu8Step++;
			break;
		case 8:
			Nmos_Set(1,1);//第2位亮
			if(vGu16TimeCnt_1>=(U2_T1+T0+T2)) //第2路导通时间截止
				Gu8Step++;
			break;
		case 9:
			Nmos_Set(0,1);//第2位灭
			if(vGu16TimeCnt_1>=(U1_T1+T0)) //第1路关断时间截止
				Gu8Step++;
			break;
		case 10:
			Nmos_Set(1,0);//第1位亮
			if(vGu16TimeCnt_1>=(U1_T1+T0+T2)) //第1路导通时间截止
				Gu8Step++;
			break;
		case 11:
			Nmos_Set(0,0);//第1位灭
			if(vGu16TimeCnt_1>=T3) //T3s时间到,全部熄灭可以接受控制
				Gu8Step++;
			break;
		case 12:
			Gu8Step = 0;//step 清0
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//清定时器运行标志
			vGu8Key2_lock = 0;//解按键锁
			break;
		default:	
			break;
	}
}
void Key3_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			Nmos_Set(1,6);//debug led
			//vGu8Key3_lock=1;//按键上锁
			vGu8TimeFlag_1 = 1;//定时器运行置位
			Nmos_Set(1,0);
			Nmos_Set(1,1);
			Nmos_Set(1,2);
			Nmos_Set(1,3);
			Nmos_Set(0,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T0) //T0时间到
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);//第1-5位灭
			
			if(vGu16TimeCnt_1>=(U4_T1+T0)) //第4路关断时间截止
				Gu8Step++;
			break;
		case 2:
			Nmos_Set(1,3);//第4位亮
			if(vGu16TimeCnt_1>=(U4_T1+T0+T2)) //第4路导通时间截止
				Gu8Step++;		
			break;	
		case 3:
			Nmos_Set(0,3);//第4位灭
			if(vGu16TimeCnt_1>=(U3_T1+T0)) //第3路关断时间截止
				Gu8Step++;
			break;
		case 4:
			Nmos_Set(1,2);//第3位亮
			if(vGu16TimeCnt_1>=(U3_T1+T0+T2)) //第3路导通时间截止
				Gu8Step++;
			break;
		case 5:
			Nmos_Set(0,2);//第3位灭
			if(vGu16TimeCnt_1>=(U2_T1+T0)) //第2路关断时间截止
				Gu8Step++;
			break;
		case 6:
			Nmos_Set(1,1);//第2位亮
			if(vGu16TimeCnt_1>=(U2_T1+T0+T2)) //第2路导通时间截止
				Gu8Step++;
			break;
		case 7:
			Nmos_Set(0,1);//第2位灭
			if(vGu16TimeCnt_1>=(U1_T1+T0)) //第1路关断时间截止
				Gu8Step++;
			break;
		case 8:
			Nmos_Set(1,0);//第1位亮
			if(vGu16TimeCnt_1>=(U1_T1+T0+T2)) //第1路导通时间截止
				Gu8Step++;
			break;
		case 9:
			Nmos_Set(0,0);//第1位灭
			if(vGu16TimeCnt_1>=T3) //T3s时间到,全部熄灭可以接受控制
				Gu8Step++;
			break;
		case 10:
			vGu8Key3_lock = 0;//解按键锁
			vGu8TimeFlag_1 = 0;//清定时器运行标志
			Gu8Step = 0;//step 清0
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//清定时器运行标志
			vGu8Key3_lock = 0;//解按键锁
			break;
		default:	
			break;
	}
}

void Key4_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			Nmos_Set(1,6);//debug led
			vGu8Key4_lock=1;//按键上锁
			vGu8TimeFlag_1 = 1;//定时器运行置位
			Nmos_Set(1,0);
			Nmos_Set(1,1);
			Nmos_Set(1,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T0) //T0时间到
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);//第1-5位灭
			if(vGu16TimeCnt_1>=(U3_T1+T0)) //第3路关断时间截止
				Gu8Step++;
			break;
		case 2:
			Nmos_Set(1,2);//第3位亮
			if(vGu16TimeCnt_1>=(U3_T1+T0+T2)) //第3路导通时间截止
				Gu8Step++;		
			break;	
		case 3:
			Nmos_Set(0,2);//第3位灭
			if(vGu16TimeCnt_1>=(U2_T1+T0)) //第2路关断时间截止
				Gu8Step++;
			break;
		case 4:
			Nmos_Set(1,1);//第2位亮
			if(vGu16TimeCnt_1>=(U2_T1+T0+T2)) //第2路导通时间截止
				Gu8Step++;
			break;
		case 5:
			Nmos_Set(0,1);//第2位灭
			if(vGu16TimeCnt_1>=(U1_T1+T0)) //第1路关断时间截止
				Gu8Step++;
			break;
		case 6:
			Nmos_Set(1,0);//第2位亮
			if(vGu16TimeCnt_1>=(U1_T1+T0+T2)) //第1路导通时间截止
				Gu8Step++;
			break;
		case 7:
			Nmos_Set(0,0);//第1位灭
			if(vGu16TimeCnt_1>=T3) //T3s时间到,全部熄灭可以接受控制
				Gu8Step++;
			break;
		case 8:
			Gu8Step = 0;//step 清0
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//清定时器运行标志
			vGu8Key4_lock = 0;//解按键锁
			break;
		default:	
			break;
	}
}
void Key5_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			Nmos_Set(1,6);//debug led
			//vGu8Key5_lock=1;//按键上锁
			vGu8TimeFlag_1 = 1;//定时器运行置位
			Nmos_Set(1,0);
			Nmos_Set(1,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T0) //T0时间到
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);//第1-5位灭
			if(vGu16TimeCnt_1>=(U2_T1+T0)) //第2路关断时间截止
				Gu8Step++;
			break;
		case 2:
			Nmos_Set(1,1);//第2位亮
			if(vGu16TimeCnt_1>=(U2_T1+T0+T2)) //第2路导通时间截止
				Gu8Step++;		
			break;	
		case 3:
			Nmos_Set(0,1);//第2位灭
			if(vGu16TimeCnt_1>=(U1_T1+T0)) //第1路关断时间截止
				Gu8Step++;
			break;
		case 4:
			Nmos_Set(1,0);//第1位亮
			if(vGu16TimeCnt_1>=(U1_T1+T0+T2)) //第1路导通时间截止
				Gu8Step++;
			break;
		case 5:
			Nmos_Set(0,0);//第1位灭
			if(vGu16TimeCnt_1>=T3) //T3s时间到,全部熄灭可以接受控制
				Gu8Step++;
			break;
		case 6:
			Gu8Step = 0;//step 清0
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//清定时器运行标志
			vGu8Key5_lock = 0;//解按键锁		
			break;
		default:	
			break;
	}
}
void Key6_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			//vGu8Key6_lock=1;//按键上锁
			Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 1;//定时器运行置位	
			Nmos_Set(1,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);
			Nmos_Set(1,5);//
			if(LED_TIME_3S<=vGu16TimeCnt_1) //3s时间到
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);//第1-5位灭
			if(vGu16TimeCnt_1>=(U1_T1+T0)) //第1路关断时间截止
				Gu8Step++;
			break;
		case 2:
			Nmos_Set(1,0);//第1位亮
			if(vGu16TimeCnt_1>=(U1_T1+T0+T2)) //第1路导通时间截止
				Gu8Step++;		
			break;	
		case 3:
			Nmos_Set(0,0);//第1位灭
			if(vGu16TimeCnt_1>=T3) //T3s时间到,全部熄灭可以接受控制
				Gu8Step++;
			break;
		case 4:
			vGu8Key6_lock = 0;//解按键锁
			vGu8TimeFlag_1 = 0;//清定时器运行标志
			Gu8Step = 0;//step 清0
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//清定时器运行标志
			vGu8Key6_lock = 0;//解按键锁
			break;
		default:	
			break;
	}
}
void Key7_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			//vGu8Key7_lock=1;//按键上锁
			Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 1;//定时器运行置位
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T3) //T3s时间到,全部熄灭可以接受控制
				Gu8Step++;
			break;
		case 1:
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//清定时器运行标志
			vGu8Key7_lock = 0;//解按键锁
			Gu8Step = 0;//step 清0			
			break;
		default:	
			break;
	}
}

void Key8_Fun(void)
{	
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			Nmos_Set(1,0xff);//全部关闭
			Nmos_Set(0,6);//debug led
			vGu8TimeFlag_1 = 1;//定时器运行置位
			//print_char(0x22);
			if(vGu16TimeCnt_1>=T3) //T3s时间到,全部熄灭可以接受控制
				Gu8Step++;
			break;
		case 1:
			vGu8Key1_lock = 0;
			vGu8Key2_lock = 0;
			vGu8Key3_lock = 0;
			vGu8Key4_lock = 0;
			vGu8Key5_lock = 0;
			vGu8Key6_lock = 0;
			vGu8Key7_lock = 0;
			
			KeyCode = 0;
			vGu8TimeFlag_1 = 0;//清定时器运行标志
			vGu8Key8_lock = 0;//解按键锁
			Gu8Step = 0;//step 清0			
			break;
		default:	
			break;
	}
}
int Check_KeyStatus(void)
{
	if(vGu8Key7_lock || vGu8Key6_lock || vGu8Key5_lock || vGu8Key4_lock || vGu8Key3_lock || vGu8Key2_lock || vGu8Key1_lock)
		return 1;
	else 
		return 0;	
}