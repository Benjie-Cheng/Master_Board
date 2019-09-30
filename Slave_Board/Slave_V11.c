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
#define  MAIN_Fosc		22118400L	//定义主时钟
#include "STC15Fxxxx.H"
#include<stdio.h>
#include <stdlib.h>
#define MAX              16             //随机数最大值
#define MIN              1              //随机数最小值
#define creat_random() (rand() % (MAX + 1 - MIN)+ MIN)      //产生随机数，用于数据加密
#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 中断频率, 1000次/秒
#define	Baudrate1	115200UL                                //通信波特率115200

#define	UART1_RX_LENGTH 5
#define	UART1_TX_LENGTH 8
#define GPIO_KEY64_BOARD
#define KEY_LED_GPIO P55

/********************** 公用量 ************************/
BOOL B_1ms;	 //1ms标志
u16	msecond; //1s计数
u8 cnt50ms;  //50ms计数
BOOL B_TX1_Busy;  //发送忙标志

/********************** 接收主机的协议定义 ************************/
//帧头+指令+数据长度+随机数+数据包+帧尾(未定义)
//MJX+cmd+0x03+随机数+（（继电器L8+继电器H8）+随机数）+帧尾
//-----------------------------------------------------------------
#define ID_LENGTH(x)  sizeof(x)/sizeof(u8)
#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //接收串口1缓存数组
u8 idata RxData[UART1_RX_LENGTH];//提取数据专用，随机数+两字节继电器状态
u8 code SEND_ID[] ="MJX";
u8 RX_CONT = 0;  
BOOL Hand(u8 *a);
enum {
	CMD_1 = 0x01,  //继电器状态指令
	CMD_2,		   //按键值指令
	CMD_3,//
	CMD_4,//
	CMD_MAX
};

/********************** 发送协议定义 ************************/
//MJX+cmd+0x02+随机数+（键值+随机数)+帧尾(未定义)
//-----------------------------------------------------------------
u8 tx_buffer_vale[UART1_TX_LENGTH]={'M','J','X',CMD_2,0x02,0x00,0x00,'*'};	 //发送数组,数据位key值 0~63+16


/********************** 8*8矩阵键盘 ************************/
#if defined GPIO_KEY64_BOARD
#define KEY_BOARD_GPIO_X P1
#define KEY_BOARD_GPIO_Y P2
u8 code Line_KeyTable[]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};	//线行码
u8 IO_KeyState =0,IO_KeyState_x =0,IO_KeyState_y =0,IO_KeyState_old = 0xff;	//行列键盘变量
u8	KeyCode= 0xff;	//给用户使用的键码, 0~63有效
#endif
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
//void uart_recv_event_proc(void);
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
/**************************************
功能描述：握手成功与否函数
入口参数：uint8 *a
返回值：位
***************************************/
BOOL Hand(u8 *a)
{ 
  if(strstr(Rec_Buf,a)!=NULL)     //判断字符串a是否是字符串Rec_Buf的子串
		return 1;                     //如果字符串a是字符串Rec_Buf的子串
	else
		return 0;                     //如果字符串a不是字符串Rec_Buf的子串
}

/**************************************
功能描述：清除缓存内容函数
入口参数：无
返回值：无
***************************************/
void CLR_Buf(void)
{
	u8 k;
	for(k=0;k<Buf_Max;k++)        //将串口2缓存数组的值都清为零  
	{
		Rec_Buf[k] = 0;
	}
    RX_CONT = 0;                    
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
	timer0_init();
	uart1_config();
	A_HC595_MR = 1;//复位禁止
	A_HC595_OE = 0;//使能芯片
}

//========================================================================
// 描述: 按键指示灯 。
//========================================================================
void key_led_on(BOOL enable)
{
	if(enable)
		KEY_LED_GPIO = TRUE;
	else
		KEY_LED_GPIO = FALSE;
}
void key_led_reverse(void)
{
	KEY_LED_GPIO = ~KEY_LED_GPIO;	
}
/**************** 向HC595发送一个字节函数 ******************/
void data_in_595(u8 dat)
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
void Data_Out595()
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
			//print_char(KeyCode);
		}
			key_led_on(TRUE);//熄灭按键提示led
	}
	KEY_BOARD_GPIO_X = 0xff;
	KEY_BOARD_GPIO_Y = 0xff;
}
void Do_Keyboard_event(u8 key_code)
{	
	if(key_code){
		tx_buffer_vale[5] = creat_random();//产生随机数
		tx_buffer_vale[6] = KeyCode - 16 + tx_buffer_vale[5];//随机数绑定在键值中发送，接收需要解密
		print_string(tx_buffer_vale);
	}
}
#endif
//========================================================================
// 函数: void Rcev_DataAnalysis(void)
// 描述: uart读取数据包校验及处理。
// 参数: nine.
// 返回: none.
// 版本: VER1.0
// 日期: 2019-9-20
// 备注: 
//========================================================================
//cmd：MJX+cmd1+数据长度+随机数+595值;
void Switch_Led_DataProce(void)
{
	u8 i=0;
	for(i=0;i<Rec_Buf[ID_LENGTH(SEND_ID)];i++)
	{
		RxData[i] = Rec_Buf[i+ID_LENGTH(SEND_ID)+1];//取出数据长度字节后的随机数
		if(i==0){
			//do nothing
		}
		else
			RxData[i] = RxData[i] - RxData[0];
			Data_In595(RxData[i]);//先发高位，再发低位
	}
	Data_Out595();
}
void Rcev_DataAnalysis(void)
{
	u8 recv_cmd = 0;
	if(Hand(SEND_ID))
	{
#if defined FAKE_SERIAL
		Fake_PrintString(SEND_ID);
		Fake_PrintString(":");
		Fake_PrintString("\r\n");
#endif
		recv_cmd = Rec_Buf[ID_LENGTH(SEND_ID)-1];//长度为字符+\0,获取指令
		switch(recv_cmd)
		{
			case CMD_1://LED Channel Data
			#if defined FAKE_SERIAL
					Fake_PrintString("74hc595 LED CMD!\r\n");
			#endif
					Switch_Led_DataProce();
				break;
			default:
			#if defined FAKE_SERIAL
				Fake_PrintString("ERROR CMD!\r\n");
			#endif
				break;
		}		
		CLR_Buf();
	}
}
/********************** 主函数 ************************/
void main(void)
{
	stc15x_hw_init();
	Data_In595(0x00);
	Data_In595(0x00);
	Data_Out595();	//开机默认关闭通道显示LED
	CLR_Buf();      //清除接收缓存
#if defined FAKE_SERIAL
	Fake_PrintString("Keyboard system init ok !\r\n");	//模拟串口发送
#endif
	while(1)
	{
		if(B_1ms)	//1ms到
		{
			B_1ms = 0;
			if(++msecond >= 1000)	//1秒到
			{	
				msecond = 0;
				//key_led_reverse();
			}
		  if(++cnt50ms >= 50)		//50ms扫描一次行列键盘
			{
				cnt50ms = 0;
		#if defined GPIO_KEY64_BOARD
				Gpio_Keyscan();
		#endif
			}
		#if defined GPIO_KEY64_BOARD
			if(KeyCode > 0)		//有键按下
			{
				Do_Keyboard_event(KeyCode);//send KeyCode to master
				KeyCode = 0;
			}
		#endif
			Rcev_DataAnalysis();//帧头检验，数据分析处理
		}
	}
} 
//========================================================================
// 描述: Timer0 1ms中断函数。
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	B_1ms = 1;		//1ms标志
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
	tx1_cnt = 0;
	rx1_cnt = 0;
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
	u8 i;
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
