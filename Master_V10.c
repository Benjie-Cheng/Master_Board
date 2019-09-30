/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* 如果要在程序中使用此代码,请在程序中注明使用了版权 ---------------   */
/*---------------------------------------------------------------------*/

/*************	本程序功能说明	**************

用STC的MCU接收蓝牙，wifi、无线、按键指令，实现继电器控制，流水灯带作业。
1、本机串口接收从机8*8矩阵键盘按键码控制继电器。
	//{SEND_ID,0x00,0x00,SEND_END};帧头+随机数+按键码+帧尾	 //按键码需要解密
2、蓝牙接收数据。
3、wifi接收数据。
4、无线接收数据。
5、控制继电器。

存在缺陷：
1、。
2、。

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

#define	UART1_RX_LENGTH 2
#define	UART1_TX_LENGTH 6
#define GPIO_KEY64_BOARD
#define KEY_LED_GPIO P55

//#define SERIAL_DEBUG 
/********************** 公用量 ************************/
#define setbit(x,y) x|=(1<<y)         //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y)        //将X的第Y位清0
#define reversebit(x,y)  x^=(1<<y)    //将x的第Y位取反
BOOL B_1ms;	 //1ms标志
u16	msecond; //1s计数
u8 cnt50ms;  //50ms计数

u8 tx1_cnt;	//发送计数
u8 rx1_cnt;	//接收计数
BOOL rx_end_flag; //数据处理计数
BOOL B_TX1_Busy;  //发送忙标志

/********************** 接收从机发送协议 ************************/
//帧头+随机数加密+（键值+随机数）+帧尾
#define REC_ID 0xAA
#define REC_END 0xFF
u8 idata RX1_Buffer[UART1_RX_LENGTH];	//接收缓冲，两字节继电器状态
BOOL rec_start = 0;
#define	UART1_RX_LENGTH 2
u8 idata RxData[UART1_RX_LENGTH];	//接收缓冲，随机数+按键码
/********************** 主机发送协议定义 ************************/
#define SEND_ID 0x1A
#define SEND_END 0x3A
u8 switch_status_low = 0;
u8 switch_status_high = 0;
//帧头+随机数加密+(继电器状态(L,H)+随机数)+帧尾
u8 tx_buffer_vale[UART1_TX_LENGTH]={SEND_ID,0x00,0x00,0x00,SEND_END,'*'};	 //发送数组,发送随机数+继电器状态
u8 cmd_data = 0xff;//为0~63实体按键码
u8 switch_status0 = 0x00;//第一个继电器状态，高关断
u8 switch_status1 = 0x00;//第二个继电器状态，高关断
/********************** A 给继电器用的595 ************************/
//输出高-》反相器-》低-》导通继电器（LED亮）
sbit	B_HC595_SER   = P0^4;	//pin 04	SER		data input
sbit	B_HC595_RCLK  = P0^1;	//pin 01	RCLk	store (latch) clock
sbit	B_HC595_SRCLK = P0^3;	//pin 03	SRCLK	Shift data clock
sbit	B_HC595_OE    = P0^0;	//pin 00	OE 		低电平 使能enable pin
sbit	B_HC595_MR    = P0^2;	//pin 02	低电平复位	


void uart1_config();	// 选择波特率, 2: 使用Timer2做波特率, 其它值: 使用Timer1做波特率.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void uart_recv_event_proc(void);
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
	timer0_init();
	uart1_config();
	B_HC595_MR = 1;//复位禁止
	B_HC595_OE = 0;//使能芯片
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
/**************** 向继电器HC595发送一个字节函数 ******************/
void switch_data_in_595(u8 dat)
{		
	u8	i;
	for(i=0; i<8; i++)
	{
		dat <<= 1;
		B_HC595_SER   = CY;
		B_HC595_SRCLK = 0;
		NOP2();
		B_HC595_SRCLK = 1;
	}
}
/**************** 继电器HC595数据锁存函数 ******************/
void switch_data_out_595()
{		
	B_HC595_RCLK = 0;
	NOP2();
	B_HC595_RCLK = 1;
}
/**************** 根据按键值动作继电器 ******************/
//暂时只支持16位
void switch_channel_ctrl(u8 switch_cmd)
{
	if(switch_cmd < 8)
		reversebit(switch_status0,switch_cmd);//将该位取反
	else if((switch_cmd-8) < 8)
		reversebit(switch_status1,switch_cmd-8);//将该位取反
	else {
		
	}
	switch_data_in_595(switch_status1);//发送高8位
	switch_data_in_595(switch_status0);//发送低8位
	switch_data_out_595();	
	//LED显示低点起开关状态
	tx_buffer_vale[1] = creat_random();//产生随机数
	tx_buffer_vale[2] = switch_status0 + tx_buffer_vale[1];
	tx_buffer_vale[3] = switch_status1 + tx_buffer_vale[1];
	//tx_buffer_vale[4] = SEND_END;
	//print_char('*');
	print_string(tx_buffer_vale);
}
void multi_event_proc(void)
{
	if(cmd_data !=0xff){
		switch_channel_ctrl(cmd_data);//执行串口收到的按键值，动作继电器
		cmd_data = 0xff;
	}
	
}
//========================================================================
// 函数: void uart_recv_event_proc(void)
// 描述: uart读取数据包校验及处理。
// 参数: nine.
// 返回: none.
// 版本: VER1.0
// 日期: 2019-9-20
// 备注: 
//========================================================================
void uart_recv_event_proc(void)
{
	if(rx_end_flag)//校验自定义ID
	{
		RxData[1] = RxData[1] - RxData[0];
		cmd_data = RxData[1];
		//print_char(cmd_data);
		//print_char('\n');
		rx_end_flag = 0;
	}	
}
/********************** 主函数 ************************/
void main(void)
{
	stc15x_hw_init();
	switch_data_in_595(0x00);//关闭低8位继电器
	switch_data_in_595(0x00);//关闭高8路继电器
	switch_data_out_595();   //开机默认关闭8路继电器
#if defined SERIAL_DEBUG
	//print_string("STC15W408as UART1 init OK!\r\n");	//SUART1发送一个字符串
#endif
	while(1)
	{

		if(B_1ms)	//1ms到
		{
			B_1ms = 0;
			if(++msecond >= 1000)	//1秒到
			{	
				msecond = 0;	
				//switch_data_in_595(0x33);//关闭低8位继电器
				//switch_data_out_595();   //开机默认关闭8路继电器
				key_led_reverse();
			}
		  if(++cnt50ms >= 50)		//50ms扫描一次行列键盘
			{
				cnt50ms = 0;	
			}
			uart_recv_event_proc();
			multi_event_proc();//按键，红外，wifi,蓝牙值控制继电器
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
//测试收到的数据是否正常
#if defined SERIAL_DEBUG
		//print_char(SBUF);
#endif
		if(SBUF == REC_ID)//自定义帧头
		{
			rec_start= 1;
			rx1_cnt = 0;
			rx_end_flag = 0;
		}
		else if(SBUF ==REC_END)
		{
			rec_start = 0;
			for(i=0; i < rx1_cnt; i++)
			{
				RxData[i] =RX1_Buffer[i];
			}
			rx_end_flag = 1;//告诉系统已接收一个完整的数据包 
		}
		else if(rec_start)                                              //判断是否处于接收状态
		{
			RX1_Buffer[rx1_cnt++] = SBUF;
		}

	}

	if(TI)
	{
		TI = 0;
		B_TX1_Busy = 0;
	}
}
