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
#include "USART.h"
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
	UARTx_config(2);//串口2初始化
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
#if defined UART1_TIMER1
void uart1_int (void) interrupt UART1_VECTOR
{
	if(RI)//接收中断
	{
		RI = 0;
		Rec_Buf[RX_CONT] = SBUF;                    //把串口1缓存SBUF寄存器数据依次存放到数组Rec_Buf2中
		RX_CONT++;                                   
	  if(RX_CONT>Buf_Max)                          //接收数大于定义接收数组最大个数时，覆盖接收数组之前值
		{
			RX_CONT = 0;
		}           
	}
	if(TI)//发送中断
	{
		TI = 0;
		B_TX1_Busy = 0;
	}
}
#endif
#if defined FAKE_UART3
//========================================================================
// 函数: void   timer2_int (void) interrupt 12
// 描述: Timer2中断处理程序.
// 参数: None
// 返回: none.
// 版本: V1.0, 2012-11-22
//========================================================================
void timer2_int (void) interrupt 12
{
    if(Rx3_Ring)        //已收到起始位
    {
        if (--Rx3_BitCnt == 0)      //接收完一帧数据
        {
            Rx3_Ring = 0;           //停止接收
            Rx3_BUF = Rx3_DAT;      //存储数据到缓冲区
            RX3_End = 1;
            AUXR &=  ~(1<<4);   //Timer2 停止运行
            INT_CLKO |=  (1 << 6);  //允许INT4中断
        }
        else
        {
            Rx3_DAT >>= 1;                  //把接收的单b数据 暂存到 RxShiftReg(接收缓冲)
            if(P_RX3) Rx3_DAT |= 0x80;      //shift RX data to RX buffer
        }
    }

    if(Tx3_Ting)                    // 不发送, 退出
    {
        if(Tx3_BitCnt == 0)         //发送计数器为0 表明单字节发送还没开始
        {
            P_TX3 = 0;              //发送开始位
            Tx3_DAT = Tx3_BUF;      //把缓冲的数据放到发送的buff
            Tx3_BitCnt = 9;         //发送数据位数 (8数据位+1停止位)
        }
        else                        //发送计数器为非0 正在发送数据
        {
            if (--Tx3_BitCnt == 0)  //发送计数器减为0 表明单字节发送结束
            {
                P_TX3 = 1;          //送停止位数据
                Tx3_Ting = 0;       //发送停止
            }
            else
            {
                Tx3_DAT >>= 1;      //把最低位送到 CY(益处标志位)
                P_TX3 = CY;         //发送一个bit数据
            }
        }
    }
}


/********************* INT4中断函数 *************************/
void Ext_INT4 (void) interrupt 16
{
    AUXR &=  ~(1<<4);   //Timer2 停止运行
    T2H = (65536 - (UART3_BitTime / 2 + UART3_BitTime)) / 256;  //起始位 + 半个数据位
    T2L = (65536 - (UART3_BitTime / 2 + UART3_BitTime)) % 256;  //起始位 + 半个数据位
    AUXR |=  (1<<4);    //Timer2 开始运行
    Rx3_Ring = 1;       //标志已收到起始位
    Rx3_BitCnt = 9;     //初始化接收的数据位数(8个数据位+1个停止位)
    
    INT_CLKO &= ~(1 << 6);  //禁止INT4中断
    T2H = (65536 - UART3_BitTime) / 256;    //数据位
    T2L = (65536 - UART3_BitTime) % 256;    //数据位
}
#endif
