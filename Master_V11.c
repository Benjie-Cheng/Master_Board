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

配置：
定时器0 用于1ms定时;
定时器1 用于串口1波特率发生,给蓝牙使用,蓝牙默认9600;
定时器2 用于实现模拟串口P3.0 P3.1，用于与键盘板通信,115200;

1、本机串口接收从机8*8矩阵键盘按键码控制继电器。 
	帧头+指令+数据长度+随机数+按键码+帧尾(暂未定义) //按键码需要解密
2、蓝牙接收数据。
3、wifi接收数据。
4、无线接收数据。
5、控制继电器。

存在缺陷：
1、模拟串口uart3收发数据不稳定，优化项
V1.1记录：
1、修改蓝牙握手成功提示方法。                         20190930
2、增加按键解析，控制595的开关，并将通道值发给按键板  20190930
******************************************/
#include "debug.h"
#include "ws2811.h"
//#define  MAIN_Fosc		22118400L	//定义主时钟
//#include "STC15Fxxxx.H" //移至 debug.h
#include<stdio.h>
#include <stdlib.h>
#define MAX              16             //随机数最大值
#define MIN              1              //随机数最小值
#define creat_random() (rand() % (MAX + 1 - MIN)+ MIN)      //产生随机数，用于数据加密
#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 中断频率, 1000次/秒


//========================================================================
/*************	本地变量声明	**************/
BOOL B_1ms;	 //1ms标志
u16	msecond; //1s计数
u8 cnt50ms;  //50ms计数
u8 cnt10ms;  //10ms计数
u8 code SEND_ID[] ="MJX";
/*u8 code SEND_END[] ="tel:15901856750";*/   //暂时未用到
#define ID_LENGTH(x)  sizeof(x)/sizeof(u8)
u8 idata RxData[5];
enum {
	CMD_1 = 0x01,//蓝牙
	CMD_2,//Key_board
	CMD_3,//wifi
	CMD_4,//PT2272
	CMD_MAX
};
//========================================================================
// 描述: Uart1 蓝牙模块使用定义。
#define UART1_TIMER1

#if defined UART1_TIMER1
#define	Baudrate1	9600UL 
BOOL B_TX1_Busy;  //发送忙标志
void uart1_config(void);
#endif
//========================================================================
// 描述: B 给继电器用的595
//输出高-》反相器-》低-》导通继电器（LED亮）
sbit	B_HC595_SER   = P0^4;	//pin 04	SER		data input
sbit	B_HC595_RCLK  = P0^1;	//pin 01	RCLk	store (latch) clock
sbit	B_HC595_SRCLK = P0^3;	//pin 03	SRCLK	Shift data clock
sbit	B_HC595_OE    = P0^0;	//pin 00	OE 		低电平 使能enable pin
sbit	B_HC595_MR    = P0^2;	//pin 02	低电平复位	
enum {
	CMD_1 = 0x01,  //按键板收led 的状态指令
	CMD_2,//
	CMD_3,//
	CMD_4,//
	CMD_MAX
};
u8 tx_buffer_vale[9]={'M','J','X',CMD1,0x03,0x00,0x00,0x00,'*'};	 //发送数组,发送随机数+继电器状态
u8 switch_status0 = 0x00;//第一个继电器状态，高关断
u8 switch_status1 = 0x00;//第二个继电器状态，高关断
//========================================================================
// 描述: Timer2模拟串口P3.0接收, P3.1发送 与副板通信使用.
#define FAKE_UART3
#if defined FAKE_UART3

#define UART3_Baudrate  9600UL    //定义波特率
#define UART3_BitTime   (MAIN_Fosc / UART3_Baudrate)
sbit P_RX3 = P3^0;  //定义模拟串口接收IO
sbit P_TX3 = P3^1;  //定义模拟串口发送IO

u8  Tx3_DAT;        // 发送移位变量, 用户不可见
u8  Rx3_DAT;        // 接收移位变量, 用户不可见
u8  Tx3_BitCnt;     // 发送数据的位计数器, 用户不可见
u8  Rx3_BitCnt;     // 接收数据的位计数器, 用户不可见
u8  Rx3_BUF;        // 接收到的字节, 用户读取
u8  Tx3_BUF;        // 要发送的字节, 用户写入
bit Rx3_Ring;       // 正在接收标志, 低层程序使用, 用户程序不可见
bit Tx3_Ting;       // 正在发送标志, 用户置1请求发送, 底层发送完成清0
bit RX3_End;        // 接收到一个字节, 用户查询 并清0
u8  Rx3_BUF;
u8  Tx3_read;           //发送读指针
u8  Rx3_write;          //接收写指针
#define RX_Lenth        32          //接收长度
u8  idata   buf3[RX_Lenth]; //接收缓冲
u16 RxTimeOut;
bit B_RxOk;     //接收结束标志
#endif

//================================================================
// 描述: MLT_BT05,串口1，P3.6,P3.7
/****************************************************************
AT指令集
*****************************************************************/	
char code str1[]="AT\r\n";                                    		    //  AT测试，返回"OK"
char code str2[]="AT+DEFAULT\r\n";                         		        //  恢复出厂
char code str3[]="AT+BAUD\r\n";     			      					//  查询波特率+BAUD=8：115200 +BAUD=8：9600 默认
//char code str4[]="AT+RESET\r\n";                                   	// 	复位
//char code str5[]="AT+ROLE1\r\n";    									//  主从设置
//char code str6[]="AT+PIN\r\n";   										//  密码
//char code str8[]="AT+HELP\r\n";   									//  帮助

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];     //接收串口缓存数组
u8 RX_CONT = 0;  
BOOL Hand(u8 *a);
void CLR_Buf(void);


void delay_ms(u8 ms)
{
   u16 i;
	 do{
	      i = MAIN_Fosc / 13000;
		  while(--i)	;   //14T per loop
     }while(--ms);
}
void DelayMS(u16 t)
{
	while(t--)
	{
		delay_ms(1);
	}
}
//定时器0初始化
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
// 描述: UART初始化程序.
//========================================================================
#if defined FAKE_UART3
void UART3_Init(void)
{
	Tx3_read  = 0;
    Rx3_write = 0;
    Tx3_Ting  = 0;
    Rx3_Ring  = 0;
    RX3_End   = 0;
    Tx3_BitCnt = 0;
    RxTimeOut = 0;
    B_RxOk = 0;

    AUXR &=  ~(1<<4);       // Timer2 停止运行
    T2H = (65536 - UART3_BitTime) / 256;    // 数据位
    T2L = (65536 - UART3_BitTime) % 256;    // 数据位
    INT_CLKO |=  (1 << 6);  // 允许INT4中断
    IE2  |=  (1<<2);        // 允许Timer2中断
    AUXR |=  (1<<2);        // 1T
	EA = 1;                 //开总中断
}
void test(void)
{
/*	
  if(RxTimeOut != 0)      // 超时时间是否非0?
  {
      if(--RxTimeOut == 0)    // (超时时间  - 1) == 0?
      {
          B_RxOk = 1;
				 AUXR &=  ~(1<<4);   //Timer2 停止运行
                INT_CLKO &= ~(1 << 6);  //禁止INT4中断
                T2H = (65536 - UART3_BitTime) / 256;    //数据位
                T2L = (65536 - UART3_BitTime) % 256;    //数据位
                AUXR |=  (1<<4);    //Timer2 开始运行
      }
  } 
  if(B_RxOk)      // 检测是否接收OK
  {	
		B_RxOk = 0;
		if(Hand("chg"))
			Fake_PrintString("Uart3 recv data ok!\r\n");
			CLR_Buf();

    }
	*/
	 if (RX3_End)        // 检测是否收到一个字节
        {
            RX3_End = 0;    // 清除标志
            buf3[Rx3_write] = Rx3_BUF;  // 写入缓冲
            if(++Rx3_write >= RX_Lenth) Rx3_write = 0;  // 指向下一个位置,  溢出检测
            RxTimeOut = 1000;   //装载超时时间
        }
        if(RxTimeOut != 0)      // 超时时间是否非0?
        {
            if(--RxTimeOut == 0)    // (超时时间  - 1) == 0?
            {
                B_RxOk = 1;
                AUXR &=  ~(1<<4);   //Timer2 停止运行
                INT_CLKO &= ~(1 << 6);  //禁止INT4中断
                T2H = (65536 - UART3_BitTime) / 256;    //数据位
                T2L = (65536 - UART3_BitTime) % 256;    //数据位
                AUXR |=  (1<<4);    //Timer2 开始运行
            }
        }
        
        if(B_RxOk)      // 检测是否接收OK?
        {
            if (!Tx3_Ting)      // 检测是否发送空闲
            {
                if (Tx3_read != Rx3_write)  // 检测是否收到过字符
                {
                    Tx3_BUF = buf3[Tx3_read];   // 从缓冲读一个字符发送
                    Tx3_Ting = 1;               // 设置发送标志
                    if(++Tx3_read >= RX_Lenth)  Tx3_read = 0;   // 指向下一个位置,  溢出检测
                }
                else
                {
                    B_RxOk = 0;
                    AUXR &=  ~(1<<4);       //Timer2 停止运行
                    INT_CLKO |=  (1 << 6);  //允许INT4中断
                }
            }
        }
}
#endif	
/**************** 向继电器HC595发送一个字节函数 ******************/
void Switch_DataIn595(u8 dat)
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
void Switch_DataOut595()
{		
	B_HC595_RCLK = 0;
	NOP2();
	B_HC595_RCLK = 1;
}
/**************** 继电器初始化 ******************/
void Switch_ChannelIint()
{
	Switch_DataIn595(switch_status1);//关闭高8路继电器
	Switch_DataIn595(switch_status0);//关闭低8位继电器
	Switch_DataOut595();   //开机默认关闭8路继电器
	//print_string(tx_buffer_vale);模拟串口报告给按键板指示灯状态
}
/*********************按键值控制继电器*********************/
/*Function List: 按键值控制继电器
*switch_cmd：是控制的通道值，经过解码后的按键数值
*通道值需要取反后发送到595，595输出经过反相器。
*暂时只支持16位，
*switch_status0：代表低8位值
*/
void Switch_ChannelCtrl(u8 switch_cmd)
{
	if(switch_cmd < 8)
		reversebit(switch_status0,switch_cmd);//将该位取反
	else if((switch_cmd-8) < 8)
		reversebit(switch_status1,switch_cmd-8);//将该位取反
	else {
		
	}
	Switch_DataIn595(switch_status1);//发送高8位
	Switch_DataIn595(switch_status0);//发送低8位
	Switch_DataOut595();	
	//LED显示，低电平显示开关状态
	tx_buffer_vale[5] = creat_random();//产生随机数
	tx_buffer_vale[6] = switch_status1 + tx_buffer_vale[5];//先发高位
	tx_buffer_vale[7] = switch_status0 + tx_buffer_vale[5];//再发低位
	//print_string(tx_buffer_vale);模拟串口稳定后发送
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
	uart1_config();//定时器1产生波特率串口初始化,用于蓝牙
	UART3_Init();  //PCA初始化，用于小板通信
}
//========================================================================
// 函数: set_timer1_baudraye(u16 dat)
// 描述: 设置Timer1做波特率发生器。
// 参数: dat: Timer1的重装值.
// 返回: none.
// 版本: VER1.0
// 日期: 2014-11-28
// 备注: 
//========================================================================
#if defined UART1_TIMER1
void set_timer1_baudraye(u16 dat)
{
	Timer1_Stop();     //Timer stop
	Timer1_AsTimer();  //Timer2 set As Timer
	Timer1_1T();       //Timer2 set as 1T mode
	Timer1_Load(dat);  //波特率设置
	Timer1_InterruptDisable();  //禁止中断
	Timer1_Run();      //Timer run enable
}

void uart1_config(void)
{
	/*********** 波特率使用定时器1 *****************/
	S1_BRT_UseTimer1();//S1 BRT Use Timer1;
	Timer1_16bitAutoReload(); //定时器1为模式0(16位自动重载)
	set_timer1_baudraye(65536UL - (MAIN_Fosc / 4) / Baudrate1);
	//SCON = (SCON & 0x3f) | 0x40;	//UART1模式, 0x00: 同步移位输出, 0x40: 8位数据,可变波特率, 0x80: 9位数据,固定波特率, 0xc0: 9位数据,可变波特率
	S1_SHIFT();
	S1_8bit();
	S1_RX_Enable();
	//S1_USE_P30P31();//UART1 使用P30 P31口	默认
	S1_USE_P36P37();//UART1 使用P36_TX P37_RX口
	ES  = 1;	//允许中断
	REN = 1;	//允许接收
	EA = 1;
	B_TX1_Busy = 0;
}

void print_char(u8 dat)
{
		SBUF = dat;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
}
void print1_string(u8 *puts)
{
  for (; *puts != 0;	puts++)   	//遇到停止符0结束
	{
		SBUF = *puts;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
	}
}
#endif

/**************************************
功能描述：握手成功与否函数
入口参数：uint8 *a
返回值：位
***************************************/
BOOL Hand(u8 *a)
{ 
  if(strstr(Rec_Buf,a)!=NULL)     //判断字符串a是否是字符串Rec_Buf的子串
		return 1;                 //如果字符串a是字符串Rec_Buf的子串
	else
		return 0;                 //如果字符串a不是字符串Rec_Buf的子串
}

/**************************************
功能描述：清除缓存内容函数
入口参数：无
返回值：无
***************************************/
void CLR_Buf(void)
{
	u8 k;
	for(k=0;k<Buf_Max;k++)        //将串口缓存数组的值都清为零  
	{
		Rec_Buf[k] = 0;
	}
    RX_CONT = 0;                    
}
//cmd：MJX+cmd1+数据长度+随机数+通道值;
void Bluetooth_DataProce(void)
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
	}
	if(RxData[1] == 1)
	{
		Fake_PrintString("recv success BT05 0x01!\r\n");
	}
	if(RxData[1] == 0x02)
	{
		Fake_PrintString("recv success BT05 0x02!\r\n");
	}
}
//cmd：MJX+cmd2+数据长度+随机数+通道值;
void Keyboard_DataProce(void)
{
	u8 i=0;
	
	for(i=0;i<Rec_Buf[ID_LENGTH(SEND_ID)];i++)
	{
		RxData[i] = Rec_Buf[i+ID_LENGTH(SEND_ID)+1];//取出数据长度字节后的随机数
		if(i==0){
			//do nothing
		}
		else{
			RxData[i] = RxData[i] - RxData[0];
			Switch_ChannelCtrl(RxData[i]);
			Fake_PrintString("recv success Key_Code to contral switch!\r\n");
		}
	}	
/*
	if(RxData[1] == 1)
	{
		Fake_PrintString("recv success Key_Code 0x01!\r\n");
	}
*/
}
void Rcev_DataAnalysis(void)
{
	u8 recv_cmd = 0;
	if(Hand(SEND_ID))
	{
		Fake_PrintString(SEND_ID);
		Fake_PrintString(":");
		Fake_PrintString("\r\n");
		recv_cmd = Rec_Buf[ID_LENGTH(SEND_ID)-1];//长度为字符+\0,获取指令
		switch(recv_cmd)
		{
			case CMD_1://蓝牙
					Fake_PrintString("BT05 CMD!\r\n");	
					Bluetooth_DataProce();
				break;
			case CMD_2://按键
					Fake_PrintString("Key_Code CMD!\r\n");
					Keyboard_DataProce();
				break;
			default:
				Fake_PrintString("ERROR CMD!\r\n");
				break;
		}		
		CLR_Buf();
	}
}

/********************** 主函数 ************************/
void main(void)
{
	u8 j;
	stc15x_hw_init();
	Switch_ChannelIint();//继电器初始化，全关闭
#if defined FAKE_SERIAL
	Fake_PrintString("STC15W408as fake UART init OK!\r\n");	//模拟串口9600，发送一个字符串
#endif
	CLR_Buf();//清除接收缓存
	do{
		print1_string(str1);	//握手指令
		DelayMS(500);
		if(++j>5){
			Fake_PrintString("BT05 hand failed\r\n");
			j = 0;				//Can add error led display
			break;
		}
	}while(!Hand("OK"))			//判断是否握手成功,如果不成功延时一会,再发送AT握手指令
	CLR_Buf(); 
	Fake_PrintString("BT05 hand OK\r\n");
	key_led_reverse();
	while(1)
	{
		if(B_1ms)	//1ms到
		{
			B_1ms = 0;
			if(++msecond >= 1000)	//1秒到
			{	
				msecond = 0;	
				key_led_reverse();
			}
		  if(++cnt50ms >= 50)		//50ms扫描一次行列键盘
			{
				cnt50ms = 0;	
			}
			if(++cnt10ms >= 10)		//10ms扫描是否有数据需要处理
			{
				cnt10ms = 0;	
				Rcev_DataAnalysis();
			}
			
		}
		test();	
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
					// Rx3_BUF = Rx3_DAT;      //存储数据到缓冲区
            RX3_End = 1;
				/*
					  Rec_Buf[Rx3_write] = Rx3_BUF;  // 写入缓冲
            if(++Rx3_write >= Buf_Max) Rx3_write = 0;  // 指向下一个位置,  溢出检测
						RxTimeOut = 1000;   //装载超时时间
					*/
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