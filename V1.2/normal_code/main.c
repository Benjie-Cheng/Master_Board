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

资源：
定时器0 用于模拟串口接收使用;
定时器1 用于串口1波特率发生,给蓝牙使用,蓝牙默认9600;
定时器2 用于用于1ms定时

1、本机串口接收从机8*8矩阵键盘按键码控制继电器。 
	帧头+指令+数据长度+随机数+按键码+帧尾(暂未定义) //按键码需要解密
2、蓝牙接收数据。串口1，中断方式,115200
3、wifi接收数据。
4、无线接收数据。
5、控制继电器。

存在缺陷：
1、模拟串口uart3收发数据不稳定，优化项
2、1中问题已经使用定时器0模拟多路串口接收解决

V1.1记录：
1、修改蓝牙握手成功提示方法。                         20190930
2、增加按键解析，控制595的开关，并将通道值发给按键板  20190930
V1.2记录：
1、蓝牙协议修改成适配APP方式，其他通信接口暂时不变。  20191008
2、内部存储4K SRAM 暂时未调通。                       20191008
待调试：
1、内部EEPROM 暂时未调通。                            20191008
2、2000路595电路。                                    20191008
******************************************/
#include "config.h"
#include "debug.h"
#include "led_lib.h"

//========================================================================
/*************	本地变量声明	**************/
BOOL B_1ms;	 //1ms标志
u16	msecond; //1s计数
u8 cnt50ms;  //50ms计数
u8 cnt10ms;  //10ms计数
u8 code SEND_ID[] ="MJX";
u8 code SEND_END[] ="0xFF";
#define ID_LENGTH(x)  sizeof(x)/sizeof(u8)
u8 xdata RxData[5];
enum {
	CMD_1 = 0x01,//蓝牙
	CMD_2,//Key_board
	CMD_3,//wifi
	CMD_4,//PT2272
	CMD_MAX
};
//========================================================================
//描述: Uart1 蓝牙模块使用定义。
#define UART1_TIMER1

#if defined UART1_TIMER1
#define	Baudrate1	115200UL 
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
	CMD1 = 0x01,  //按键板收led 的状态指令
	CMD2,//
	CMD3,//
	CMD4,//
};
#define	UART1_TX_LENGTH 9
u8 tx_buffer_vale[UART1_TX_LENGTH]={'M','J','X',0x01,0x03,0x00,0x00,0x00,'*'};	 //发送数组,发送随机数+继电器状态
u8 switch_status0 = 0x00;//第一个继电器状态，高关断
u8 switch_status1 = 0x00;//第二个继电器状态，高关断
//================================================================
// 描述: MLT_BT05,串口1，P3.6,P3.7
/****************************************************************
AT指令集
*****************************************************************/	
char code str1[]="AT\r\n";                                    		    //  AT测试，返回"OK"
char code str2[]="AT+DEFAULT\r\n";                         		        //  恢复出厂
char code str3[]="AT+BAUD\r\n";     			      					//  查询波特率+BAUD=8：115200 +BAUD=8：9600 默认
//char code str4[]="AT+RESET\r\n";                                   	// 	复位
char code test[]="debug test\r\n";   									//  
#define Buf_Max 32
u8 xdata Rec_Buf[Buf_Max];     //接收串口缓存数组

BOOL Hand(u8 *RxBuf,u8 *a);
void CLR_Buf(void);
//========================================================================
/*************	模拟串口的配置	**************/
#define BaudRate		9600		//模拟串口波特率
#define Timer0_Reload		(65536 - MAIN_Fosc / BaudRate / 3)
#define RxLength		32		//接收缓冲长度
//定义按键 模拟串口
sbit P_RXB_30 = P3^0;			//定义模拟串口接收IO
sbit P_RXB_36 = P3^4;			//定义模拟串口接收IO


bit	B_Rx_OK;	 		//接收完的标志位, 收到数据块系统设置1, 用户处理数据后必须清0			
u8 RX_CONT = 0;  //接收到的字节数, 用户处理数据后必须清0

//=====================================================================
//===== 下面这些为系统使用的常量或变量, 用户不可见，请勿修改 =============
#define	RxBitLenth	9		//8个数据位+1个停止位
#define	TxBitLenth	9		//8个数据位+1个停止位
u8  TxShiftReg,RxShiftReg;	//发送 接收 移位
u8  RxSample;		//发送和接收检测 采样计数器(3倍速率检测)
u8  TxBitCnt,RxBitCnt;		//发送和接收的数据 位计数器
u8	RxTimeOut;		//接收超时计数
bit	RxStartFlag;			//正在接收一个字节(已收到起始位)
u8 uart_flag =0;

//=====================================================================
u16 led_stat_cont = 0;
BOOL Recv_Ok = 0;
BOOL Recv_start =0;

//#define BOOL_WS2811_LED
#if defined(BOOL_WS2811_LED)
#define bit_to_u8(x) rem(x,8) ? (mod(x,8)+1) : mod(x,8)
u16 LED_StatByteSize = bit_to_u8(nWs);
u8 xdata LED_StatByte[bit_to_u8(nWs)];//nWs，为LED个数
u16 bool_led_count = 0;//接收到LED数据的个数。
void Clear_LedStatByte(void);
void WS2811_Display(void);
#else
u8 xdata LED_Buff[nWs];
void Clear_LedBuff(void);
#endif
/*************  外部函数和变量声明 *****************/
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


//========================================================================
// 函数: void UART_INIT(void)
// 描述: UART模块的初始变量.
// 参数: 无.
// 返回: 无.
// 版本: V1.0, 2012-10-22
//========================================================================
void UART_INIT(void)
{
	RxStartFlag = 0;
	RxSample = 4;
	RxTimeOut = 0;
	B_Rx_OK = 0;
	RX_CONT = 0;
}
BOOL Hand(u8 *RxBuf,u8 *a)
{ 
  if(strstr(RxBuf,a)!=NULL)           //判断字符串a是否是字符串Rec_Buf的子串
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
	for(k=0;k<Buf_Max;k++)        //将串口缓存数组的值都清为零  
	{
		Rec_Buf[k] = 0;
	}
    RX_CONT = 0;                    
}
void timer0_init(void)//用于模拟中断的定时器
{
	Timer0_1T();
	Timer0_AsTimer();
	Timer0_16bitAutoReload();
	Timer0_Load(Timer0_Reload);
	Timer0_InterruptEnable();
	PT0 = 1;	//定时器0高优先级中断
	Timer0_Run();
	EA = 1;					//打开总中断					open global interrupt switch
}	
void Timer2Init(void)		//
{
	Timer2_AsTimer();//使用定时器2
	Timer2_InterruptDisable();
	Timer2_Stop();	
	Timer2_1T();
	T2L = 0x66;		//1ms定时
	T2H = 0x7E;		//1ms定时
	Timer2_InterruptEnable();
	Timer2_Run();
}
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
	//Fake_PrintString1(TX_P31,tx_buffer_vale);//模拟串口稳定后发送
}

#if 0
/****************LED_HC595发送一个字节 ******************/
void LED_DataIn595(u8 dat)
{		
	u8	i;
	for(i=0; i<8; i++)
	{
		dat >>= 1;//先发低位
		B_HC595_SER   = CY;
		B_HC595_SRCLK = 0;
		NOP2();
		B_HC595_SRCLK = 1;
	}
}
/**************** LED_HC595数据锁存******************/
void LED_DataOut595()
{		
	B_HC595_RCLK = 0;
	NOP2();
	B_HC595_RCLK = 1;
}
void HC595_Display(void)//数组已经倒叙，正序扫描即可
{
	u16 i;
	for(i=0;i<LED_StatByteSize;i++){
		LED_DataIn595(LED_StatByte[i]);
	}
	LED_DataOut595();
}
#endif
//========================================================================
// 描述: 硬件初始化 。
//========================================================================
void stc15x_hw_init(void)
{
	P0n_standard(0xff);	//设置为准双向口
	P1n_standard(0xff);	//设置为准双向口
	P2n_standard(0xff);	//设置为准双向口
	//P3n_standard(0xff);	//设置为准双向口
	P4n_standard(0xff);	//设置为准双向口
	P5n_standard(0xff);	//设置为准双向口	

	//InternalRAM_enable();
	timer0_init();
	uart1_config();//定时器1产生波特率串口初始化,用于蓝牙
	UART_INIT();				//UART模块的初始变量
	Timer2Init();//用于1ms定时
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
#define	BaudRate1		9600UL	//?????
#define	Timer1_Reload	(65536UL -(MAIN_Fosc / 4 / BaudRate1))		//Timer 1 ???, ??300KHZ
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
	S1_USE_P36P37();  //UART1 使用P36_TX P37_RX口
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
		Fake_PrintString(TX_P54,"recv success BT05 0x01!\r\n");
	}
	if(RxData[1] == 0x02)
	{
		Fake_PrintString(TX_P54,"recv success BT05 0x02!\r\n");
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
			//Fake_PrintString(TX_P54,"recv success Key_Code to contral switch!\r\n");
		}
	}	
}

void Rcev_DataAnalysis(void)
{
	u8 recv_cmd = 0;
	if(Hand(Rec_Buf,SEND_ID)&&Hand(Rec_Buf,SEND_END))
	{
		Fake_PrintString(TX_P54,SEND_ID);
		Fake_PrintString(TX_P54,":");
		Fake_PrintString(TX_P54,"\r\n");
		recv_cmd = Rec_Buf[ID_LENGTH(SEND_ID)-1];//长度为字符+\0,获取指令
		switch(recv_cmd)
		{
			case CMD_1://蓝牙
					Fake_PrintString(TX_P54,"BT05 CMD!\r\n");	
					Bluetooth_DataProce();
				break;
			case CMD_2://按键
					Fake_PrintString(TX_P54,"Key_Code CMD!\r\n");
					Keyboard_DataProce();
				break;
			default:
				Fake_PrintString(TX_P54,"ERROR CMD!\r\n");
				break;
				CLR_Buf();
		}		
		CLR_Buf();
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
	u8 j,k;
	CLR_Buf();//清除接收缓存
	Timer0_Stop();
	Clear_WS2811();
	Timer0_Run();
	stc15x_hw_init();
	B_HC595_MR = 1;//复位禁止
	B_HC595_OE = 0;//使能芯片
	Switch_ChannelIint();//继电器初始化，全关闭
#if defined(BOOL_WS2811_LED)
	Clear_LedStatByte();//清空存储BOOL型LED状态的buff
#else
	Clear_LedBuff();//清空带灰度的WS2811状态的缓存数组
#endif
	Fake_PrintString(TX_P54,"STC15W408as fake UART init OK!\r\n");	//模拟串口9600，发送一个字符串

	do{
		print1_string(str1);	//握手指令
		DelayMS(500);
		if(++j>5){
			Fake_PrintString(TX_P54,"BT05 hand failed\r\n");
			j = 0;				//Can add error led display
			break;
		}
	}while(!Hand(Rec_Buf,"OK"));			//判断是否握手成功,如果不成功延时一会,再发送AT握手指令
	CLR_Buf(); 
	Fake_PrintString(TX_P54,"BT05 hand OK\r\n");
	
	while (1)
	{
	//	Timer0_Stop();
	//	liushui123x(1,60);
	//	Timer0_Run();
		if(Recv_Ok)
		{
			Timer0_Stop();//保护WS2811发送数据，否则定时器会导致其闪烁
#if defined(BOOL_WS2811_LED)
			WS2811_Display();
			Clear_LedStatByte();
#else
			for(k = 0;k < nWs;k++){
				WS2811_SendByte(LED_Buff[k]);
			}
			Clear_LedBuff();
#endif
			WS2811_Reset();
			Timer0_Run();
			//key_led_reverse();
			Recv_Ok = 0;
			k = 0;
		}
		if (B_Rx_OK)	//接收完的标志位, 收到数据块系统设置1, 用户处理数据后必须清0
		{
			if(RX_CONT > 0)	//确认有数据
			{
			//	for(i=0; i<RX_CONT; i++)	TxSend(TX_P54,Rec_Buf[i]);	//把收到的数据原样返回,用于测试
			}			
			if(Hand(Rec_Buf,"MJX"))
			{
				//Fake_PrintString(TX_P54,"STC15W408as fake UART init OK!\r\n");	//模拟串口9600，发送一个字符串
			}
			Rcev_DataAnalysis();
			RX_CONT  = 0;	//清除字节数
			B_Rx_OK = 0;	//清除接收完成标志
		}

		if(B_1ms)	//1ms计时
		{
			B_1ms = 0;
			if(++msecond >= 1000)	//1s计时
			{	
				msecond = 0;	
				key_led_reverse();
				//Fake_PrintString1(TX_P31,test);//模拟串口稳定后发送
			}
		    if(++cnt50ms >= 50)		//50ms扫描一次行列键盘
			{
				cnt50ms = 0;	
			}
			if(++cnt10ms >= 10)		//10ms扫描是否有数据需要处理
			{
				cnt10ms = 0;
			}
		}
	}
}


//========================================================================
// 函数: void tm0(void) interrupt 1
// 描述: 定时器0中断程序, for UART 以波特率3倍的速度采样判断 开始位.
// 参数: 无.
// 返回: 无.
// 版本: V1.0, 2012-10-22
//========================================================================

void timer0_int(void) interrupt TIMER0_VECTOR
{
//====================== 模拟串口接收程序 ========================================

	if (RxStartFlag)			//已接收到起始位
	{
		if (--RxSample == 0)			//接收数据以定时器的1/3来接收
		{
			RxSample = 3;               //重置接收计数器  接收数据以定时器的1/3来接收	reset send baudrate counter
			if (--RxBitCnt == 0)		//接收完一帧数据
			{
				RxStartFlag = 0;        //停止接收			stop receive
				if((P_RXB_30||P_RXB_36) && !B_Rx_OK)	//确认停止位正确,并且缓冲已空
				{
					Rec_Buf[RX_CONT] = RxShiftReg;     //存储数据到缓冲区	save the data to RBUF
					RX_CONT++;                                   
					if(RX_CONT>Buf_Max)                          //接收数大于定义接收数组最大个数时，覆盖接收数组之前值
					{
						RX_CONT = 0;
					}  
					RxTimeOut = 105;				//超时计数初值, 35个位的时间(对应5个字节), 参考MODBUS协议
				}
			}
			else
			{
				RxShiftReg >>= 1;			  //把接收的单b数据 暂存到 RxShiftReg(接收缓冲)
				if(uart_flag ==1){
					if (P_RXB_30)	RxShiftReg |= 0x80;  //shift RX data to RX buffer
				}
				else if (uart_flag ==2){
					if (P_RXB_36)	RxShiftReg |= 0x80;  //shift RX data to RX buffer
				}
			}
		}
	}
	else if(!P_RXB_36)
	{
		uart_flag = 2;
		RxStartFlag = 1;       //如果是则设置开始接收标志位 	set start receive flag
		RxSample = 4;       //初始化接收波特率计数器       	initial receive baudrate counter
		RxBitCnt = RxBitLenth;       //初始化接收的数据位数(8个数据位+1个停止位)    initial receive bit number (8 data bits + 1 stop bit)
		
	}
	else if (!P_RXB_30)		//判断是不是开始位 P_RXB_30=0;
	{
		RxStartFlag = 1;       //如果是则设置开始接收标志位 	set start receive flag
		RxSample = 4;       //初始化接收波特率计数器       	initial receive baudrate counter
		RxBitCnt = RxBitLenth;       //初始化接收的数据位数(8个数据位+1个停止位)    initial receive bit number (8 data bits + 1 stop bit)
		uart_flag = 1;
	}

	if(RxTimeOut > 0)	//接收超时处理
	{
		if(--RxTimeOut == 0)	B_Rx_OK = 1;	//标志已收到一帧数据
	}
}
#if defined(BOOL_WS2811_LED)
u8 Setbit1_InByte(u8 i,u8 ubyte)
{
	setbit(ubyte,i);
	return ubyte;
}
u8 Setbit0_InByte(u8 i,u8 ubyte)
{
	clrbit(ubyte,i);
	return ubyte;
}
//========================================================================
// 函数: Write_LedStatBuff(u8 val,u16 i)
// 描述: 将中断收到的led 状态转化为字节的bit 表示。
// 参数: val 收到的数值，val，u16 i 第i 个LED.
// 返回: none.
// 版本: VER1.0
// 日期: 2019-10-08
// 备注: 
//========================================================================
void Write_LedStatBuff(u8 val,u16 i)
{
    u16 mu,yu;
    mu = LED_StatByteSize-mod(i,8)-1;//倒叙填入数组
    yu = rem(i,8);//0-7循环
    if(val > 0)
			LED_StatByte[mu] = Setbit1_InByte(yu,LED_StatByte[mu]);
    else if(!val)
			LED_StatByte[mu] = Setbit0_InByte(yu,LED_StatByte[mu]);
	//printf("mu=%d,a = %d,LED_StatByte =%x\n",mu,yu,LED_StatByte[mu]);
}
void Clear_LedStatByte(void)
{
	u16 i;
	for(i=0;i<LED_StatByteSize;i++)
		LED_StatByte[i] = 0;
}
void WS2811_Display(void)
{
	u16 i;
	u8 val,j;
	for(i=0;i<LED_StatByteSize;i++){
		for(j=0;j<8;j++){
			val = LED_StatByte[LED_StatByteSize-i-1]&(0x01<<j);
			if(val)
				val = 0xfe;
			else
				val = 0x00; 
			WS2811_SendByte(val);
		}
	}
}
#else
//带灰度的LED 缓存
void Clear_LedBuff(void)
{	
	u16 i;
	for(i=0;i<nWs;i++)
		LED_Buff[i] = 0;	
}
#endif
#if defined UART1_TIMER1
void uart1_int (void) interrupt UART1_VECTOR
{
	u8 temp;
	Timer0_Stop();

	if(RI)//接收中断
	{
		RI = 0;
		temp = SBUF;
		if(temp==0x05){
			Recv_start = 1;
		}
		else if(temp == 0xff){
			Recv_start = 0;
			//key_led_reverse();
#if defined(BOOL_WS2811_LED)
			//bool_led_count++;
			//for(;bool_led_count<nWs;bool_led_count++)
			//	Write_LedStatBuff(0,bool_led_count);//收到的LED路数小于nWs，自动补全0
			bool_led_count = 0;
#else	
			//while(led_stat_cont<nWs)
			//	LED_Buff[led_stat_cont++] = 0;
			led_stat_cont = 0;
#endif
			Recv_Ok=1;
		}
		else if(Recv_start){ 
#if defined(BOOL_WS2811_LED)	
		Write_LedStatBuff(SBUF,bool_led_count++);//收到的状态转化成bit，填入缓存区，计数加1
#else
			LED_Buff[led_stat_cont] = SBUF ? SBUF : 0;
			led_stat_cont++;			
#endif
		}
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
	Timer0_Run();
}
#endif
/********************* Timer2定时中断************************/
void timer2_int (void) interrupt TIMER2_VECTOR
{
	B_1ms = 1;
}
