#ifndef __STC15Wxx_CONF_H
#define __STC15Wxx_CONF_H

/*********************************************************/

//#define MAIN_Fosc		 5529600L	//定义主时钟	 110 ~ 4800
//#define MAIN_Fosc		11059200L	//定义主时钟	 110 ~ 9600
//#define MAIN_Fosc		12000000L	//定义主时钟	 110 ~ 9600
//#define MAIN_Fosc		22118400L	//定义主时钟	 220 ~ 19200
//#define MAIN_Fosc		24000000L	//定义主时钟	 220 ~ 19200
#define MAIN_Fosc		33177600L	//定义主时钟	 220 ~ 38400



/*********************************************************/

  #include	"STC15Fxxxx.H"
  #include<intrins.h>
  #include"string.h"
  #include<stdio.h>
  #include<stdlib.h>


/**************************************************************************/

#define Main_Fosc_KHZ	(MAIN_Fosc / 1000)
#define BaudRate		9600		//模拟串口波特率
#define Timer0_Reload		(65536 - MAIN_Fosc / BaudRate / 3)

/***********************************************************/
/**********************************************************/
typedef     bit BOOL;
typedef 	unsigned char	u8;						 /* 无符号8位整型变量                        */
typedef     signed   char  	int8;                    /* 有符号8位整型变量                        */
typedef 	unsigned int	u16;					 /* 无符号16位整型变量                       */
typedef     signed   int   	int16;                   /* 有符号16位整型变量                       */
typedef 	unsigned long	u32;					 /* 无符号32位长整型变量      		         */
typedef     signed   int   	int16;                   /* 有符号16位整型变量                       */
typedef     float         	fp32;                    /* 单精度浮点数(32位长度)                   */
typedef     double       	fp64;                    /* 双精度浮点数(64位长度)                   */
/*----------------------------------------#define-----------------------------------*/
	#ifndef TRUE
	#define TRUE  				1
	#endif
	
	#ifndef FALSE
	#define FALSE 				0
	#endif
	
	#ifndef NULL
	#define NULL  				0
	#endif
	
	#ifndef uchar 
	#define uchar unsigned char 
	#endif
	#ifndef uint 
	#define uint unsigned int 
	#endif
	
	#define NO                   0
	#define YES                  1
	
	
	#define LOW                  0
	#define HIGH                 1
	
	#define ON                   1
	#define OFF                  0
	#define ENABLE               1
	#define DISENABLE			0
	#define MASTER				1
	#define SLAVE				0
	#define ONEWAY				0
	#define TWOWAY				1
	
	#define mod(a,b)	a/b          	  //求模
	#define rem(a,b)	a%b  			  //求余
	#define odd(x)      rem(x,2)  		  //奇数判断
	#define setbit(x,y) x|=(1<<y)         //将X的第Y位置1
	#define clrbit(x,y) x&=~(1<<y)        //将X的第Y位清0
	#define reversebit(x,y)  x^=(1<<y)    //将x的第Y位取反
	#define getbit(x,y)  x&=(1<<y)    	  //得到x的第Y位
	#define MAX              16           //随机数最大值
	#define MIN              1            //随机数最小值
	#define creat_random() (rand() % (MAX + 1 - MIN)+ MIN)      //产生随机数，用于数据加密

/**********************************************************/


/*-------------------------------------Peripheral----------------------------------------*/ 
	#include"stc15wxx_debug.h"
	#include"stc15wxx_msg.h"
	#include"stc15wxx_sys.h"
	#include"Peripheral.h"
	#include"LED_Lib.h"
	
/*--------------------系统调试----------------*/
//#define DEBUG_Vampireyifeng
	enum {
		TX_P31 = 0x01,//
		TX_DENUG,//模拟串口
	};
	sbit	TX_DENUG_GPIO = P5^5;	//定义模拟串口发送端,可以是任意IO
	#define LED_DEBUG_GPIO P54      //定义调试LED
	sbit	P_TXD = P5^5;           //定义模拟串口发送端,可以是任意IO
/*--------------------------------------------*/

/*----------------------------------- GPIO定义 -----------------------------------*/
// 描述: 驱动共阳极数码管和继电器595*2
//输出低-导通数码管（LED亮）
sbit	A_HC595_SER   = P1^1;	//pin 04	SER		data input
sbit	A_HC595_RCLK  = P3^2;	//pin 01	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P1^0;	//pin 03	SRCLK	Shift data clock

// 描述: 四个按键定义
sbit	Speed_Key = P1^2;
sbit	Mode_Key  = P1^3;
sbit	Del_Key   = P1^4;
sbit	OK_key    = P1^5;

// 描述: WS2811
sbit	WS2811_A = P3^0;	
sbit	WS2811_B = P3^1;
#define WS2811_SDA_GPIO	WS2811_A

#endif