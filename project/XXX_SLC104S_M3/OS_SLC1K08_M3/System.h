#ifndef __DEBUG_H_
#define __DEBUG_H_

/*********************************************************/

//#define MAIN_Fosc		 5529600L	//定义主时钟	 110 ~ 4800
//#define MAIN_Fosc		11059200L	//定义主时钟	 110 ~ 9600
//#define MAIN_Fosc		12000000L	//定义主时钟	 110 ~ 9600
//#define MAIN_Fosc		22118400L	//定义主时钟	 220 ~ 19200
#define MAIN_Fosc		24000000L	//定义主时钟	 220 ~ 19200
//#define MAIN_Fosc		33177600L	//定义主时钟	 220 ~ 38400
//  #define MAIN_Fosc		35000000L	//定义主时钟	 220 ~ 38400
  
/**********************************************************/

#define Main_Fosc_KHZ	(MAIN_Fosc / 1000)
#define	Baudrate1	115200UL                                //通信波特率115200
//#define	Baudrate1	9600UL
/***********************************************************/

#include	"STC15Fxxxx.H"
#include <rtx51tny.h>                 /* RTX-51 tiny functions & defines      */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>           // 加入此头文件后,可使用strstr库函数

/***********************************************************/
/***********************common define***********************************/

typedef		bit BOOL;
typedef 	unsigned char	u8;						 /* 无符号8位整型变量                        */
typedef   signed   char int8;          /* 有符号8位整型变量                        */
typedef 	unsigned int	u16;					 /* 无符号16位整型变量                       */
typedef   signed   int  int16;         /* 有符号16位整型变量                       */
typedef 	unsigned long	u32;					 /* 无符号32位长整型变量      		           */
typedef   float         fp32;          /* 单精度浮点数(32位长度)                   */
typedef   double       	fp64;          /* 双精度浮点数(64位长度)                   */
typedef   unsigned char BYTE;
typedef   unsigned int  WORD;

	#define UP  				1
	#define DWON  		  0
/*
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
*/
#define mod(a,b)	a/b                 //求模
#define rem(a,b)	a%b                 //求余
#define odd(x)      rem(x,2)          //奇数判断
#define setbit(x,y) x|=(1<<y)         //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y)        //将X的第Y位清0
#define reversebit(x,y)  x^=(1<<y)    //将x的第Y位取反
#define getbit(x,y)   ((x) >> (y)&1)  //得到x的第Y位
#define MAX              16           //随机数最大值
#define MIN              1            //随机数最小值
#define creat_random() (rand() % (MAX + 1 - MIN)+ MIN)      //产生随机数，用于数据加密
#define is_max(left, right, value) value>right ? left : value
#define is_min(left, right, value) value<left ? right : value

//---------------define by project-----------------//
#define INIT_0 0
#define KEY_SCAN_1 1
//#define KEY_DONE_2 2
#define LIGHTS_2 2
#define DISPLAY 3

#define LOG_LEVEL 1
#define UART_SUPPORT 0    //支持串口需要定义
#define LOCK_SUPPORT 0    //支持锁系统需要定义
#define ID_SUPPORT 0      //支持全球ID需要定义
typedef enum {
	INFO =1,
	DEBUG,
	ERR
}LogLevel;
enum {
	SPEED_MIN  =1,
	SPEED_MAX  =20,
	SPEED_STEP =2,
	LED_NUM_MIN  = 1,
	LED_NUM_MAX  = 255,
	LED_NUM_STEP = 1,
	LED_TYPE_MIN  = 0,
	LED_TYPE_MAX  = 10,
	LED_TYPE_STEP = 1,
};

#define MAX_BL 150
#define MIN_BL 2
#define BL_SET 1
#define BL_MIN(n) n ? MIN_BL:0
typedef enum {
	SET_LINE = 0,
	Mode1,
	Mode2,
	Mode3,
	Mode4,
	Mode5,
	Mode6,
	Mode7,
	Mode8,
	Mode9
}LightType;
//typedef enum {
//	INIT_0,                          /* task number of task:  init           */
//	KEY_SCAN_1,						   /* task number of task:  keyscan        */
//	KEY_DONE_2,                        /* task number of task:  keydone        */
//	LIGHTS_3,                          /* task number of task:  lights         */
//	DISPLAY,
//};//SysTask;

typedef enum {
	KeyNull=0,
	KeySpeed=1,
	KeyRunMode,
	KeyLedNum
}KeyEnum;

typedef enum {
	ON       =0x01,
	OFF      =0x00,
	ON_ALL   =0xff,
	OFF_ALL  =0xfe,
	OFF_ON   =0xfd,
	TEST_REV = 0x11,
	LOCK_REV = 0x66
	
}LogicType;
typedef enum {
	LED_NUM_CFG=0,
	RUN_TYPE_CFG,
	SPEED_CFG,
	REV_CFG,
	MAX_CFG
/*
data0：路数
data1：模式
data2：速度
data3:
data4:
*/
}E2promCfg;
enum Run_Mode{
	SET_MODE=0,
	RUN_MODE,
	LOCK_MODE
};
typedef struct{
	u8 Step;   //运行步骤,switch 切换
	u8 Mode;   //运行和设置两种状态
	u8 LedMode;//花样模式
	u8 LedNum;
	u8 LedSpeed;
	u8 State;  //状态
}SysRunStruct;

extern SysRunStruct;
//SysRunStruct SysRun;
//-------------stc15w104---------------
/*
#define SPEED_GPIO P34
#define MODE_GPIO  P30
#define NLED_GPIO  P33
*/
//--------stc8f1k08
#define SPEED_GPIO P54
#define MODE_GPIO  P30
#define NLED_GPIO  P33
//------common function------------------------------//
void TxSend(u8 dat);
void Fake_PrintString(u8 *puts);
void key_led_reverse(void);
void key_led_on(BOOL enable);
void uDelayMs(u16 t);		//@35MHz

#endif
