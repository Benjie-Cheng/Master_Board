#ifndef __DEBUG_H_
#define __DEBUG_H_

/*********************************************************/

//#define MAIN_Fosc		 5529600L	//定义主时钟	 110 ~ 4800
//#define MAIN_Fosc		11059200L	//定义主时钟	 110 ~ 9600
//#define MAIN_Fosc		12000000L	//定义主时钟	 110 ~ 9600
//#define MAIN_Fosc		22118400L	//定义主时钟	 220 ~ 19200
//#define MAIN_Fosc		24000000L	//定义主时钟	 220 ~ 19200
//#define MAIN_Fosc		33177600L	//定义主时钟	 220 ~ 38400
  #define MAIN_Fosc		35000000L	//定义主时钟	 220 ~ 38400
  
/**********************************************************/

#define Main_Fosc_KHZ	(MAIN_Fosc / 1000)

/***********************************************************/

#include	"STC15Fxxxx.H"
//#include "config.h"
#include<stdio.h>
#include <stdlib.h>
#include <string.h>           // 加入此头文件后,可使用strstr库函数

/***********************************************************/
/***********************common define***********************************/

typedef   bit BOOL;
typedef 	unsigned char	u8;						 /* 无符号8位整型变量                        */
typedef   signed   char int8;          /* 有符号8位整型变量                        */
typedef 	unsigned int	u16;					 /* 无符号16位整型变量                       */
typedef   signed   int  int16;         /* 有符号16位整型变量                       */
typedef 	unsigned long	u32;					 /* 无符号32位长整型变量      		           */
typedef   float         fp32;          /* 单精度浮点数(32位长度)                   */
typedef   double       	fp64;          /* 双精度浮点数(64位长度)                   */
typedef   unsigned char BYTE;
typedef   unsigned int  WORD;

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
//---------------define by project-----------------//
enum Gu8Step{
	TURN_ON_MODE=0,
	TURN_OFF_MODE,
	IDLE_MODE,
};
enum Run_Mode{
	DUAL_MINU_MODE=0,
	ONLY_MINU_MODE,
	DUAL_SEC_MODE,
};
typedef enum {
	KeyNull=0,
	KeySetMode=1,
	KeyRunMode=2,
	KeyOnTime=3,
	KeyOffTime=4,
	KeyCom1,
	KeyCom2,
	KeyCom3,
	KeyCom4,
	KeyCom5,
	KeyCom6,
	KeyBright
}KeyEnum;
typedef enum {
	ON       =0x01,
	OFF      =0x00,
	ON_ALL   =0xff,
	OFF_ALL  =0xfe,
	OFF_ON   =0xfd
}LogicType;
typedef enum {
	COM1=0,
	COM2=1,
	COM3=2,
	ONLY_MIN_LED=3,
	DUAL_MIN_LED=4,
	DUAL_SEC_LED=5,
	RUN_MODE_LED=6,
	OUT1=7,
	COMX=8
}Hc595Bit;
typedef enum {
	ON_TIME_CFG=0,
	OFF_TIME_CFG,
	RUN_MODE_CFG,
	PT_CFG,
	COM_CFG,
	MAX_CFG
/*
data0：通时间
data1：关时间
data2：运行模式
data3:支持PT2272
data4:端口支持数据
*/
}E2promCfg;
typedef struct{
	u8 Step;   //运行步骤,switch 切换
	u8 Mode;   //运行模式 switch 切换，掉电需要记录到EEPROM中	
}SysRunStruct;

typedef struct{
	u8 on;
	u8 off;	
}SetTimestruct;//设置通断的时间结构体，需要写入EEPROM中的时间
typedef struct{
	u8 on;
	u8 off;
	u16 tMode;//用于双秒，双分切换的中间变量	
}RunTimestruct;//运行时间结构体



//------common function------------------------------//
void TxSend(u8 dat);
void Fake_PrintString(u8 *puts);
void key_led_reverse(void);
void key_led_on(BOOL enable);

#endif
