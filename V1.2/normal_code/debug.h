#ifndef __DEBUG_H_
#define __DEBUG_H_
#include "config.h"
#include<stdio.h>
#include <stdlib.h>
#include <string.h>           // 加入此头文件后,可使用strstr库函数
#define mod(a,b)	a/b  //求模
#define rem(a,b)	a%b  //求余
#define odd(x)      rem(x,2)  //奇数判断
#define setbit(x,y) x|=(1<<y)         //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y)        //将X的第Y位清0
#define reversebit(x,y)  x^=(1<<y)    //将x的第Y位取反
#define getbit(x,y)  x&=(1<<y)    	  //得到x的第Y位
#define MAX              16             //随机数最大值
#define MIN              1              //随机数最小值
#define creat_random() (rand() % (MAX + 1 - MIN)+ MIN)      //产生随机数，用于数据加密
enum {
	TX_P31 = 0x01,//
	TX_P54,//Key_board

};
sbit	P_TXD = P3^1;	//定义模拟串口发送端,可以是任意IO
sbit	P_TXD_P54 = P3^6;	//定义模拟串口发送端,可以是任意IO
void TxSend(u8 soft_uart,u8 dat);
void Fake_PrintString(u8 soft_uart,u8 *puts);
void Fake_PrintString1(u8 soft_uart,u8 *puts);
void key_led_reverse(void);
void key_led_on(BOOL enable);

#endif
