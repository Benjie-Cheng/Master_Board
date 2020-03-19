#ifndef __DEBUG_H_
#define __DEBUG_H_
#include "config.h"
#include<stdio.h>
#include <stdlib.h>
#include <string.h>           // 加入此头文件后,可使用strstr库函数
#define mod(a,b)	a/b  //求模
#define rem(a,b)	a%b  //求余
#define setbit(x,y) x|=(1<<y)         //将X的第Y位置1
#define clrbit(x,y) x&=~(1<<y)        //将X的第Y位清0
#define reversebit(x,y)  x^=(1<<y)    //将x的第Y位取反
#define getbit(x,y)   ((x) >> (y)&1)
#endif
