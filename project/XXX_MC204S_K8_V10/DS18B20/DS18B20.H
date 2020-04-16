#ifndef __DS18B20_H
#define __DS18B20_H
#include "config.h"
#include "debug.h" 
//#include <intrinsics.h>
 
 
/**********************************
DS18B20芯片控制
**********************************/
 
#define DS18B20_DQ	P33

u16 Ds_GetTemper(void);温度获取
 
#endif