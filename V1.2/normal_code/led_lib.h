#ifndef __LED_LIB_H_
#define __LED_LIB_H_


#include "config.h"
#include "debug.h"
#define nWs 200

extern void DelayMS(u16 t);
void WS2811_SendByte(u8 dat);		//@高位先传
void WS2811_Reset(void);        //@复位芯片
void Clear_WS2811(void);        //@清0函数

void liushui123x(BOOL RL,u16 led_num);// 顺序三路单灯流水

#endif