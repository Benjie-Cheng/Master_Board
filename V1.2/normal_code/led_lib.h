#ifndef __LED_LIB_H_
#define __LED_LIB_H_


#include "config.h"
#include "debug.h"
#define nWs 200

extern void DelayMS(u16 t);
void WS2811_SendByte(u8 dat);		//@��λ�ȴ�
void WS2811_Reset(void);        //@��λоƬ
void Clear_WS2811(void);        //@��0����

void liushui123x(BOOL RL,u16 led_num);// ˳����·������ˮ

#endif