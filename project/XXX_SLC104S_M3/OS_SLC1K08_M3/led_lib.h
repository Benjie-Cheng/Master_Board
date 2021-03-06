#ifndef __LED_LIB_H_
#define __LED_LIB_H_
#include "system.h"
//-------------stc15w104---------------
/*
#define WS2811_SDA_GPIO	 P35
#define WS2811_SDA1_GPIO P35
*/
//--------stc8f1k08
#define WS2811_SDA_GPIO	 P55
#define WS2811_SDA1_GPIO P55

//#define HZ_35000000 1
#define HZ_12000000 1

#define BLUE  1
#define RED   2
#define GREEN 3
#define WHITE 0
extern SysRunStruct SysRun;
//extern u8  nWs;
//extern void DelayMS(u16 t);

void WS2811_SendByte(u8 dat);		//@高位先传
void WS2811_Reset(void);        //@复位芯片
void Clear_WS2811(void);        //@清0函数

void liushui123x(BOOL RL,u16 led_num);// 顺序三路单灯流水
void PAOMAADD(u8 TYPE,u16 GG1,u16 GG2);//单色单灯叠加亮，底色单灯叠加灭
void PAOMADEC(u8 GG1,u8 GG2);//
void LIUXINGYU32(bit bG);//32灯拖尾变暗流水
void LIUXINGYU16(u8 GG1,bit bG);//16灯拖尾变暗流水
void LIUXINGYU168(bit bG);//顺序每32路间隔16路拖尾
void LIUXINGYU8(u8 GG1,bit bG);//顺序8路RGB拖尾+RGB补色
void LIUXINGYU88(bit bG,bit bG1);//待调试
void quancaipiao(void);//全彩飘
void qicaituowei(void);//七彩拖尾
void DDPADD4(void);//待调试
void DDPADD5(void);//待调试
void PAOMA(u8 GG1,u8 GG2);//
//void ChangeHigh(bit GG1,bit RR1,bit BB1); //纯色渐亮
//void ChangeLose(bit GG1,bit RR1,bit BB1);	//纯色渐变暗
void RgbChange(void);//七彩渐变色
void led_test();
void TurnOn(u8 nLed,bit BL); //常亮数量
void TurnOff(u8 nLed,bit BL);	//常灭
void ChangeHigh(u8 nLed,bit BL); //纯色渐亮
void ChangeLose(u8 nLed,bit BL);	//纯色渐暗
void func1(u8 num1,u8 num2,u8 nLed,bit up);
void BreathingAdd_Two(u8 nLed);
void BreathingDel_Two(u8 nLed);
void liushui(u8 nLed,bit BL);
void liushui123(u8 nLed,bit BL);

//------------此区域为快速区域-----------------------------//
void liushui_fast(u8 nLed,bit BL,u8 nRun);//单个快速流水
#endif