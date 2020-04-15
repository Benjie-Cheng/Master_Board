#ifndef __LED_LIB_H_
#define __LED_LIB_H_
#include "system.h"

#define WS2811_SDA_GPIO	 P35
#define WS2811_SDA1_GPIO P35
#define HZ_35000000 1

#define BLUE  1
#define RED   2
#define GREEN 3
#define WHITE 0
extern SysRunStruct SysRun;
//extern u8  nWs;
//extern void DelayMS(u16 t);

void WS2811_SendByte(u8 dat);		//@��λ�ȴ�
void WS2811_Reset(void);        //@��λоƬ
void Clear_WS2811(void);        //@��0����

void liushui123x(BOOL RL,u16 led_num);// ˳����·������ˮ
void PAOMAADD(u8 TYPE,u16 GG1,u16 GG2);//��ɫ���Ƶ���������ɫ���Ƶ�����
void PAOMADEC(u8 GG1,u8 GG2);//
void LIUXINGYU32(bit bG);//32����β�䰵��ˮ
void LIUXINGYU16(u8 GG1,bit bG);//16����β�䰵��ˮ
void LIUXINGYU168(bit bG);//˳��ÿ32·���16·��β
void LIUXINGYU8(u8 GG1,bit bG);//˳��8·RGB��β+RGB��ɫ
void LIUXINGYU88(bit bG,bit bG1);//������
void quancaipiao(void);//ȫ��Ʈ
void qicaituowei(void);//�߲���β
void DDPADD4(void);//������
void DDPADD5(void);//������
void PAOMA(u8 GG1,u8 GG2);//
//void ChangeHigh(bit GG1,bit RR1,bit BB1); //��ɫ����
//void ChangeLose(bit GG1,bit RR1,bit BB1);	//��ɫ���䰵
void RgbChange(void);//�߲ʽ���ɫ
void led_test();
void TurnOn(u8 nLed,bit BL); //��������
void TurnOff(u8 nLed,bit BL);	//����
void ChangeHigh(u8 nLed,bit BL); //��ɫ����
void ChangeLose(u8 nLed,bit BL);	//��ɫ����
void func1(u8 num1,u8 num2,u8 nLed,bit up);
void BreathingAdd_Two(u8 nLed);
void BreathingDel_Two(u8 nLed);
//void liushui(u8 nLed,bit BL);
//void liushui123(u8 nLed,bit BL);
void BLSet(u8 nLed,bit BL,u8 brightness);
#endif