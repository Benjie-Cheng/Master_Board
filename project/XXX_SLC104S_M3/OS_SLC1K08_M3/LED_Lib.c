#include "led_lib.h"
//#include <rtx51tny.h>                 /* RTX-51 tiny functions & defines      */

#define WS2811_SDA_GPIO_H() {WS2811_SDA_GPIO=1;WS2811_SDA1_GPIO = 1;}
#define WS2811_SDA_GPIO_L() {WS2811_SDA_GPIO=0;WS2811_SDA1_GPIO = 0;}
#define BLUE_LIGHT(BL) {WS2811_SendByte(BL);WS2811_SendByte(0);WS2811_SendByte(0);}
#define RED_LIGHT(BL) {WS2811_SendByte(0);WS2811_SendByte(BL);WS2811_SendByte(0);}
#define GREEN_LIGHT(BL) {WS2811_SendByte(0);WS2811_SendByte(0);WS2811_SendByte(BL);}
#define WHITE_LIGHT(BL) {WS2811_SendByte(BL);WS2811_SendByte(BL);WS2811_SendByte(BL);}
#define LED_SET(TYPE,BL) {\
	if(BLUE==TYPE){BLUE_LIGHT(BL);}\
	else if(RED==TYPE){RED_LIGHT(BL);}\
	else if(GREEN==TYPE){GREEN_LIGHT(BL);}\
	else if(WHITE==TYPE){WHITE_LIGHT(BL);}}

//extern   SysRun.LedNum = 1;
/*
#ifdef HZ_11059200
void Delay1us()		//@11.0592MHz
{
	_nop_();
	_nop_();
	_nop_();
}
#elif HZ_22118400
void Delay1us()		//@22.1184MHz
{
	unsigned char i;

	i = 3;
	while (--i);
}
#elif HZ_33177600
void Delay1us()		//@33.1776MHz
{
	unsigned char i;

	_nop_();
	_nop_();
	_nop_();
	i = 5;
	while (--i);
}
#elif HZ_35000000
void Delay1us()		//@35MHz
{
	unsigned char i;

	_nop_();

	i = 1;
	//i = 6;//200���ƻ�����
	while (--i);
}
#endif
*/
extern u8 count;
void Delay1us()		//@35MHz
{
	unsigned char i;

	_nop_();

	i = 1;
	//i = 6;//200?????
	while (--i);
}
void WS2811_Send_H(void)		//@���͸ߵ�ƽ��1��
{
	WS2811_SDA_GPIO_H(); 
	Delay1us();
  WS2811_SDA_GPIO_L();
  //Delay1us();
}
void WS2811_Send_L(void)		//@���͵͵�ƽ��0��
{
	
	WS2811_SDA_GPIO_H(); 
	//Delay1us();
	WS2811_SDA_GPIO_L();
	Delay1us();
	//Delay1us();
	/*********************
	��������������ʱ��led�Ƴ���100������˸
	*********************/
	//Delay1us();
	//Delay1us();
}


void WS2811_Reset(void)  //@��λоƬ
{
	u8 i ;
	//WS2811_SDA_GPIO_H();
	WS2811_SDA_GPIO_L();
	for(i=0;i<60;i++)
		Delay1us();
}
//========================================================================
// ����: WS2811����һ�ֽ�����@��λ�ȴ�
//========================================================================
void WS2811_SendByte(u8 dat)		//@��λ�ȴ�
{
    u8 i;
	EA =1;
    for(i=0;i<8;i++)
    {
        if(dat & 0x80) //��������1
        {
					WS2811_Send_H();
        }
        else           //��������0
        {
						WS2811_Send_L();
        }
        dat <<= 1;
    }
	EA =1;
}
void WS2811_Send24bit(u32 dat)		//@��λ�ȴ�
{
    u8 i;
    for(i=0;i<24;i++)
    {
        if(dat & 0x800000) //��������1
        {
					WS2811_Send_H();
        }
        else           //��������0
        {
						WS2811_Send_L();
        }
        dat <<= 1;
    }
}

void Clear_WS2811(void)
{
	u16 i=0;
	
	//for(i=0;i<(SysRun.LedNum+300);i++){
	for(i=0;i<300;i++){
		WS2811_SendByte(0);
		//Delay1us();
		//Delay1us();
		Delay1us();
	}
	WS2811_Reset();
}
/*
void ChangeHigh(bit GG1,bit RR1,bit BB1) //��ɫ����
{  
	u8 n,i,num;//RGB_count=RGB_c;
	for(i=0;i<250;i++)//brightness
	{
		for(n=0;n<SysRun.LedNum;n++)//RGB_count
		{
			WS2811_SendByte(num=GG1?i:0);
			WS2811_SendByte(num=RR1?i:0);
			WS2811_SendByte(num=BB1?i:0);
		}
		WS2811_Reset();
		if(i<50)
			os_wait2(K_TMO, 2);
		else
			os_wait2(K_TMO, 1);
	}
}

void ChangeLose(bit GG1,bit RR1,bit BB1)	//��ɫ����
{  
	u8 n,i,num;//,RGB_count=RGB_c;
	for(i=250;i>0;i--)
	{
		for(n=SysRun.LedNum;n>0;n--)
		{
			WS2811_SendByte(num=GG1?i:0);
	    WS2811_SendByte(num=RR1?i:0);
	    WS2811_SendByte(num=BB1?i:0);
		}
			WS2811_Reset();
		//if(i<50)
			os_wait2(K_TMO, 2);
		//else
		//	os_wait2(K_TMO, 2);
	}	
}
*/
void RgbChange(void)
{
//		ChangeHigh(1,0,0);
//		ChangeLose(1,0,0);
//		ChangeHigh(0,1,0);
//		ChangeLose(0,1,0);
//		ChangeHigh(0,0,1);
//		ChangeLose(0,0,1);
//		ChangeHigh(1,1,1);			
//		ChangeLose(1,1,1);
//		ChangeHigh(1,1,0);			
//		ChangeLose(1,1,0);
//		ChangeHigh(1,0,1);			
//		ChangeLose(1,0,1);
//		ChangeHigh(0,1,1);			
//		ChangeLose(0,1,1);
}
void led_test()
{

	WS2811_SendByte(0);
	WS2811_SendByte(0);
	WS2811_SendByte(0);
	//WS2811_Send_L();
	//LED_SET(BLUE,20);
	//LED_SET(RED,80);
	//LED_SET(GREEN,120);
	//LED_SET(WHITE,250);
	WS2811_Reset();
	os_wait2(K_TMO, 2);
}

void OS_Delay(u8 t,u8 n)
{
	u16 i,j;
	for(i=0;i<t;i++)//RGB_count
	{
		for(j=0;j<n;j++)
			os_wait2(K_TMO, 1);
	}
}
/**********************************************
*���ܣ����� 
**********************************************/
void TurnOn(u8 nLed,bit BL) //��������
{  
	u8 n,ModLed,RemLed;//RGB_count=RGB_c;
	
	ModLed = mod(nLed,3);//ws2811����
	RemLed = rem(nLed,3);//���һ��ws2811�ĵ�·��
	//Clear_WS2811();
	if(ModLed == 0)//С��3·
	{
		WS2811_SendByte(mod((nLed+2),3)? MAX_BL:BL_MIN(BL));
		WS2811_SendByte(mod((nLed+1),3)? MAX_BL:BL_MIN(BL));
		WS2811_SendByte(mod((nLed+0),3)? MAX_BL:BL_MIN(BL));
	}
	else
	{
		for(n=0;n<nLed;n++)//RGB_count
			WS2811_SendByte(MAX_BL);
		for(n=0;n<3-RemLed;n++)//RGB_count
			WS2811_SendByte(0);
	}		
	//WS2811_Reset();
	os_wait2(K_TMO, 10);
}
/**********************************************
*���ܣ����� 
**********************************************/
void TurnOff(u8 nLed,bit BL)	//����
{  
	u8 n,RemLed;//RGB_count=RGB_c;
	
	//ModLed = mod(nLed,3);//ws2811����
	RemLed = rem(nLed,3);//���һ��ws2811�ĵ�·��
	for(n=0;n<nLed;n++)//RGB_count
		WS2811_SendByte(BL ? BL_MIN(BL):0);
	for(n=0;n<3-RemLed;n++)//RGB_count
		WS2811_SendByte(0);
	WS2811_Reset();
}
/**********************************************
*���ܣ���ɫ���� 
**********************************************/
void ChangeHigh(u8 nLed,bit BL) //��ɫ����
{  
	u8 n,i,ModLed,RemLed;//RGB_count=RGB_c;
	
	ModLed = mod(nLed,3);//ws2811����
	RemLed = rem(nLed,3);//���һ��ws2811�ĵ�·��
	for(i=BL_MIN(BL);i<=MAX_BL;i++)//brightness
	{
		if(ModLed == 0)//С��3·
		{
			WS2811_SendByte(mod((nLed+2),3)? i:BL_MIN(BL));
			WS2811_SendByte(mod((nLed+1),3)? i:BL_MIN(BL));
			WS2811_SendByte(mod((nLed+0),3)? i:BL_MIN(BL));
		}
		else
		{
			for(n=0;n<nLed;n++)//RGB_count
				WS2811_SendByte(i);
			for(n=0;n<3-RemLed;n++)//RGB_count
				WS2811_SendByte(0);
		}
		//WS2811_Reset();
		if(i<50)
			OS_Delay(SysRun.LedSpeed,8);
		else
			OS_Delay(SysRun.LedSpeed,2);
	}
}
/**********************************************
*���ܣ���ɫ����
**********************************************/
void ChangeLose(u8 nLed,bit BL)	//��ɫ����
{  
	u8 n,i,ModLed,RemLed;//RGB_count=RGB_c;
	RemLed = rem(nLed,3);//���һ��ws2811�ĵ�·��
	for(i=MAX_BL;i>(BL_MIN(BL));i--)
	{
		if(ModLed == 0)//С��3·
		{
			WS2811_SendByte(mod((nLed+2),3)? i:BL_MIN(BL));
			WS2811_SendByte(mod((nLed+1),3)? i:BL_MIN(BL));
			WS2811_SendByte(mod((nLed+0),3)? i:BL_MIN(BL));
		}
		else
		{
			for(n=0;n<nLed;n++)//RGB_count
				WS2811_SendByte(i);
			for(n=0;n<3-RemLed;n++)//RGB_count
				WS2811_SendByte(0);
		}
		//WS2811_Reset();
		if(i<50)
			OS_Delay(SysRun.LedSpeed,5);
		else
			OS_Delay(SysRun.LedSpeed,2);
	}	
}

/**********************************************
*���ܣ�����n1+n2���� n1+n2=n
**********************************************/
void func1(u8 num1,u8 num2,u8 nLed,bit up)
{

	//��ʼǰȫ��
	u8 i,t,RemLed;//RGB_count=RGB_c;
	RemLed = rem(nLed,3);//���һ��ws2811�ĵ�·��
	if(up){
		if(nLed<=3)
		{
			for(i=BL_MIN(BL_SET);i<MAX_BL;i++)//brightness
			{
				for(t=0;t<num1;t++)
					WS2811_SendByte(MAX_BL);
				for(t=0;t<num2;t++)
					WS2811_SendByte(i);
/*****-----------------------------------**********/
				WS2811_SendByte(BL_MIN(BL_SET));
/*****-----------------------------------**********/
				for(t=0;t<3-RemLed-1;t++)
					WS2811_SendByte(0);
				WS2811_Reset();
				os_wait2(K_TMO, SysRun.LedSpeed);
			}
		}
		else
		{
			//ModLed = mod(nLed-num1,3);//ws2811����
			RemLed = rem(nLed,3);//���һ��ws2811�ĵ�·��
			for(i=BL_MIN(BL_SET);i<MAX_BL;i++)//brightness
			{
				for(t=0;t<num1;t++)
					WS2811_SendByte(MAX_BL);
				for(t=0;t<num2;t++)
					WS2811_SendByte(i);
/*****-----------------------------------**********/
				WS2811_SendByte(BL_MIN(BL_SET));
/*****-----------------------------------**********/
				//for(t=0;t<(nLed-num1-num2);t++)
				//	WS2811_SendByte(BL_MIN(BL_SET));
				for(t=0;t<3-RemLed-1;t++)
					WS2811_SendByte(0);
				WS2811_Reset();
				os_wait2(K_TMO, SysRun.LedSpeed);
			}		
		}
	}
	else
	{
		if(nLed<=3)
		{
			for(i=MAX_BL;i>BL_MIN(BL_SET);i--)//brightness
			{
				for(t=0;t<num1;t++)
					WS2811_SendByte(MAX_BL);
				for(t=0;t<num2;t++)
					WS2811_SendByte(i);
/*****-----------------------------------**********/
				WS2811_SendByte(BL_MIN(BL_SET));
/*****-----------------------------------**********/
				for(t=0;t<3-RemLed-1;t++)
					WS2811_SendByte(0);
				WS2811_Reset();
				os_wait2(K_TMO, SysRun.LedSpeed);
			}
		}
		else
		{
			//ModLed = mod(nLed-num1,3);//ws2811����
			RemLed = rem(nLed,3);//���һ��ws2811�ĵ�·��
			for(i=MAX_BL;i>BL_MIN(BL_SET);i--)//brightness
			{
				for(t=0;t<num1;t++)
					WS2811_SendByte(MAX_BL);
				for(t=0;t<num2;t++)
					WS2811_SendByte(i);
/*****-----------------------------------**********/
				WS2811_SendByte(BL_MIN(BL_SET));
/*****-----------------------------------**********/
				for(t=0;t<3-RemLed-1;t++)
					WS2811_SendByte(0);
				WS2811_Reset();
				os_wait2(K_TMO, SysRun.LedSpeed);
			}		
		}
	}		
}
/**********************************************
*���ܣ����������㶨�� 0->n
**********************************************/
void BreathingAdd_Two(u8 nLed)
{
	u8 n;
	//��ʼǰȫ��
	TurnOff(nLed,0);
	for(n=0;n<nLed;n++)
	{
		func1(n,1,n,UP);
		//os_wait2(K_TMO, 1);
	}
//	os_wait2(K_TMO, SysRun.LedSpeed);
}
/**********************************************
*���ܣ������𵽺��� n->0
**********************************************/
void BreathingDel_Two(u8 nLed)
{
	u8 n;
	//��ʼǰȫ��
	//TurnOff(nLed);
	for(n=0;n<nLed;n++)
	{
		func1(nLed-n-1,1,n,DWON);
		//os_wait2(K_TMO, 1);
	}
	//TurnOff(nLed);
	//func1(0,1,nLed,DWON);
	//os_wait2(K_TMO, SysRun.LedSpeed);
}

void liushui_fast(u8 nLed,bit BL,u8 nRun)
{
	u8 i,n;
	TurnOff(nLed,BL);//����
	for(n=0;n<=nLed;n++)
	{

		for(i=0;i<n-nRun;i++)
		{
			WS2811_SendByte(BL_MIN(BL));
		}
		for(i=0;i<nRun;i++)
		{
			WS2811_SendByte(MAX_BL);//����
		}
		//uDelayus(50);
		OS_Delay(SysRun.LedSpeed,5);
	}

}
/**********************************************
*���ܣ�n������
**********************************************/
void breath_n(u8 n,bit BL,u8 mode)
{
	u8 i,j;
	if(mode==3){
			for(i=0;i<n;i++)
			{
				WS2811_SendByte(BL_MIN(BL));
			}
			WS2811_SendByte(MAX_BL);//����
			//uDelayus(50);
			OS_Delay(SysRun.LedSpeed,4);
			return;
		}
	for(j=BL_MIN(BL);j<=MAX_BL;j++)//brightness
		{
			for(i=0;i<n;i++)
			{
				WS2811_SendByte(BL_MIN(BL));
			}
			if(mode==UP)//������
				WS2811_SendByte(j);
			else if(mode==DWON)//������
				WS2811_SendByte(MAX_BL+(BL_MIN(BL))-j);
			else
				WS2811_SendByte(MAX_BL);//����
			//WS2811_Reset();
			//uDelayMs(1);
			uDelayus(50);
			//OS_Delay(SysRun.LedSpeed,1);
		}
			//WS2811_Reset();
			OS_Delay(SysRun.LedSpeed,1);
}
/**********************************************
*���ܣ�һ·������ˮ,���������
**********************************************/
void liushui123(u8 nLed,bit BL)
{
	u8 n;
	TurnOff(nLed,BL);//����
	for(n=0;n<nLed;n++)
	{
		breath_n(n,BL,UP);
		//breath_n(n,BL,3);
		breath_n(n,BL,DWON);
	}
}
/**********************************************
*���ܣ���������
**********************************************/
void liushui(u8 nLed,bit BL)
{
	u8 n;
	TurnOff(nLed,BL);//����
	for(n=0;n<nLed;n++)
	{
		breath_n(n,BL,3);
	}
}