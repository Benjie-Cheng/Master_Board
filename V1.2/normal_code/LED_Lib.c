#include "led_lib.h"
#define WS2811_SDA_GPIO	P05
#define HZ_33177600 1

#ifdef HZ_11059200
void Delay1us()		//@11.0592MHz
{
	_nop_();
	_nop_();
	_nop_();
}
/*
#elif HZ_22118400
void Delay1us()		//@22.1184MHz
{
	unsigned char i;

	i = 3;
	while (--i);
}
*/
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

#endif

void WS2811_Send_H(void)		//@发送高电平“1”
{
	WS2811_SDA_GPIO=1; 
	Delay1us();
  WS2811_SDA_GPIO=0;
  Delay1us();

}
void WS2811_Send_L(void)		//@发送低电平“0”
{
	WS2811_SDA_GPIO=1; 
	//Delay1us();
	WS2811_SDA_GPIO=0;
	Delay1us();
	//Delay1us();
	//Delay1us();
	//Delay1us();
}

void Delay60us()		//@11.0592MHz
{
	unsigned char i, j;

	i = 1;
	j = 162;
	do
	{
		while (--j);
	} while (--i);
}
void WS2811_Reset(void)  //@复位芯片
{
	u8 i ;
	//WS2811_SDA_GPIO=1;
	WS2811_SDA_GPIO=0;
	for(i=0;i<60;i++)
		Delay1us();
}
//========================================================================
// 描述: WS2811发送一字节数据@高位先传
//========================================================================
void WS2811_SendByte(u8 dat)		//@高位先传
{
    u8 i;
    for(i=0;i<8;i++)
    {
        if(dat & 0x80) //发送数据1
        {
					WS2811_Send_H();
        }
        else           //发送数据0
        {
						WS2811_Send_L();
        }
        dat <<= 1;
    }
}
void WS2811_Send24bit(u32 dat)		//@高位先传
{
    u8 i;
    for(i=0;i<24;i++)
    {
        if(dat & 0x800000) //发送数据1
        {
					WS2811_Send_H();
        }
        else           //发送数据0
        {
						WS2811_Send_L();
        }
        dat <<= 1;
    }
}

void Clear_WS2811(void)
{
	u16 i=0;
	
	for(i=0;i<nWs;i++){
		WS2811_SendByte(0);
		Delay1us();
		Delay1us();
		Delay1us();
	}
	WS2811_Reset();
}

int LED_SPEED = 15;
void liushui123x(BOOL RL,u16 led_num)// 顺序三路单灯流水
{
	u8 n,i,num;
  for(i=0;i<100;i++)  //RGB_count+8
	{   
		if(RL)//RL = 1 由1->3
		{
			for(n=led_num;n>0;n--)
			{		
					WS2811_SendByte(num=((i+2)%3)?0:200); 
					WS2811_SendByte(num=((i+1)%3)?0:200);
					WS2811_SendByte(num=((i+0)%3)?0:200);		
	    }
		}
		else{
			for(n=led_num;n>0;n--)
			{
					WS2811_SendByte(num=((i+0)%3)?0:200); 
					WS2811_SendByte(num=((i+1)%3)?0:200);
					WS2811_SendByte(num=((i+2)%3)?0:200);
			}	
		}
			 WS2811_Reset();
			 DelayMS(10*LED_SPEED);
	}
	
}