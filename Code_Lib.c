//带动1200路/3
#define WS2811_SDA_GPIO	P10

void WS2811_Delay1us(void)		//@22.1184MHz
{
	unsigned char i;
	i = 3;
	while (--i);
}
void WS2811_Delayus(u8 time)		//@22.1184MHz
{
	for(i=0;i < time;i++)
		WS2811_Delay1us();
}
WS2811_Send_H(void)		//@发送高电平“1”
{
	WS2811_SDA_GPIO=1; 
	WS2811_Delayus(2);
    WS2811_SDA_GPIO=0;	
	WS2811_Delayus(2);
}
WS2811_Send_L(void)		//@发送低电平“0”
{
	WS2811_SDA_GPIO=1; 
	WS2811_Delayus(1);
    WS2811_SDA_GPIO=0;	
	WS2811_Delayus(4);
}
void WS2811_Reset(void)  //@复位芯片
{
	WS2811_SDA_GPIO=0;
	WS2811_Delayus(60);
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
//-----------------------------------------------------------------------------
/********************** A  595 控制脚************************/
sbit	A_HC595_SER   = P3^4;	//pin 34	SER		data input
sbit	A_HC595_RCLK  = P3^7;	//pin 37	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P3^5;	//pin 35	SRCLK	Shift data clock
sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		低电平 使能enable pin
sbit	A_HC595_MR    = P3^6;	//pin 36	低电平复位	
/**************** 向HC595发送一个字节函数 ******************/
void HC595_Data_in(u8 dat)
{		
	u8	i;
	for(i=0; i<8; i++)
	{
		dat <<= 1;
		A_HC595_SER   = CY;
		A_HC595_SRCLK = 0;
		NOP2();
		A_HC595_SRCLK = 1;
	}
}
/**************** HC595数据锁存函数 ******************/
void HC595_Data_out()
{		
	A_HC595_RCLK = 0;
	NOP2();
	A_HC595_RCLK = 1;
}
//
void HC595_Send_L(u16 num)		//@发num 个0
{		
	u8	i;
	for(i=0; i< num; i++)
	{
		A_HC595_SER   = 0;
		A_HC595_SRCLK = 0;
		NOP2();
		A_HC595_SRCLK = 1;
	}
	HC595_Data_out();
}
void HC595_Send_H(u16 num)		//@发num 个1
{		
	u16	i;
	for(i=0; i< num; i++)
	{
		A_HC595_SER   = 1;
		A_HC595_SRCLK = 0;
		NOP2();
		A_HC595_SRCLK = 1;
	}
	HC595_Data_out();
}
#define LEDS_NUM 100
//从第一个点亮到最后一个
void LED1(u16 num)
{
	u16	i;
	for(i=0; i< num; i++)
	{
		HC595_Send_H(1);
		delayms(125*SHUDUNUMM);
	}	
}




























