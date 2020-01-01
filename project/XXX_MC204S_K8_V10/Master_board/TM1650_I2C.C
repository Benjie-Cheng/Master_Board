#include "TM1650_I2C.h"


 
//--------------------------------------------------------------
// Prototype      : void IIC_Init_TM1650(void)
// Calls          : 
// Description    : 
//--------------------------------------------------------------
void IIC_Init_TM1650(void)
{
   /*空闲状态*/
   SCL_TM1650  = 1;
   SDA_TM1650  = 1;	
   TDelay_us(5);
   TM1650_Set(CMD_MODE, BL(8) | P7_MODE | CMD_ON);//5级亮度，7段显示，关闭显示
}
void Tube_CMD(u8 mode, u8 Brightness)
{
	u8 p7_mode = 0;
	if(mode)
		p7_mode = P8_MODE;
    if(Brightness)
    {
        TM1650_Set(CMD_MODE, BL(Brightness) | P7_MODE | CMD_ON);//开启显示
    }
    else
    {
        TM1650_Set(CMD_MODE, BL(Brightness) | P7_MODE | CMD_OFF);//关闭显示
    }
}

//--------------------------------------------------------------
// Prototype      : void Delay_us(void)
// Description    : 大约延时 z us
//--------------------------------------------------------------
void TDelay_us(u8 z)
{   
	for(;z>0;z--)
	{
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
	}
}
//--------------------------------------------------------------
// Prototype      : void I2C_Start(void)
// Calls          : Delay_5us()
// Description    : Start Singnal
//--------------------------------------------------------------
void I2C_Start_TM1650(void)
{
    // SDA 1->0 while SCL High
  	//SCL高电平期间，SDA出现一个下降沿表示起始信号
    SCL_TM1650 = 1;     //时钟线保持为高          
    SDA_TM1650 = 1;    	//数据线先保持为高，起始信号要该口的下降沿 	
    TDelay_us(5);  //有一个大概5us的延时具体以器件而定            
    SDA_TM1650 = 0;        //数据线拉低出现下降沿   
}
 
 
//--------------------------------------------------------------
// Prototype      : void I2C_Stop(void)
// Calls          : Delay_5us()
// Description    : Stop Singnal
//-------------------------------------------------------------- 
void I2C_Stop_TM1650(void)
{
    // SDA 0->1 while SCL High
    //SCL高电平期间，SDA产生一个上升沿 表示停止
    SCL_TM1650 = 1;		//先保证时钟线为高电平
	SDA_TM1650 = 0;		//保证数据线为低电平
    TDelay_us(5);  //延时 以得到一个可靠的电平信号            
    SDA_TM1650 = 1;        //数据线出现上升沿   
}
 
 
//应答函数
void IIC_Ack_TM1650(void)
{
	u8 timeout = 1;
	SCL_TM1650 = 1;
	TDelay_us(5);
	SCL_TM1650 = 0;
	while((SDA_TM1650)&&(timeout<=100))
	{
		timeout++;
	}
	TDelay_us(5);
	SCL_TM1650 = 0;
}
 
void IIC_WrByte_TM1650(u8 txd)//写一个字节高位在前，低位在后
{

    //定义一个计数变量
    u8 i;
    //将时钟线拉低允许数据改变
	SCL_TM1650=0;
	TDelay_us(1);  
    //按位发送数据
    for(i = 0;i < 8; i ++)
    {
		txd = txd<<1;
		SDA_TM1650 = CY;		
		SCL_TM1650=0;			
		TDelay_us(IIC_uS);   
		SCL_TM1650=1;
		TDelay_us(IIC_uS);  
		SCL_TM1650=0;	
    }
}
 
/*
u8 Scan_Key(void)	  //按键扫描
{
	u8 i;
	u8 rekey;
	I2C_Start_TM1650();
	IIC_WrByte_TM1650(CMD_KEY_MODE);//读按键命令
	IIC_Ack_TM1650();
	for(i=0;i<8;i++)
	{
	   SCL_TM1650=1;
	   rekey = rekey<<1;
	   
	   if(SDA_TM1650)
	   {
	   	rekey++;
	   } 
	   TDelay_us(5);
	   SCL_TM1650=0;	
	}
	IIC_Ack_TM1650();
	I2C_Stop_TM1650();
	return(rekey);
}
*/
void TM1650_Set(u8 add,u8 dat) //数码管显示
{
	//写显存必须从高地址开始写
	I2C_Start_TM1650();
	IIC_WrByte_TM1650(add); //第一个显存地址
	IIC_Ack_TM1650();
	IIC_WrByte_TM1650(dat);
	IIC_Ack_TM1650();
	I2C_Stop_TM1650();
}
void Init_Tm1650(void)
{
	IIC_Init_TM1650();
}
