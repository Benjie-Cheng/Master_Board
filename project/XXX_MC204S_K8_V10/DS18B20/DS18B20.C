#include "DS18B20.h"


#define DS_DELAY_1US(n) {delay_us(n);}
/*
*分辨率：9-12bit，0.5℃、0.25℃、0.125℃、0.0625℃
*/

//--------------------------------------------------------------
// Prototype      : void DS18B20_init(void)
// Calls          : 
// Description    : 
//--------------------------------------------------------------
void DS18B20_init(void)
{
       DS18B20_DQ=1;
       DS_DELAY_1US(1);     //稍作延时
       DS18B20_DQ=0;
       DS_DELAY_1US(600);    //延时480到960us-----------精确延时
       DS18B20_DQ=1;
       i = 0;
       while(DS18B20_DQ)    //等待DS18B20拉低总线
       {
           DS_DELAY_1US(100);
           i++;
           if(i>5)//约等待>5MS
           {
               return 0;//初始化失败
           }    
       }
}
//--------------------------------------------------------------
// Prototype      : void Ds_WriteByte(u8 dat)
// Calls          : 
// Description    : 写一个字节
//--------------------------------------------------------------
static void Ds_WriteByte(u8 dat)   //写一个字节
{
   u8 i;
   for(i=0;i<8;i++)
   {
     DS18B20_DQ=0;  //每写入一位数据之前先把总线拉低1us
     _nop_();
     DS18B20_DQ=dat&0x01;    //取最低位写入
     DS_DELAY_1US(68);   //延时68us，持续时间最少60us,给从机采样
     DS18B20_DQ=1;   //然后释放总线
     dat=dat>>1;    //从低位开始写
   }
   DS_DELAY_1US(10);
}
//--------------------------------------------------------------
// Prototype      : u8 DS_ReadByte(void)
// Calls          : 
// Description    : 读一个字节
//--------------------------------------------------------------
static u8 DS_ReadByte(void)    //读一个字节
{
  u8 i,dat=0;
  for(i=0;i<8;i++)
  {
     DS18B20_DQ=0;  //先将总线拉低1us
     _nop_();
     DS18B20_DQ=1;  //然后释放总线
     _nop_();_nop_();
     _nop_();_nop_();
     if(DS18B20_DQ) dat=dat|0x80;   //每次读一位
     dat=dat>>1;       //从最低位开始读
     DS_DELAY_1US(45);   //读取完之后等待48us再接着读取下一个数
   }
   return dat;
}
//--------------------------------------------------------------
// Prototype      : u16 DS_ReadTempCfg(void)
// Calls          : 
// Description    : 读取ROM值
//--------------------------------------------------------------
static u16 DS_ReadTempCfg (void)
{    
   u8 a,b;         
   u16 t=0;
   DS18B20_init();       
   DS_DELAY_1US(15);
   Ds_WriteByte(0xcc);     //跳过ROM操作命令
   Ds_WriteByte(0x44);     //发送启动温度转换命令
   DS18B20_init();       
   DS_DELAY_1US(15);
   Ds_WriteByte(0xcc);    //跳过ROM操作命令
   Ds_WriteByte(0xbe);    //发送读温度寄存器命令
   a=DS_ReadByte();       //先读低八位
   b=DS_ReadByte();       //再读高八位
   t=b;        
   t<<=8;      //左移八位
   t=t|a;      //t为16位的数，使高八位为b的值，低八位为a的值  
   return t;    //返回温度值
}
//--------------------------------------------------------------
// Prototype      : u16 Ds_GetTemper(void)
// Calls          : 
// Description    : 获取温度
//--------------------------------------------------------------
u16 Ds_GetTemper(void)
{
    u16 temper;
    float tp;
    temper=DS_ReadTempCfg();
    if(temper<0)    //考虑负温度的情况
    {
        temper=temper-1;
        temper=~temper;
        tp=temper*0.0625;  //16位温度转换成10进制的温度
        temper=tp*100+0.5;   //留两个小数点，并四舍五入
    }
    else
    {
        tp=temper*0.0625;  //16位温度转换成10进制的温度
        temper=tp*100+0.5;  //留两个小数点，并四舍五入
    }
    return temper;
}