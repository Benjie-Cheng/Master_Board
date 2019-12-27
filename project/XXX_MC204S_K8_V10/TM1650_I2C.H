#ifndef __TM1650_I2C_H
#define __TM1650_I2C_H

#include <intrinsics.h>
 
 
/**********************************
TM1650芯片控制数码管
P32为SCL口
P14为SDA口
**********************************/
 
#define SCL_TM1650	P32
#define SDA_TM1650	P14
#define IIC_uS  10 
#define CMD_OFF 0x00 //关闭显示
#define CMD_ON  0x01 //开显示
#define CMD_MODE     0x48 //模式命令
#define CMD_KEY_MODE 0x49 //读按键数据命令
#define DIG1 0x68
#define DIG2 0x6A
#define DIG3 0x6C
#define DIG4 0x6E

#define BL(x)       (x)*16     //亮度等级1~8
#define PMODE(x)     x ?  0x08 : 0x00
 
/*
#define SDAM_TM1650              PC_IDR_IDR6
#define SET_SCL_OUT_TM1650()    {PC_DDR_DDR7=1; PC_CR1_C17 = 1; PC_CR2_C27 = 0;}
#define SET_SDA_OUT_TM1650()    {PC_DDR_DDR6=1; PC_CR1_C16 = 1; PC_CR2_C26 = 0;}
#define SET_SDA_IN_TM1650()     {PC_DDR_DDR6=0; PC_CR1_C16 = 0; PC_CR2_C26 = 0;}
*/
 
void IIC_Init_TM1650(void);
void TDelay_us(u8 z);
void I2C_Start_TM1650(void);
void I2C_Stop_TM1650(void);
void IIC_Ack_TM1650(void);
void IIC_NAck_TM1650(void);
uint8_t IIC_Wait_Ack_TM1650(void);
void IIC_WrByte_TM1650(uint8_t txd);
 
//u8 Scan_Key(void);
void TM1650_Set(u8 add,u8 dat);
void Init_Tm1650(void);
void Tube_CMD(u8 mode, u8 Brightness);
 
#endif