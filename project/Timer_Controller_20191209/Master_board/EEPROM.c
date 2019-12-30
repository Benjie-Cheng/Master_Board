
/*------------------------------------------------------------------*/
/* --- STC MCU International Limited -------------------------------*/
/* --- STC 1T Series MCU RC Demo -----------------------------------*/
/* --- Mobile: (86)13922805190 -------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ---------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ---------------------*/
/* --- Web: www.GXWMCU.com -----------------------------------------*/
/* --- QQ:  800003751 ----------------------------------------------*/
/* If you want to use the program or the program referenced in the  */
/* article, please specify in which data and procedures from STC    */
/*------------------------------------------------------------------*/


//	本程序是STC系列的内置EEPROM读写程序。

#include "config.h"
#include "eeprom.h"

/*----------------------------
??IAP
----------------------------*/
void IapIdle()
{
    IAP_CONTR = 0;                  //??IAP??
    IAP_CMD = 0;                    //???????
    IAP_TRIG = 0;                   //???????
    IAP_ADDRH = 0x80;               //???????IAP??
    IAP_ADDRL = 0;
}

/*----------------------------
?ISP/IAP/EEPROM???????
----------------------------*/
BYTE IapReadByte(WORD addr)
{
    BYTE dat;                       //?????

    IAP_CONTR = ENABLE_IAP;         //??IAP
    IAP_CMD = CMD_READ;             //??IAP??
    IAP_ADDRL = addr;               //??IAP???
    IAP_ADDRH = addr >> 8;          //??IAP???
    IAP_TRIG = 0x5a;                //?????(0x5a)
    IAP_TRIG = 0xa5;                //?????(0xa5)
    _nop_();                        //??ISP/IAP/EEPROM????
    dat = IAP_DATA;                 //?ISP/IAP/EEPROM??
    IapIdle();                      //??IAP??

    return dat;                     //??
}

/*----------------------------
???????ISP/IAP/EEPROM??
----------------------------*/
void IapProgramByte(WORD addr, BYTE dat)
{
    IAP_CONTR = ENABLE_IAP;         //??IAP
    IAP_CMD = CMD_PROGRAM;          //??IAP??
    IAP_ADDRL = addr;               //??IAP???
    IAP_ADDRH = addr >> 8;          //??IAP???
    IAP_DATA = dat;                 //?ISP/IAP/EEPROM??
    IAP_TRIG = 0x5a;                //?????(0x5a)
    IAP_TRIG = 0xa5;                //?????(0xa5)
    _nop_();                        //??ISP/IAP/EEPROM????
    IapIdle();
}

/*----------------------------
????
----------------------------*/
void IapEraseSector(WORD addr)
{
    IAP_CONTR = ENABLE_IAP;         //??IAP
    IAP_CMD = CMD_ERASE;            //??IAP??
    IAP_ADDRL = addr;               //??IAP???
    IAP_ADDRH = addr >> 8;          //??IAP???
    IAP_TRIG = 0x5a;                //?????(0x5a)
    IAP_TRIG = 0xa5;                //?????(0xa5)
    _nop_();                        //??ISP/IAP/EEPROM????
    IapIdle();
}
