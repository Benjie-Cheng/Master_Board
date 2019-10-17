//========================================================================
#include"stc15wxx_conf.h"

// 描述: 模拟串口相关。
#define FAKE_SERIAL
#if defined FAKE_SERIAL

//========================================================================
// 函数: void	BitTime(void)
// 描述: 位时间函数。
// 参数: none.
// 返回: none.
// 版本: VER1.0
// 日期: 2013-4-1
// 备注: 
//========================================================================
void BitTime(void)
{
	u16 i;
	i = ((MAIN_Fosc / 100) * 104) / 130000L - 1;		//根据主时钟来计算位时间
	while(--i);
}
//========================================================================
// 函数: void	TxSend(uchar dat)
// 描述: 模拟串口发送一个字节。9600，N，8，1
// 参数: dat: 要发送的数据字节.
// 返回: none.
// 版本: VER1.0
// 日期: 2013-4-1
// 备注: 
//========================================================================
void TxSend(u8 soft_uart,u8 dat)
{
	u8	i;
	EA = 1;
	Timer0_Stop();
	if(soft_uart==TX_P31)
		P_TXD = 0;
	else if(soft_uart==TX_DENUG)
		TX_DENUG_GPIO = 0;
	BitTime();
	for(i=0; i<8; i++)
	{
		if(dat & 1){	
				if(soft_uart==TX_P31)
					P_TXD = 1;
				else if(soft_uart==TX_DENUG)
					TX_DENUG_GPIO = 1;
		}
		else{		
			if(soft_uart==TX_P31)
				P_TXD = 0;
			else if(soft_uart==TX_DENUG)
					TX_DENUG_GPIO = 0;			
		}
		dat >>= 1;
		BitTime();
	}
	if(soft_uart==TX_P31)
		P_TXD = 1;
	else if(soft_uart==TX_DENUG)
		TX_DENUG_GPIO = 1;
	EA = 1;
	Timer0_Run();
	BitTime();
	BitTime();
}
//========================================================================
// 函数: void PrintString(unsigned char code *puts)
// 描述: 模拟串口发送一串字符串。9600，N，8，1
// 参数: *puts: 要发送的字符指针.
// 返回: none.
// 版本: VER1.0
// 日期: 2013-4-1
// 备注: 
//========================================================================
void Fake_PrintString(u8 soft_uart,u8 *puts)
{
    for (; *puts != 0;	puts++)  TxSend(soft_uart,*puts);
}

void Fake_PrintString1(u8 soft_uart,u8 *puts)
{
    for (; *puts != '*';	puts++)  TxSend(soft_uart,*puts);
}
#define LED_DEBUG
#if defined LED_DEBUG
//========================================================================
// 描述: 按键指示灯 。
//========================================================================
void Debug_LedOn(BOOL enable)
{
	if(enable)
		LED_DEBUG_GPIO = TRUE;
	else
		LED_DEBUG_GPIO = FALSE;
}
void Debug_LedReverse(void)
{
	LED_DEBUG_GPIO = ~LED_DEBUG_GPIO;	
}
#endif

#endif


