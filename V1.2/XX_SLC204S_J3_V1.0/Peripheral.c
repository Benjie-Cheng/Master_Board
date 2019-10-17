/**
  ***************************************************************
  * @file    
  * @author  
  * @version 
  * @date    
  * @brief  
  * @attention
  **************************************************************
  * <h2><center>&copy; COPYRIGHT 2013 @Vampireyifeng</center></h2>
  */
  
#include"stc15wxx_conf.h"

#define const_key_time_short  20    //短按的按键去抖动延时的时间
#define const_key_time_long   400   //长按的按键去抖动延时的时间

unsigned char ucKeySec=0;   //被触发的按键编号

unsigned int  uiKeyTimeCnt1=0; //按键去抖动延时计数器
unsigned char ucKeyLock1=0; //按键触发后自锁的变量标志
unsigned char ucShortTouchFlag1=0; //短按的触发标志

unsigned int  uiKeyTimeCnt2=0; //按键去抖动延时计数器
unsigned char ucKeyLock2=0; //按键触发后自锁的变量标志
unsigned char ucShortTouchFlag2=0; //短按的触发标志

/****************向HC595发送一个字节函数 ******************/
void vDataIn595(u8 dat)
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
/****************HC595数据锁存函数 ******************/
void vDataOut595(void)
{		
	A_HC595_RCLK = 0;
	NOP2();
	A_HC595_RCLK = 1;
}

void vTaskKeySan(void)
{
	if(Speed_Key==1)//IO是高电平，说明两个按键没有全部被按下，这时要及时清零一些标志位
	{
		ucKeyLock1=0; //按键自锁标志清零
		uiKeyTimeCnt1=0;//按键去抖动延时计数器清零
		if(ucShortTouchFlag1==1)  //短按触发标志
		{
			ucShortTouchFlag1=0;
			ucKeySec=1;    //触发一号键的短按
		}
	}
	else if(ucKeyLock1==0)//有按键按下，且是第一次被按下
	{
		uiKeyTimeCnt1++; //累加定时中断次数
		if(uiKeyTimeCnt1>const_key_time_short)
		{
			ucShortTouchFlag1=1;   //激活按键短按的有效标志  
		}
		if(uiKeyTimeCnt1>const_key_time_long)
		{
			ucShortTouchFlag1=0;  //清除按键短按的有效标志
			uiKeyTimeCnt1=0;
			ucKeyLock1=1;  //自锁按键置位,避免一直触发
			ucKeySec=2;    //触发1号键的长按			
		}
	}
	
	if(Mode_Key==1)//IO是高电平，说明两个按键没有全部被按下，这时要及时清零一些标志位
	{
		ucKeyLock2=0; //按键自锁标志清零
		uiKeyTimeCnt2=0;//按键去抖动延时计数器清零   
		if(ucShortTouchFlag2==1)  //短按触发标志
		{
			ucShortTouchFlag2=0;
			ucKeySec=3;    //触发2号键的短按
		}
	}
	else if(ucKeyLock2==0)//有按键按下，且是第一次被按下
	{
		uiKeyTimeCnt2++; //累加定时中断次数
		if(uiKeyTimeCnt2>const_key_time_short)
		{
			ucShortTouchFlag2=1;   //激活按键短按的有效标志  
		}
	
		if(uiKeyTimeCnt2>const_key_time_long)
		{
			ucShortTouchFlag2=0;  //清除按键短按的有效标志
			uiKeyTimeCnt2=0;
			ucKeyLock2=1;  //自锁按键置位,避免一直触发	
			ucKeySec=4;    //触发2号键的长按		
		}
	}	
}
/*
void vKey_Service(void) //按键服务的应用程序
{
  switch(ucKeySec) //按键服务状态切换
  {
    case 1:// 1号键的短按 
          ucKeySec=0;  //响应按键服务处理程序后，按键编号清零，避免一致触发
          break;        
    case 2:// 1号键的长按  
          ucKeySec=0;  //响应按键服务处理程序后，按键编号清零，避免一致触发
          break;      
    case 3:// 2号键的短按 
          ucKeySec=0;  //响应按键服务处理程序后，按键编号清零，避免一致触发
          break;        
    case 4:// 2号键的长按
          ucKeySec=0;  //响应按键服务处理程序后，按键编号清零，避免一致触发
          break;   
  }               
}
*/