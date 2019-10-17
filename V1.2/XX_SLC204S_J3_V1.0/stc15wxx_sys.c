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

  /** @addtogroup 
  * @{
  */

/** @defgroup 
  * @brief 
  * @{
  */ 

/** @defgroup 
  * @{
  */
/**
  * @}
  */

/** @defgroup 
  * @{
  */
void delay_ms(u8 ms)
{
   u16 i;
	 do{
	      i = MAIN_Fosc / 13000;
		  while(--i)	;   //14T per loop
     }while(--ms);
}
void vDelayMS(u16 t)
{
	while(t--)
	{
		delay_ms(1);
	}
}
void Timer2Init(void)
{
	Timer2_AsTimer();//使用定时器2
	Timer2_InterruptDisable();
	Timer2_Stop();	
	Timer2_1T();
	T2L = 0x66;		//1ms定时
	T2H = 0x7E;		//1ms定时
	Timer2_InterruptEnable();
	Timer2_Run();
}
void Timer0_init(void)//用于模拟中断的定时器
{
	Timer0_1T();
	Timer0_AsTimer();
	Timer0_16bitAutoReload();
	Timer0_Load(Timer0_Reload);
	Timer0_InterruptEnable();
	PT0 = 1;	//定时器0高优先级中断
	Timer0_Run();
	EA = 1;					//打开总中断					open global interrupt switch
}
void vInit_MCU(void)
{
	P0n_standard(0xff);	//设置为准双向口
	P1n_standard(0xff);	//设置为准双向口
	P2n_standard(0xff);	//设置为准双向口
	//P3n_standard(0xff);	//设置为准双向口
	P4n_standard(0xff);	//设置为准双向口
	P5n_standard(0xff);	//设置为准双向口
	//InternalRAM_enable();
	//Timer0_init();//用于模拟中断的定时器
	Timer2Init();//用于1ms定时
	
}
/**
  * @brief  
  * @note   
  * @param  
  * @retval 
  *   
  *     @arg 
  *     @arg 
  *     @arg 
  * @retval 
  *@deprecated
  */


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
/******************* (C) COPYRIGHT 2013 @Vampireyifeng*****END OF FILE****/



