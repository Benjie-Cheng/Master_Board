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
	Timer2_AsTimer();//ʹ�ö�ʱ��2
	Timer2_InterruptDisable();
	Timer2_Stop();	
	Timer2_1T();
	T2L = 0x66;		//1ms��ʱ
	T2H = 0x7E;		//1ms��ʱ
	Timer2_InterruptEnable();
	Timer2_Run();
}
void Timer0_init(void)//����ģ���жϵĶ�ʱ��
{
	Timer0_1T();
	Timer0_AsTimer();
	Timer0_16bitAutoReload();
	Timer0_Load(Timer0_Reload);
	Timer0_InterruptEnable();
	PT0 = 1;	//��ʱ��0�����ȼ��ж�
	Timer0_Run();
	EA = 1;					//�����ж�					open global interrupt switch
}
void vInit_MCU(void)
{
	P0n_standard(0xff);	//����Ϊ׼˫���
	P1n_standard(0xff);	//����Ϊ׼˫���
	P2n_standard(0xff);	//����Ϊ׼˫���
	//P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���
	//InternalRAM_enable();
	//Timer0_init();//����ģ���жϵĶ�ʱ��
	Timer2Init();//����1ms��ʱ
	
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



