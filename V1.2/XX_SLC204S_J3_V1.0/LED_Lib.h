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
  #ifndef __STC15Wxx_LED_LIB_H
  #define __STC15Wxx_LED_LIB_H

  extern u16 nWs;//LED 个数
  extern void WS2811_SendByte(u8 dat);		//@高位先传
  extern void WS2811_Reset(void);  //@复位芯片
  extern void Clear_WS2811(void);
  extern void liushui123x(BOOL RL,u16 led_num);// 顺序三路单灯流水

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

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
  #endif
/******************* (C) COPYRIGHT 2013 @Vampireyifeng*****END OF FILE****/