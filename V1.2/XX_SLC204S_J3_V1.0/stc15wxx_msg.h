#ifndef __STC15Wxx_MSG_H
#define __STC15Wxx_MSG_H

	#define  RINGBUFF_LEN  10      //定义最大接收字节数 200 
    
/** @defgroup 
  * @{
  */
extern void vRingBuff_Init(void); 
extern u8 uWrite_RingBuff(u8 data); 
extern u8 uRead_RingBuff(u8 *rData);
extern u8 uCheck_Msg();
extern u8 uCheck_ADP();

/**
  * @}
  */
#endif
/******************* (C) COPYRIGHT 2013 @Vampireyifeng*****END OF FILE****/