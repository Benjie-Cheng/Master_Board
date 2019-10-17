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


typedef struct{
	u16 Head;
	u16 Tail;
	u16 Lenght;
	u8 Ring_Buff[RINGBUFF_LEN];
}RingBuff_t;
RingBuff_t ringBuff;//创建一个ringBuff的缓冲区

void vRingBuff_Init(void)
{
	//初始化相关信息
	ringBuff.Head = 0;
	ringBuff.Tail = 0;
	ringBuff.Lenght = 0;
}
u8 uWrite_RingBuff(u8 data)
{
	if(ringBuff.Lenght >= RINGBUFF_LEN) //判断缓冲区是否已满
	{
		return FLASE;
	}
	ringBuff.Ring_Buff[ringBuff.Tail]=data;
	//    ringBuff.Tail++;
	ringBuff.Tail = (ringBuff.Tail+1)%RINGBUFF_LEN;//防止越界非法访问
	ringBuff.Lenght++;
	return TRUE;
}
u8 uRead_RingBuff(u8 *rData)
{
	if(ringBuff.Lenght == 0)//判断非空
	{
		*rData=0;
		return FLASE;
	}
	*rData = ringBuff.Ring_Buff[ringBuff.Head];//先进先出FIFO，从缓冲区头出
//   ringBuff.Head++;
	ringBuff.Head = (ringBuff.Head+1)%RINGBUFF_LEN;//防止越界非法访问
	ringBuff.Lenght--;
	return TRUE;
}
u8 uCheck_Msg()//判断满函数
{
    if (ringBuff.Lenght==RINGBUFF_LEN) {
        return TRUE;
    }
    else
    {
        return FLASE;
    }
}
u8 uCheck_ADP()//检测有无数据
{
    if (ringBuff.Lenght)
        return TRUE;
    else 
        return FLASE;
}