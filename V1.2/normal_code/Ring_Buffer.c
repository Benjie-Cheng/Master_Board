#include<stdio.h>
#include <stdlib.h>

typedef 	unsigned int	u16;
typedef 	unsigned char	u8;

#define  RINGBUFF_LEN          10     //定义最大接收字节数 200 
#define  FLASE   0 
#define  TRUE    1 
void RingBuff_Init(void); 
u8 Write_RingBuff(u8 data); 
u8 Read_RingBuff(u8 *rData);
u8 display[10]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0a};

typedef struct{
	u16 Head;
	u16 Tail;
	u16 Lenght;
	u8 Ring_Buff[RINGBUFF_LEN];
}RingBuff_t;
RingBuff_t ringBuff;//创建一个ringBuff的缓冲区

void RingBuff_Init(void)
{
	//初始化相关信息
	ringBuff.Head = 0;
	ringBuff.Tail = 0;
	ringBuff.Lenght = 0;
}
u8 Write_RingBuff(u8 data)
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
u8 Read_RingBuff(u8 *rData)
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

void main(void)
{
	u8 i,k;
	RingBuff_Init();
	for(i=0;i<10;i++){
		if(uCheck_Msg()){
			printf("busy=%d\n",uCheck_Msg());
			break;
		}else
			Write_RingBuff(display[i]);
	}
	
	printf("k = %d,uCheck_ADP=%d,busy=%d\n",k,uCheck_ADP(),uCheck_Msg());
	printf("\n");
	for(i=0;i<12;i++){
		if(uCheck_ADP()){
			Read_RingBuff(&k);
			printf("k = %d\n",k);
			}
		else
			printf("no data \n");
	}
	
}