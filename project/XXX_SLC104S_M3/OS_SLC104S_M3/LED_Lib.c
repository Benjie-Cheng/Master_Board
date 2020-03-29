#include "led_lib.h"
//#include <rtx51tny.h>                 /* RTX-51 tiny functions & defines      */

#define WS2811_SDA_GPIO_H() {WS2811_SDA_GPIO=1;WS2811_SDA1_GPIO = 1;}
#define WS2811_SDA_GPIO_L() {WS2811_SDA_GPIO=0;WS2811_SDA1_GPIO = 0;}
#define BLUE_LIGHT(BL) {WS2811_SendByte(BL);WS2811_SendByte(0);WS2811_SendByte(0);}
#define RED_LIGHT(BL) {WS2811_SendByte(0);WS2811_SendByte(BL);WS2811_SendByte(0);}
#define GREEN_LIGHT(BL) {WS2811_SendByte(0);WS2811_SendByte(0);WS2811_SendByte(BL);}
#define WHITE_LIGHT(BL) {WS2811_SendByte(BL);WS2811_SendByte(BL);WS2811_SendByte(BL);}
#define LED_SET(TYPE,BL) {\
	if(BLUE==TYPE){BLUE_LIGHT(BL);}\
	else if(RED==TYPE){RED_LIGHT(BL);}\
	else if(GREEN==TYPE){GREEN_LIGHT(BL);}\
	else if(WHITE==TYPE){WHITE_LIGHT(BL);}}

//extern   SysRun.LedNum = 1;
#ifdef HZ_11059200
void Delay1us()		//@11.0592MHz
{
	_nop_();
	_nop_();
	_nop_();
}
#elif HZ_22118400
void Delay1us()		//@22.1184MHz
{
	unsigned char i;

	i = 3;
	while (--i);
}
#elif HZ_33177600
void Delay1us()		//@33.1776MHz
{
	unsigned char i;

	_nop_();
	_nop_();
	_nop_();
	i = 5;
	while (--i);
}
#elif HZ_35000000
void Delay1us()		//@35MHz
{
	unsigned char i;

	_nop_();

	i = 1;
	//i = 6;//200个灯会闪。
	while (--i);
}
#endif

void WS2811_Send_H(void)		//@发送高电平“1”
{
	WS2811_SDA_GPIO_H(); 
	Delay1us();
  WS2811_SDA_GPIO_L();
  //Delay1us();

}
void WS2811_Send_L(void)		//@发送低电平“0”
{
	WS2811_SDA_GPIO_H(); 
	//Delay1us();
	WS2811_SDA_GPIO_L();
	Delay1us();
	//Delay1us();
	/*********************
	加了下面两个延时，led灯超过100个会闪烁
	*********************/
	//Delay1us();
	//Delay1us();
}

void Delay60us()		//@11.0592MHz
{
	unsigned char i, j;

	i = 1;
	j = 162;
	do
	{
		while (--j);
	} while (--i);
}
void WS2811_Reset(void)  //@复位芯片
{
	u8 i ;
	//WS2811_SDA_GPIO_H();
	WS2811_SDA_GPIO_L();
	for(i=0;i<60;i++)
		Delay1us();
}
//========================================================================
// 描述: WS2811发送一字节数据@高位先传
//========================================================================
void WS2811_SendByte(u8 dat)		//@高位先传
{
    u8 i;
    for(i=0;i<8;i++)
    {
        if(dat & 0x80) //发送数据1
        {
					WS2811_Send_H();
        }
        else           //发送数据0
        {
						WS2811_Send_L();
        }
        dat <<= 1;
    }
}
void WS2811_Send24bit(u32 dat)		//@高位先传
{
    u8 i;
    for(i=0;i<24;i++)
    {
        if(dat & 0x800000) //发送数据1
        {
					WS2811_Send_H();
        }
        else           //发送数据0
        {
						WS2811_Send_L();
        }
        dat <<= 1;
    }
}

void Clear_WS2811(void)
{
	u16 i=0;
	
	for(i=0;i<(SysRun.LedNum+300);i++){
		WS2811_SendByte(0);
		Delay1us();
		Delay1us();
		Delay1us();
	}
	WS2811_Reset();
}

int LED_SPEED = 50;
void liushui123x(BOOL RL,u16 led_num)// 顺序三路单灯流水
{
	u8 n,i,num;
  for(i=0;i<SysRun.LedNum;i++)  //RGB_count+8
	{   
		if(RL)//RL = 1 由1->3
		{
			for(n=SysRun.LedNum;n>0;n--)
			{		
					WS2811_SendByte(num=((i+2)%3)?0:200); 
					WS2811_SendByte(num=((i+1)%3)?0:200);
					WS2811_SendByte(num=((i+0)%3)?0:200);		
	     }
		 }
		else{
			for(n=led_num;n>0;n--)
			{
					WS2811_SendByte(num=((i+0)%3)?0:200); 
					WS2811_SendByte(num=((i+1)%3)?0:200);
					WS2811_SendByte(num=((i+2)%3)?0:200);
			}	
		}
			 WS2811_Reset();
			// DelayMS(10*LED_SPEED);
			os_wait2(K_TMO, 20);
	}
	
}
void PAOMA(u8 GG1,u8 GG2)//
{
	u8 n,i,num;
  for(i=0;i<100;i++)  //RGB_count+8
	{   
		if(i>7)
		{   
			for(num=i-8;num>0;num--)
			{		
				WS2811_SendByte(GG1);	
			}
																								 
			for(n=8;n>0;n--)
			{
				WS2811_SendByte(GG2);
			}
		} 
		else
		{
			for(n=i;n>0;n--)
			{
				WS2811_SendByte(GG2);	    
			}
		}	
			WS2811_Reset();
			os_wait2(K_TMO, 5);
	  }
}
void PAOMAADD(u8 TYPE,u16 GG1,u16 GG2)//单色单灯叠加亮，底色单灯叠加灭
{ 										  
	u8 i,num;
	for(i=0;i<SysRun.LedNum;i++)		 //RGB_count
	{
		for(num=0;num<=i;num++)
		{
			LED_SET(TYPE,GG1);
		}
		for(num=SysRun.LedNum-i;num>0;num--)  //RGB_count
		{
			//WS2811_SendByte(GG2);
			LED_SET(TYPE,GG2);
		}
		WS2811_Reset();// reset();
		os_wait2(K_TMO, 5);
		//os_wait2(K_TMO, 100);			
	}
}
void PAOMADEC(u8 GG1,u8 GG2)//单色单灯减灭，底色单灯减灭
{
	u8 i,num;//,RGB_count=RGB_c; 
	for(i=SysRun.LedNum;i>0;i--)	//RGB_count+8
	{						 
		for(num=0;num<=i;num++)
		{
			WS2811_SendByte(GG1);
//    WS2811_SendByte(RR1);
//    WS2811_SendByte(BB1);
    }
    for(num=SysRun.LedNum-i;num>0;num--)	//RGB_count+8
    {
			WS2811_SendByte(GG2);
//    WS2811_SendByte(RR2);
//    WS2811_SendByte(BB2);
    }
    WS2811_Reset();// reset(); 
		os_wait2(K_TMO, 5);
	} 
}
void LIUXINGYU32(bit bG)//32灯拖尾变暗流水
{
	u16 nn,ii,nnum,NLed;
	NLed=SysRun.LedNum+10;
 	for(ii=0;ii<NLed*3;ii++)
  {   
		if(ii>31)//
		{   
			for(nnum=32;nnum<=ii;nnum++)
			{		
				WS2811_SendByte(0);	
			}																				 
			for(nn=32;nn>0;nn--)
			{
					WS2811_SendByte(nnum=bG?(128-nn*4):0);
			}
			for(nnum=NLed*3+32-ii;nnum>0;nnum--)
	    {		
				WS2811_SendByte(0);
	    }
		} 
		else
		{
			for(nn=ii;nn>0;nn--)//nn<32时
			{
				WS2811_SendByte(nnum=bG?(128-nn*4):0);	    
			}
			for(nnum=NLed*3+32-ii;nnum>0;nnum--)
	    {		
				WS2811_SendByte(0);
			}
		}	
     WS2811_Reset();// reset();
		 os_wait2(K_TMO, 3);		
	 }
}
void LIUXINGYU16(u8 GG1,bit bG)//16灯拖尾变暗流水
{
	//u8 n,i,num;
	u16 n,i,num,NLed;
	NLed=SysRun.LedNum+10;
  for(i=0;i<NLed*3;i++)
	{   
		if(i>15)
		{   
			for(num=16;num<=i;num++)
			{		
				WS2811_SendByte(GG1);
//	                send_dat(RR1);
//	                send_dat(BB1); 	
			}
																								 
			for(n=16;n>0;n--)
			{
				WS2811_SendByte(num=bG?(240-15*n):0);
//				    send_dat(num=bR?(64-4*n):0);
//				    send_dat(num=bB?(64-4*n):0);
			}
		} 
		else
		{
			for(n=i;n>0;n--)
			{
				WS2811_SendByte(num=bG?(240-15*n):0);
//				    send_dat(num=bR?(64-4*n):0);
//				    send_dat(num=bB?(64-4*n):0);		    
			}
		}	
		WS2811_Reset();
		os_wait2(K_TMO, 3);				
	}
}

u8 code LIUXINGYUDataTable[25] =
{
255,225,190,160,135,110,80,60,45,35,23,15,10,5,3,1,0,0,0,0,0,0,0,0
};
void LIUXINGYU168(bit bG)//顺序每32路间隔16路拖尾
{
  	u8 nn,i,num;
		for(i=0;i<250;i++)
		{   
			for(nn=i;nn>0;nn--)
			{
				WS2811_SendByte(num=bG?LIUXINGYUDataTable[nn%25]:0);
//				    send_dat(num=bR?LIUXINGYUDataTable[nn%25]:0);
//				    send_dat(num=bB?LIUXINGYUDataTable[nn%25]:0);		    
			}															 
		WS2811_Reset();
		os_wait2(K_TMO, 3);			
		}
//  	for(i=250;i>0;i--)
//        {   
//            	for(nn=i;nn>0;nn--)
//				 {
//					send_dat(num=bG?LIUXINGYUDataTable[25-nn%25]:0);
////				    send_dat(num=bR?LIUXINGYUDataTable[25-nn%25]:0);
////				    send_dat(num=bB?LIUXINGYUDataTable[25-nn%25]:0);		    
//				  }															 
//            	reset();
//            	for(t=SHUDU;t>0;t--)  delay(15);		
//	  }
}
void LIUXINGYU8(u8 GG1,bit bG)//顺序8路RGB拖尾+RGB补色
{
	//u8 n,i,t,num;//,RGB_count=RGB_c;
	u16 n,i,num,NLed;
	NLed=SysRun.LedNum+20;
  for(i=0;i<NLed*3+8;i++)
	{   
		if(i>7)
		{   
			for(num=8;num<=i;num++)
			{		
				WS2811_SendByte(GG1);
//	                send_dat(RR1);
//	                send_dat(BB1); 	
			}
																								 
			for(n=8;n>0;n--)
			{
				WS2811_SendByte(num=bG?(192-3*n*n):0);
//				    send_dat(num=bR?(64-n*n):0);
//				    send_dat(num=bB?(64-n*n):0);
;
			}
		} 
		else
		{
			for(n=i;n>0;n--)
			{
				WS2811_SendByte(num=bG?(192-3*n*n):0);
//				    send_dat(num=bR?(64-n*n):0);
//				    send_dat(num=bB?(64-n*n):0);		    
			}
		}	
		WS2811_Reset();
		os_wait2(K_TMO, 3);
	}
}
void LIUXINGYU88(bit bG,bit bG1)// ???16???8?RGB??
{
	u8 n,i,t,num;//,RGB_count=RGB_c;
	u8 chishu=1;
	while(1)
	{
		for(i=1;i<=8;i++)
		{  
			for(t=0;t<(SysRun.LedNum*+1);t++)
			{
				for(n=i;n>0;n--)
				{
					WS2811_SendByte(num=bG?(192-3*n*n):0);
//				    send_dat(num=bR?(64-n*n):0);
//				    send_dat(num=bB?(64-n*n):0);		    
				}
					for(n=0;n<8-i;n++)
				{
					WS2811_SendByte(num=bG1?(3*n*n):0);
//				    send_dat(num=bR1?(n*n):0);
//				    send_dat(num=bB1?(n*n):0);		    
				}
			}	
				WS2811_Reset();
        os_wait2(K_TMO, 50);
		}
		chishu++;
		if (chishu >10)
		break;
	}
}
u8 code r_DataTable[63] =
{
  200,190,180,170,160,150,140,130,120,110,100,90,80,70,60,50,40,30,20,10,0,        
  
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,                                       
  
  0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200
};


u8 code g_DataTable[63] =
{
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,                                            
 
  0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,

  200,190,180,170,160,150,140,130,120,110,100,90,80,70,60,50,40,30,20,10,0,        
 
};

 
u8 code b_DataTable[63] =
{ 
  0,10,20,30,40,50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200,

  200,190,180,170,160,150,140,130,120,110,100,90,80,70,60,50,40,30,20,10,0,        
   
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0   
};
//-----------全彩飘------------//
#define r 0
#define g 1
#define b 2
void quancaipiao(void)
{
  u8 nn,i;
  u8  step_num = SysRun.LedNum;

  u8 temp_memeory[3] = {0, 0, 0};
  
  while(1)
  {
		for(i = 0; i <SysRun.LedNum; i++)
		{ 
			if(step_num < i)
			{
				temp_memeory[r] = 0;
				temp_memeory[g] = 0;
				temp_memeory[b] = 0;
			}
			else
      {
				temp_memeory[r] = r_DataTable[(step_num - i)%63];
				temp_memeory[g] = g_DataTable[(step_num - i)%63];
				temp_memeory[b] = b_DataTable[(step_num - i)%63];
			}
      
			WS2811_SendByte(temp_memeory[r]);
			WS2811_SendByte(temp_memeory[g]);
			WS2811_SendByte(temp_memeory[b]);
//	  transmit_one_point_data(temp_memeory[r], temp_memeory[g], temp_memeory[b]);
		}
		WS2811_Reset();
		os_wait2(K_TMO, 10);

		step_num++;
		if(step_num>253)
		{
			step_num=190;
			nn++;
			if(nn>10)
			{
				 break;
			}	
	  }
  }
}
u8 code R_Data[105] =
{
  250,250,225,200,180,150,125,100,80,50,20,5,3,1,0, //hong    
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,               //lv
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,               //lan                                    
  250,200,150,100,80,50,20,10,5,3,1,0,0,0,0,  //honglv
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,               //lvlan
  250,200,150,100,80,50,20,10,5,3,1,0,0,0,0,  //honglan
  250,200,150,100,80,50,20,10,5,3,1,0,0,0,0   //bai
};
void qicaituowei(void)
{
  u8 nn,i;
  u16  step_num = 150;
  u8 temp_memeory[3] = {0, 0, 0};
  
  while(1)
  {
		for(i = 0; i <150; i++)
		{ 
			if(step_num < i)
			{
				temp_memeory[r] = 0;
				temp_memeory[g] = 0;
				temp_memeory[b] = 0;
			}
			else
      {
				temp_memeory[r] = R_Data[(step_num - i)%15];
				temp_memeory[g] = R_Data[(step_num - i)%105];
				temp_memeory[b] = R_Data[(step_num - i)%105];
			}
			WS2811_SendByte(temp_memeory[r]);
			//WS2811_SendByte(temp_memeory[g]);
		  //WS2811_SendByte(temp_memeory[b]);      
		}
		WS2811_Reset();
		os_wait2(K_TMO, 20);
		step_num++;
		if(step_num>195)
		{	  
			step_num=150;
			nn++;
			if(nn>3)
			{
				 break;
			}
			//	 break;	
  	}
  }
}
u8 code DD_Data[] =
{ 
  250,220,200,170,130,100,70,50,30,15,10,5,1,0,0,
};														
void DDPADD4(void)				//?????????
{
  u8 i,iii,t;
  u8  step_num =100;
  while(1)
  {
		for(i = 0; i <step_num; i++)
		{ 

			for(iii=SysRun.LedNum;iii>0;iii--)
				{	
					//WS2811_SendByte(0);
					//WS2811_SendByte(0);
					for(t=6;t>0;t--)
					{
						WS2811_SendByte(DD_Data[(iii+i)%15]);
					}
				}

			WS2811_Reset();
			os_wait2(K_TMO, 10);     

		}	  
		for(i = 0; i <step_num; i++)
		{ 
			for(iii=0;iii<SysRun.LedNum;iii++)
			{	
//					send_dat(0);
//					send_dat(0);
				for(t=6;t>0;t--)
				{
					WS2811_SendByte(DD_Data[(iii+i)%15]);
				}	
			}

			WS2811_Reset();
			os_wait2(K_TMO, 10);   
		}
		break;	
  }
}
u8 code DDD_Data[] =
{ 
  250,200,150,100,50,15,5,1,0,
};	
void DDPADD5(void)				//?????????
{
  u8 i,iii;
  u8  step_num =54;
  while(1)
  {
		for(i = 0; i <step_num; i++)
		{ 
			for(iii=SysRun.LedNum*3;iii>0;iii--)
			{	
//					send_dat(0);
//					send_dat(0);
				WS2811_SendByte(DDD_Data[(iii+i)%9]);
			}
			WS2811_Reset();
			os_wait2(K_TMO, 10); 
		}	  
//	for(i = 0; i <step_num; i++)
//	{ 
//
//			    for(iii=0;iii<dengzhu;iii++)
//					{	
////					send_dat(0);
////					send_dat(0);
//					send_dat(DDD_Data[(iii+i)%9]);
//					}
//
//		reset();
//		for(t=SHUDU;t>0;t--)  delay(50);      
//		}
	 break;	
  }
}
void ChangeHigh(bit GG1,bit RR1,bit BB1) //纯色渐亮
{  
	u8 n,i,num;//RGB_count=RGB_c;
	for(i=0;i<250;i++)//brightness
	{
		for(n=0;n<SysRun.LedNum;n++)//RGB_count
		{
			WS2811_SendByte(num=GG1?i:0);
			WS2811_SendByte(num=RR1?i:0);
			WS2811_SendByte(num=BB1?i:0);
		}
		WS2811_Reset();
		if(i<50)
			os_wait2(K_TMO, 2);
		else
			os_wait2(K_TMO, 1);
	}
}

void ChangeLose(bit GG1,bit RR1,bit BB1)	//纯色渐暗
{  
	u8 n,i,num;//,RGB_count=RGB_c;
	for(i=250;i>0;i--)
	{
		for(n=SysRun.LedNum;n>0;n--)
		{
			WS2811_SendByte(num=GG1?i:0);
	    WS2811_SendByte(num=RR1?i:0);
	    WS2811_SendByte(num=BB1?i:0);
		}
			WS2811_Reset();
		if(i<50)
			os_wait2(K_TMO, 2);
		else
			os_wait2(K_TMO, 1);
	}	
}
void RgbChange(void)
{
		ChangeHigh(1,0,0);
		ChangeLose(1,0,0);
		ChangeHigh(0,1,0);
		ChangeLose(0,1,0);
		ChangeHigh(0,0,1);
		ChangeLose(0,0,1);
		ChangeHigh(1,1,1);			
		ChangeLose(1,1,1);
		ChangeHigh(1,1,0);			
		ChangeLose(1,1,0);
		ChangeHigh(1,0,1);			
		ChangeLose(1,0,1);
		ChangeHigh(0,1,1);			
		ChangeLose(0,1,1);
}
void led_test()
{
	static i=255;
  while(i>2)
	{
		i=i-5;
		
		LED_SET(BLUE,i);

}
	//LED_SET(BLUE,20);
	//LED_SET(RED,80);
	//LED_SET(GREEN,120);
	//LED_SET(WHITE,250);
	WS2811_Reset();
	os_wait2(K_TMO, 2);
}