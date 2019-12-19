//建筑模型广场灯光效果选择模式调节速度程序
/*************	本程序功能说明	**************
74HC595控制建筑模型广场灯。
P1端口为短接口，选择流水灯模式
sbit OPEN0 = P1^0;
sbit OPEN1 = P1^1;
sbit OPEN2 = P1^2;
sbit OPEN3 = P1^3;
sbit OPEN4 = P1^4;
sbit OPEN5 = P1^5;
sbit OPEN6 = P1^6;
sbit OPEN7 = P1^7;
*/
/********************** A 给指示灯用的595 ************************/
sbit	A_HC595_SER   = P3^4;	//pin 34	SER		data input
sbit	A_HC595_RCLK  = P3^7;	//pin 37	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P3^5;	//pin 35	SRCLK	Shift data clock
sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		低电平 使能enable pin
sbit	A_HC595_MR    = P3^6;	//pin 36	低电平复位	
/*
特点：将595看成位操作，不做8位处理。

1、本机串口接收从机8*8矩阵键盘按键码控制继电器。 
	帧头+指令+数据长度+随机数+按键码+帧尾(暂未定义) //按键码需要解密
2、蓝牙接收数据。串口1，中断方式,115200
3、wifi接收数据。
4、无线接收数据。
5、控制继电器。

存在缺陷：
1、模拟串口uart3收发数据不稳定，优化项
2、1中问题已经使用定时器0模拟多路串口接收解决

V1.1记录：
1、修改蓝牙握手成功提示方法。                         20190930
2、增加按键解析，控制595的开关，并将通道值发给按键板  20190930
V1.2记录：
1、蓝牙协议修改成适配APP方式，其他通信接口暂时不变。  20191008
2、内部存储4K SRAM 暂时未调通。                       20191008
待调试：
1、内部EEPROM 暂时未调通。                            20191008
2、2000路595电路。                                    20191008
******************************************/
void vDataIn595(u8 dat);
void vDataOut595(void);//数据锁存
void LEDOFF(unsigned char Num);//关闭N个灯
void LEDON(unsigned char Num);//打开N个灯
bool B_1ms;	 	//1ms标志
bool Exit_Flag = 0; 
#define EXIT_EVENT()      {if(Exit_Flag) break;}
#define DISPLAY_CODE()    {vDataIn595(Display_Code[0]); vDataIn595(Display_Code[1]);}
#define TIME_OUT_1MS()    {while(!B_1ms){EXIT_EVENT();}}

#define LED_SPEED  0xff //led流水速度
#define LED_NUM    0xff //led数量

int Led_Speed = LED_SPEED;
int Led_Num =  LED_NUM;

void delayms(unsigned int ms) // 延时子程序
{      
	unsigned int i;
	for(i = 0; i < ms; i++);   //1ms定时时间到
	{
		TIME_OUT_1MS();
		EXIT_EVENT();//如果有退出标志，将退出。
	}
}
/**************** HC595数据锁存函数 ******************/
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
void vDataOut595(void)//数据锁存
{
	DISPLAY_CODE();//加上固定显示区;
	A_HC595_RCLK = 0;
	NOP2();
	A_HC595_RCLK = 1;
}
void LEDOFF(unsigned char Num)//关闭N个灯
{
	unsigned char i;
	for(i=0;i<Num;i++)
	{
		A_HC595_SER=0;//,BDA=0;
		A_HC595_SRCLK=0;
		NOP2();
		A_HC595_SRCLK=1;
	}
}
void LEDON(unsigned char Num)//打开N个灯
{
  unsigned char i;

	for(i=0;i<Num;i++)
	{
		A_HC595_SER=1;//,BDA=1 ;
	    A_HC595_SRCLK=0;
	    NOP2();
	    A_HC595_SRCLK=1;
	}
}

//1---------->2 逐个点亮
//1---------->2 逐个熄灭
//1---------->2 1010 逐渐扩展
//1---------->2 逐个熄灭

void LEDF1(void)	//流水向上和向下
{
	unsigned char t=0,t1=0,tt=2;	
	LEDOFF(Led_Num);
	for(t=0;t<Led_Num;t++)
	{
		LEDON(1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
	for(t=0;t<Led_Num;t++)
	{
		LEDOFF(1);
		vDataOut595();
		delayms(100*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
	for(t=0;t<Led_Num;t++)
	{
		LEDON(t+1);
		LEDOFF(Led_Num-t-1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
	for(t=0;t<Led_Num;t++)
	{
		LEDOFF(1);
		vDataOut595();
		delayms(100*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}			   
}
//点亮第一个，逐次将第一个推向最后，循环三次。------单个灯向前跑 1----L-->2
//点亮最后一个，后面全部熄灭 ，循环三次。     ------单个灯向后跑 1<---L--2         
void LEDF2(void) //扫描向上和向下
{
	unsigned char t=0,t1=0,tt=2;
	LEDON(1);
	vDataOut595();
	delayms(125*Led_Speed);	
	for(t=1;t<Led_Num;t++)
	{
		LEDOFF(1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
	LEDON(1);
	vDataOut595();
	delayms(125*Led_Speed);	
	for(t=1;t<Led_Num;t++)
	{
		LEDOFF(1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
	LEDON(1);
	vDataOut595();
	delayms(125*Led_Speed);	
	for(t=1;t<Led_Num;t++)
	{
		LEDOFF(1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
		
	for(t=0;t<Led_Num;t++)
	{
		LEDOFF(t);
		LEDON(1);
		LEDOFF(Led_Num-t-1);
		vDataOut595();
		delayms(100*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}	
	for(t=0;t<Led_Num;t++)
	{
		LEDOFF(t);
		LEDON(1);
		LEDOFF(Led_Num-t-1);
		vDataOut595();
		delayms(100*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}	
	for(t=0;t<Led_Num;t++)
	{
		LEDOFF(t);
		LEDON(1);
		LEDOFF(Led_Num-t-1);
		vDataOut595();
		delayms(100*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}		
}

//1--HH------------------->2 两个灯向前跑
//1--LL--HH--------------->2 两个灯向前跑
//1--HH--LL--HH----------->2 两个灯向前跑
void LEDF3(void)//花样流水
{
	unsigned char t=0,t1=0,tt=2;
	LEDOFF(Led_Num);
	for(t=0;t<Led_Num/4+3;t++)
	{
		LEDON(1);
		vDataOut595();
		delayms(1000*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
		LEDON(1);
		vDataOut595();
		delayms(1000*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
		LEDOFF(1);
		vDataOut595();
		delayms(1000*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
		LEDOFF(1);
		vDataOut595();
		delayms(1000*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
} 
//1--HH------------------->2 两个灯向前跑
//1--LL--HH--------------->2 两个灯向前跑
//1--HH--LL--HH----------->2 两个灯向前跑
void LEDF4(void) //后浪推前浪
{
	unsigned char t=0,t1=0,tt=2;
	LEDOFF(Led_Num);
	for(t=0;t<(Led_Num/3+1);t++)
	{
		for(t1=0;t1<3;t1++)
		{
			LEDOFF(Led_Num-t*3-t1);
			LEDON(1+t1);
			LEDOFF(t*3);
			vDataOut595();
			delayms(50*Led_Speed);
			EXIT_EVENT();//如果有退出标志，将退出。
		}
		 	delayms(500*Led_Speed);
		for(t1=0;t1<3;t1++)
		{
			LEDOFF(Led_Num-3);
			LEDON(3-t1-1);
			LEDOFF(t1+1+t*3);
			vDataOut595();
			delayms(50*Led_Speed);
			EXIT_EVENT();//如果有退出标志，将退出。
		}
	}
	LEDOFF(Led_Num);
}
//1H--------->2<----------H3 两边向中间跑
//1HH-------->2<---------HH3 逐个个增加
//1HHH------->2<--------HHH3 逐个个增加
//1HHHHHHHHHH>2<HHHHHHHHHHH3 全两
void LEDF5(void) //上下两边向中间亮
{
	unsigned char t=0,t1=0,tt=2;
	LEDOFF(Led_Num);
	vDataOut595();
	delayms(125*Led_Speed);
	EXIT_EVENT();//如果有退出标志，将退出。
	for(t1=1;t1<=Led_Num/2;t1++)
	{
		LEDON(t1);
	
		LEDOFF(Led_Num-t1*2);
		LEDON(t1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
	LEDON(Led_Num);
	vDataOut595();
	delayms(125*Led_Speed);		
}
//1HHHHHHHHHH>2<HHHHHHHHHHH3 全两
//1HHHHHHHHH-<2>-HHHHHHHHHH3 向两边灭
//1H---------<2>----------H3 向两边灭
void LEDF6(void)//中间向上下两边后灭
{
	unsigned char t=0,t1=0,tt=2;
	LEDON(Led_Num);
	vDataOut595();
	delayms(125*Led_Speed);
	EXIT_EVENT();//如果有退出标志，将退出。
	for(t1=0;t1<=Led_Num/2;t1++)
	{
		LEDON(Led_Num/2-t1);
	   	if(Led_Num%2)
			LEDOFF(t1*2+1);
		else
			LEDOFF(t1*2);
		//LEDOFF(t1*2+1);
		LEDON(Led_Num/2-t1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
 		LEDOFF(Led_Num);
		vDataOut595();
		delayms(125*Led_Speed);

	for(t1=0;t1<Led_Num/2;t1++)
	{
		LEDOFF(Led_Num/2-t1);
	   	if(Led_Num%2)
			LEDON(t1*2+1);
		else
			LEDON(t1*2);
		LEDOFF(Led_Num/2-t1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
}
void LEDF7(void)//上下两边向中间灭
{
	unsigned char t=0,t1=0,tt=2;

	for(t1=0;t1<=Led_Num/2;t1++)
	{
		LEDOFF(t1);
		LEDON(Led_Num-t1*2);;
		LEDOFF(t1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
	if(Led_Num%2)
	{
		LEDOFF(Led_Num);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
}
void LEDF8(void)//中间向上下两边先亮后灭
{
	unsigned char t=0,t1=0,tt=2;

		LEDOFF(Led_Num);
		vDataOut595();
		delayms(125*Led_Speed);
	for(t1=0;t1<=Led_Num/2;t1++)
	{
		LEDOFF(Led_Num/2-t1);
		if(Led_Num%2)
		LEDON(t1*2+1);
		else
		LEDON(t1*2);
		//LEDON(t1*2+1);
		LEDOFF(Led_Num/2-t1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}

		LEDON(Led_Num);
		vDataOut595();
		delayms(125*Led_Speed);

	for(t1=0;t1<=Led_Num/2;t1++)
	{
		LEDON(Led_Num/2-t1);
		if(Led_Num%2)
		LEDOFF(t1*2+1);
		else
		//LEDOFF(t1*2);
		LEDOFF(t1*2+1);
		LEDON(Led_Num/2-t1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
}
void LEDF9(void)//上下两边向中间先亮
{
	unsigned char t=0,t1=0,tt=2;

	
	for(t1=1;t1<=Led_Num/2;t1++)
	{
		LEDON(t1);
		LEDOFF(Led_Num-t1*2);
		LEDON(t1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	} 
	if (Led_Num%2)
	{
		LEDON(Led_Num);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}

}
void LEDF10(void)//中间向上下两边先灭后亮
{
	unsigned char t=0,t1=0,tt=2;
	    LEDON(Led_Num );
	  	vDataOut595();
		delayms(125*Led_Speed);
	for(t1=0;t1<=Led_Num/2;t1++)
	{
		LEDON(Led_Num/2-t1);
		if(Led_Num%2)
		LEDOFF(t1*2+1);
		else
		LEDOFF(t1*2);

		LEDON(Led_Num/2-t1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
	for(t1=0;t1<=Led_Num/2;t1++)
	{
		LEDOFF(Led_Num/2-t1);
		if(Led_Num%2)
		LEDON(t1*2+1);
		else
		LEDON(t1*2);
		//LEDON(t1*2+1);
		LEDOFF(Led_Num/2-t1);
		vDataOut595();
		delayms(125*Led_Speed);
		EXIT_EVENT();//如果有退出标志，将退出。
	}
}
void LEDF11(void)//向上流水+叠加	
{
	unsigned char t=0,t1=0,tt=2;
	if(OPEN1)
	{
		LEDOFF(Led_Num);
		for(t=0;t<Led_Num;t++)
		{
			for(t1=0;t1<Led_Num-t;t1++)
			{
				LEDON(t+1);
				LEDOFF(Led_Num-t-t1-1);
				LEDON(1);
				LEDOFF(t1);
				vDataOut595();
				delayms(125*Led_Speed);
				EXIT_EVENT();//如果有退出标志，将退出。
			}
			EXIT_EVENT();//如果有退出标志，将退出。
		}
	}
}

