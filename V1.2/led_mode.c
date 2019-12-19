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
595控制口
sbit MDA 	=P2^5;//P2^0;//595-14 DSB1
sbit MS_CLK =P2^2;//P2^3;//595-11 SH2
sbit MR_CLK =P2^3;//P2^2;//595-12 ST2
sbit MG_595 =P2^4;//P2^1;//595-13 OE2

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
void HC595RCLK(void);//数据锁存
void LEDOFF(unsigned char Num);//关闭N个灯
void LEDON(unsigned char Num);//打开N个灯


void HC595RCLK(void)//数据锁存
{
	MR_CLK=0; //set dataline low
	_nop_();//NOP();
	_nop_();//NOP();
	MR_CLK=1;
}
void LEDOFF(unsigned char Num)//关闭N个灯
{
	unsigned char i;
	for(i=0;i<Num;i++)
	{
		MDA=0;//,BDA=0;
		MS_CLK=0;
		NOP();
		NOP();
		MS_CLK=1;
	}
}
void LEDON(unsigned char Num)//打开N个灯
{
  unsigned char i;

	for(i=0;i<Num;i++)
	{
		MDA=1;//,BDA=1 ;
	    MS_CLK=0;
	    NOP();
	    NOP();
	    MS_CLK=1;
	}
}

//1---------->2 逐个点亮
//1---------->2 逐个熄灭
//1---------->2 1010 逐渐扩展
//1---------->2 逐个熄灭

void LEDF1(void)	//流水向上和向下
{
	unsigned char t=0,t1=0,tt=2;	
	LEDOFF(NUM);
	for(t=0;t<NUM;t++)
	{
		LEDON(1);
		HC595RCLK();
		delayms(125*NUMM);
	}
	for(t=0;t<NUM;t++)
	{
		LEDOFF(1);
		HC595RCLK();
		delayms(100*NUMM);
	}
	for(t=0;t<NUM;t++)
	{
		LEDON(t+1);
		LEDOFF(NUM-t-1);
		HC595RCLK();
		delayms(125*NUMM);
	}
	for(t=0;t<NUM;t++)
	{
		LEDOFF(1);
		HC595RCLK();
		delayms(100*NUMM);
	}			   
}
//点亮第一个，逐次将第一个推向最后，循环三次。------单个灯向前跑 1----L-->2
//点亮最后一个，后面全部熄灭 ，循环三次。     ------单个灯向后跑 1<---L--2         
void LEDF2(void) //扫描向上和向下
{
	unsigned char t=0,t1=0,tt=2;
	LEDON(1);
	HC595RCLK();
	delayms(125*NUMM);	
	for(t=1;t<NUM;t++)
	{
		LEDOFF(1);
		HC595RCLK();
		delayms(125*NUMM);
	}
	LEDON(1);
	HC595RCLK();
	delayms(125*NUMM);	
	for(t=1;t<NUM;t++)
	{
		LEDOFF(1);
		HC595RCLK();
		delayms(125*NUMM);
	}
	LEDON(1);
	HC595RCLK();
	delayms(125*NUMM);	
	for(t=1;t<NUM;t++)
	{
		LEDOFF(1);
		HC595RCLK();
		delayms(125*NUMM);
	}
		
	for(t=0;t<NUM;t++)
	{
		LEDOFF(t);
		LEDON(1);
		LEDOFF(NUM-t-1);
		HC595RCLK();
		delayms(100*NUMM);
	}	
	for(t=0;t<NUM;t++)
	{
		LEDOFF(t);
		LEDON(1);
		LEDOFF(NUM-t-1);
		HC595RCLK();
		delayms(100*NUMM);
	}	
	for(t=0;t<NUM;t++)
	{
		LEDOFF(t);
		LEDON(1);
		LEDOFF(NUM-t-1);
		HC595RCLK();
		delayms(100*NUMM);
	}		
}

//1--HH------------------->2 两个灯向前跑
//1--LL--HH--------------->2 两个灯向前跑
//1--HH--LL--HH----------->2 两个灯向前跑
void LEDF3(void)//花样流水
{
	unsigned char t=0,t1=0,tt=2;
	LEDOFF(NUM);
	for(t=0;t<NUM/4+3;t++)
	{
		LEDON(1);
		HC595RCLK();
		delayms(1000*NUMM);
		LEDON(1);
		HC595RCLK();
		delayms(1000*NUMM);
		LEDOFF(1);
		HC595RCLK();
		delayms(1000*NUMM);
		LEDOFF(1);
		HC595RCLK();
		delayms(1000*NUMM);
	}
} 
//1--HH------------------->2 两个灯向前跑
//1--LL--HH--------------->2 两个灯向前跑
//1--HH--LL--HH----------->2 两个灯向前跑
void LEDF4(void) //后浪推前浪
{
	unsigned char t=0,t1=0,tt=2;
	LEDOFF(NUM);
	for(t=0;t<(NUM/3+1);t++)
	{
		for(t1=0;t1<3;t1++)
		{
			LEDOFF(NUM-t*3-t1);
			LEDON(1+t1);
			LEDOFF(t*3);
			HC595RCLK();
			delayms(50*NUMM);
		}
		 	delayms(500*NUMM);
		for(t1=0;t1<3;t1++)
		{
			LEDOFF(NUM-3);
			LEDON(3-t1-1);
			LEDOFF(t1+1+t*3);
			HC595RCLK();
			delayms(50*NUMM);
		}
	}
	LEDOFF(NUM);
}
//1H--------->2<----------H3 两边向中间跑
//1HH-------->2<---------HH3 逐个个增加
//1HHH------->2<--------HHH3 逐个个增加
//1HHHHHHHHHH>2<HHHHHHHHHHH3 全两
void LEDF5(void) //上下两边向中间亮
{
	unsigned char t=0,t1=0,tt=2;
	LEDOFF(NUM);
	HC595RCLK();
	delayms(125*NUMM);
	for(t1=1;t1<=NUM/2;t1++)
	{
		LEDON(t1);
	
		LEDOFF(NUM-t1*2);
		LEDON(t1);
		HC595RCLK();
		delayms(125*NUMM);
	}
	LEDON(NUM);
	HC595RCLK();
	delayms(125*NUMM);		 
}
//1HHHHHHHHHH>2<HHHHHHHHHHH3 全两
//1HHHHHHHHH-<2>-HHHHHHHHHH3 向两边灭
//1H---------<2>----------H3 向两边灭
void LEDF6(void)//中间向上下两边后灭
{
	unsigned char t=0,t1=0,tt=2;
	LEDON(NUM);
	HC595RCLK();
	delayms(125*NUMM);
	for(t1=0;t1<=NUM/2;t1++)
	{
		LEDON(NUM/2-t1);
	   	if(NUM%2)
			LEDOFF(t1*2+1);
		else
			LEDOFF(t1*2);
		//LEDOFF(t1*2+1);
		LEDON(NUM/2-t1);
		HC595RCLK();
		delayms(125*NUMM);
	}
 		LEDOFF(NUM);
		HC595RCLK();
		delayms(125*NUMM);

	for(t1=0;t1<NUM/2;t1++)
	{
		LEDOFF(NUM/2-t1);
	   	if(NUM%2)
			LEDON(t1*2+1);
		else
			LEDON(t1*2);
		LEDOFF(NUM/2-t1);
		HC595RCLK();
		delayms(125*NUMM);
	}
}
void LEDF7(void)//上下两边向中间灭
{
	unsigned char t=0,t1=0,tt=2;

	for(t1=0;t1<=NUM/2;t1++)
	{
		LEDOFF(t1);
		LEDON(NUM-t1*2);;
		LEDOFF(t1);
		HC595RCLK();
		delayms(125*NUMM);
	}
	if(NUM%2)
	{
		LEDOFF(NUM);
		HC595RCLK();
		delayms(125*NUMM);
	}
}
void LEDF8(void)//中间向上下两边先亮后灭
{
	unsigned char t=0,t1=0,tt=2;

		LEDOFF(NUM);
		HC595RCLK();
		delayms(125*NUMM);
	for(t1=0;t1<=NUM/2;t1++)
	{
		LEDOFF(NUM/2-t1);
		if(NUM%2)
		LEDON(t1*2+1);
		else
		LEDON(t1*2);
		//LEDON(t1*2+1);
		LEDOFF(NUM/2-t1);
		HC595RCLK();
		delayms(125*NUMM);
	}

		LEDON(NUM);
		HC595RCLK();
		delayms(125*NUMM);

	for(t1=0;t1<=NUM/2;t1++)
	{
		LEDON(NUM/2-t1);
		if(NUM%2)
		LEDOFF(t1*2+1);
		else
		//LEDOFF(t1*2);
		LEDOFF(t1*2+1);
		LEDON(NUM/2-t1);
		HC595RCLK();
		delayms(125*NUMM);
	}
}
void LEDF9(void)//上下两边向中间先亮
{
	unsigned char t=0,t1=0,tt=2;

	
	for(t1=1;t1<=NUM/2;t1++)
	{
		LEDON(t1);
		LEDOFF(NUM-t1*2);
		LEDON(t1);
		HC595RCLK();
		delayms(125*NUMM);
	} 
	if (NUM%2)
	{
		LEDON(NUM);
		HC595RCLK();
		delayms(125*NUMM);
	}

}
void LEDF10(void)//中间向上下两边先灭后亮
{
	unsigned char t=0,t1=0,tt=2;
	    LEDON(NUM );
	  	HC595RCLK();
		delayms(125*NUMM);
	for(t1=0;t1<=NUM/2;t1++)
	{
		LEDON(NUM/2-t1);
		if(NUM%2)
		LEDOFF(t1*2+1);
		else
		LEDOFF(t1*2);

		LEDON(NUM/2-t1);
		HC595RCLK();
		delayms(125*NUMM);
	}
	for(t1=0;t1<=NUM/2;t1++)
	{
		LEDOFF(NUM/2-t1);
		if(NUM%2)
		LEDON(t1*2+1);
		else
		LEDON(t1*2);
		//LEDON(t1*2+1);
		LEDOFF(NUM/2-t1);
		HC595RCLK();
		delayms(125*NUMM);
	}
}
void LEDF11(void)//向上流水+叠加	
{
	unsigned char t=0,t1=0,tt=2;
	if(OPEN1)
	{
		LEDOFF(NUM);
		for(t=0;t<NUM;t++)
		{
			for(t1=0;t1<NUM-t;t1++)
			{
				LEDON(t+1);
				LEDOFF(NUM-t-t1-1);
				LEDON(1);
				LEDOFF(t1);
				HC595RCLK();
				delayms(125*NUMM);
			}
		}
	}
}

