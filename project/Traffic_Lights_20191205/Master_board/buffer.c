

	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd
	#define RED_LED     2
	#define YELLOW_LED  1
	#define GREEN_LED   0
	#define CAR_LED     3

#define GPIO_CHECK_PIN P20 

//“软件定时器 1” 的相关变量
volatile unsigned char vGu8TimeFlag_1=0;
volatile unsigned int vGu16TimeCnt_1=0;	
volatile unsigned char vGu8TimeFlag_2=0;
volatile unsigned int vGu16TimeCnt_2=0;	
volatile unsigned char vGu8LeveCheckFlag=0;	
BOOL flash_flag = TRUE;

#define GPIO_FILTER_TIME 50 //滤波的“ 稳定时间” 50ms

#define LED_TIME_60S 60000 //时间是 60000ms
#define LED_TIME_65S 65000 //时间是 65000ms
#define LED_TIME_95S 95000 //时间是 95000ms

static u8 Gu8Step = 0; //软件定时器 1 的 switch 切换步骤

void Kled_Set(int status,u8 Nled)
{
	u8 val;
	val = Display_Code[0];
	if(status==ON)
		Display_Code[0] = setbit(val,Nled);//高电平导通
	else if(status==OFF)
		Display_Code[0] = clrbit(val,Nled);	
	else if(ON_ALL == status)	
		Display_Code[0] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[0] = 0x00;
	else if(OFF_ON == status)
		Display_Code[0] = reversebit(val,Nled);//翻转
}
void Gpio_ValRead(void)	
{	
	static unsigned char Su8KeyLock1; //1 号按键的自锁

	if(GPIO_CHECK_PIN==1)
	{
		Su8KeyLock1=0; //按键解锁
		vGu8TimeFlag_2=0;
		vGu16TimeCnt_2=0;
		flash_flag = TRUE;
	}
	else if(0==Su8KeyLock1)
	{
		vGu8TimeFlag_2=1;//启动定时器2
		if(vGu16TimeCnt_2>=GPIO_FILTER_TIME) //滤波的“ 稳定时间” GPIO_FILTER_TIME， 长度是 50ms。
		{
			vGu8TimeFlag_2=0;//滤波时间到，定时器请0
			Su8KeyLock1=1; //按键的自锁,避免一直触发
			Kled_Set(ON,CAR_LED);//汽车停止
			key_led_on(TRUE);//按键提示led
			flash_flag = FALSE;
		}		
	}
}	
void Traffic_Led(void)
{
	switch(Gu8Step)
	{
		case 0:
			vGu8TimeFlag_1 = 1;
			Kled_Set(ON,GREEN_LED);//绿灯亮
			Kled_Set(OFF,RED_LED);//红灯灭
			vGu8LeveCheckFlag = 0;//关闭电平检测
			if(vGu16TimeCnt_1>=LED_TIME_60S) //60s时间到
				Gu8Step++;
			break;
		case 1:
			Kled_Set(OFF,GREEN_LED);//绿灯灭
			Kled_Set(ON,YELLOW_LED);//黄灯亮
			vGu8LeveCheckFlag = 1;//启动电平检测
			if(vGu16TimeCnt_1>=LED_TIME_65S) //65s时间到
				Gu8Step++;
			break;
		case 2:
			Kled_Set(OFF,GREEN_LED);//绿灯灭
			Kled_Set(OFF,YELLOW_LED);//黄灯灭
			Kled_Set(ON,RED_LED);//红灯亮
			if(vGu16TimeCnt_1>=LED_TIME_95S) //95s时间到
			{
				Gu8Step=0;		
				vGu16TimeCnt_1 = 0;
				vGu8TimeFlag_1 = 0;
			}
			break;	
		default:	
			break;
	}
}

void main(void)
{
	stc15x_hw_init();
	vDataIn595(0x00);
	vDataOut595();	//开机默认关闭通道显示LED
	puts_to_SerialPort("I am Traffic Lights!\n");
	while (1)
	{
		if(B_1ms)	//1ms到
		{
			B_1ms = 0;	
			if(++msecond >= 1000)	//1秒到
			{	
				msecond = 0;
				if(flash_flag)
					key_led_reverse();
			}
			Traffic_Led();//指示灯状态切换
			if(vGu8LeveCheckFlag)
				Gpio_ValRead();//低电平检测
			else
			{
				vGu8TimeFlag_2=0;//不检测，定时器请0
				Kled_Set(OFF,CAR_LED);//汽车运行
			}
			
		}
	}
}
//========================================================================
// 描述: Timer0 1ms中断函数。
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	B_1ms = 1;		//1ms标志
	if(vGu8TimeFlag_1)
		vGu16TimeCnt_1++;
	else
		vGu16TimeCnt_1 = 0;
	if(vGu8TimeFlag_2)
		vGu16TimeCnt_2++;
}