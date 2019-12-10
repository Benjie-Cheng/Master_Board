

	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd
	#define RED_LED     2
	#define YELLOW_LED  1
	#define GREEN_LED   0
	#define CAR_LED     3
	
	#define RUN_STOP 0xff
	#define RUN_START 0x01





#define KEY_FILTER_TIME 20 //滤波的“ 稳定时间” 20ms

#define KEY_INPUT1=P20;  //【启动】按键K1的输入口。
#define KEY_INPUT2=P21;  //【停止】按键K2的输入口。
#define KEY_INPUT3=P22;  //【模式+】按键K3的输入口。
#define KEY_INPUT4=P23;  //【模式-】按键K4的输入口。
BOOL update_flag = TRUE;
BOOL Key_EventProtect = FALSE;

#define LED_TIME_60S 60000 //时间是 60000ms
#define LED_TIME_65S 65000 //时间是 65000ms
#define LED_TIME_95S 95000 //时间是 95000ms

volatile unsigned char vGu8KeySec=0;  //按键的触发序号
static u8 Gu8Step = 1; //switch 切换步骤
u16 Bit8_ToBit16(u8 high,u8 low)
{
	u16 val;
	val = (high << 8) | low;;
	return val;
}
u8 Bit16_ToBit8(u16 num,u8 high)
{
	u16 val;
	if(high)
		val = (num >> 8) & 0xff;//高8位
	else
		val = num & 0xff; //低8位
	return val;
}
void Kled_Set(int status,u8 Nled)
{
	u16 val;
	val = Bit8_ToBit16(Display_Code[0],Display_Code[1]);
	if(status==ON)
		setbit(val,Nled);//高电平导通
	else if(status==OFF)
		clrbit(val,Nled);	
	else if(ON_ALL == status)	
		val = 0xffff;
	else if(OFF_ALL == status)
		val= 0x0000;
	else if(OFF_ON == status)
		reversebit(val,Nled);//翻转
	Display_Code[0] = Bit16_ToBit8(val,1);//高8位
	Display_Code[1] = Bit16_ToBit8(val,0);//低8位
}
void KeyScan(void); 
{
	static unsigned char Su8KeyLock1;
	static unsigned int  Su16KeyCnt1;
	static unsigned char Su8KeyLock2;
	static unsigned int  Su16KeyCnt2; 	
	static unsigned char Su8KeyLock3;
	static unsigned int  Su16KeyCnt3; 
	static unsigned char Su8KeyLock4;
	static unsigned int  Su16KeyCnt4; 
	if(Key_EventProtect)
		return;
    //【启动】按键K1的扫描识别
	if(0!=KEY_INPUT1)
	{
		Su8KeyLock1=0;
		Su16KeyCnt1=0;   
	}
	else if(0==Su8KeyLock1)
	{
		Su16KeyCnt1++;
		if(Su16KeyCnt1>=KEY_FILTER_TIME)
		{
			Su8KeyLock1=1;  
			vGu8KeySec=1;    //触发1号键
			Key_EventProtect = TRUE;
		}
	}
   //【停止】按键K2的扫描识别
	if(0!=KEY_INPUT2)
	{
		Su8KeyLock2=0;
		Su16KeyCnt2=0;      
	}
	else if(0==Su8KeyLock2)
	{
		Su16KeyCnt2++;
		if(Su16KeyCnt2>=KEY_FILTER_TIME)
		{
			Su8KeyLock2=1;  
			vGu8KeySec=2;    //触发2号键
			Key_EventProtect = TRUE;
		}
	}
   //【模式+】按键K3的扫描识别
	if(0!=KEY_INPUT3)
	{
		Su8KeyLock3=0;
		Su16KeyCnt3=0;      
	}
	else if(0==Su8KeyLock3)
	{
		Su16KeyCnt3++;
		if(Su16KeyCnt3>=KEY_FILTER_TIME)
		{
			Su8KeyLock3=1;  
			vGu8KeySec=3;    //触发3号键
			Key_EventProtect = TRUE;
		}
	}
   //【模式-】按键K4的扫描识别
	if(0!=KEY_INPUT4)
	{
		Su8KeyLock4=0;
		Su16KeyCnt4=0;      
	}
	else if(0==Su8KeyLock4)
	{
		Su16KeyCnt4++;
		if(Su16KeyCnt4>=KEY_FILTER_TIME)
		{
			Su8KeyLock4=1;  
			vGu8KeySec=4;    //触发4号键
			Key_EventProtect = TRUE;
		}
	}
}
void KeyTask(void);
{
	if(0==vGu8KeySec)
	{
		return; //按键的触发序号是0意味着无按键触发，不执行此函数下面的代码
	}
	
	switch(vGu8KeySec) //根据不同的按键触发序号执行对应的代码
	{
		case 1:     //1号按键。【启动】按键K1
			Gu8Step = RUN_START;
			vGu8KeySec=0; 
		break;
		case 2:     //2号按键。【停止】按键K2
			Gu8Step = RUN_STOP;
			vGu8KeySec=0;  
		break;
		case 3:     //3号按键。【模式+】按键K3
			Gu8Step++;
			vGu8KeySec=0;  
		break;
		case 4:     //4号按键。【模式-】按键K4
			Gu8Step--;
			vGu8KeySec=0; 
		break;
		default:     
			 
		break;
	}
	if(Gu8Step >16 && (RUN_START != Gu8Step))
		Gu8Step = 1;
	else if(Gu8Step < 1)
		Gu8Step = 16;
	update_flag = TRUE;
	Key_EventProtect = FALSE;
	
}	
void Led_StateUpdate(void)
{
	if(!update_flag)
		return ;
	Kled_Set(OFF_ALL,1);//发生改变时，先清0一次;
	switch(Gu8Step)
	{
		case 1:
			Kled_Set(ON,0);
			Kled_Set(ON,2);
		break;
		case 2:
			Kled_Set(ON,4);
		break;
		case 3:
			Kled_Set(ON,1);
			Kled_Set(ON,3);
		break;	
		case 4:
			Kled_Set(ON,5);
		break;
		case 5:
			Kled_Set(ON,0);
			Kled_Set(ON,2);
		break;
		case 6:
			Kled_Set(ON,6);
		break;
		case 7:
			Kled_Set(ON,1);
			Kled_Set(ON,3);
		break;
		case 8:
			Kled_Set(ON,5);
		break;
		case 9:
			Kled_Set(ON,0);
			Kled_Set(ON,7);
		break;
		case 10:
			Kled_Set(ON,9);
		break;
		case 11:
			Kled_Set(ON,1);
			Kled_Set(ON,8);
		break;
		case 12:
			Kled_Set(ON,10);
		break;
		case 13:
			Kled_Set(ON,0);
			Kled_Set(ON,7);
		break;
		case 14:
			Kled_Set(ON,11);
		break;
		case 15:
			Kled_Set(ON,0);
			Kled_Set(ON,8);
		break;
		case 16:
			Kled_Set(ON,10);
		break;
		
		default:
		break;
		update_flag = FALSE;//更新后及时置位0;
	}
}
void main(void)
{
	stc15x_hw_init();
	vDataIn595(0x00);
	vDataOut595();	//开机默认关闭通道显示LED
	puts_to_SerialPort("I am LED controller XXX_0003_20191210!\n");
	puts_to_SerialPort("Contact me: 15901856750\n");
	while (1)
	{	
		 KeyTask();    //按键的任务函数
		 Led_StateUpdate();//LED状态更新		 
	}
}
//========================================================================
// 描述: Timer0 1ms中断函数。
//========================================================================
void timer0 (void) interrupt TIMER0_VECTORKeyScan
{
	B_1ms = 1;		//1ms标志
	KeyScan();
}