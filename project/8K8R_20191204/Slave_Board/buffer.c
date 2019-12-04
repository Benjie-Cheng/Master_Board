

	#define ON       1
	#define OFF      0
	#define ON_ALL   0xff
	#define OFF_ALL  0xfe
	#define OFF_ON   0xfd
	#define KEY1_VAL 0x10
	#define KEY2_VAL 0x11
	#define KEY3_VAL 0x12
	#define KEY4_VAL 0x13
	#define KEY5_VAL 0x14
	#define KEY6_VAL 0x15
	#define KEY7_VAL 0x16
	#define KEY8_VAL 0x17

#define	UART1_TX_LENGTH 10
static u8 Display_Code[1]={0x00};//1个595控制按键板LED灯。
static u8 To_Marster_Data[UART1_TX_LENGTH]={0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff};//主控板继电器开关状态，帧头0x01,帧尾0xff;

void Kled_Set(BOOL en,u8 Kled)
{
	u8 val;
	val = Display_Code[0];
	if(en)
		Display_Code[0] = setbit(val,Kled);//高电平导通
	else
		Display_Code[0] = clrbit(val,Kled);	
	if(ON_ALL == Kled)	
		Display_Code[0] = 0xff;
	else if(OFF_ALL == Kled)
		Display_Code[0] = 0x00;
	else if(ON_OFF == Kled)
		Display_Code[0] = reversebit(val,Kled);//翻转
}

void Gpio_Keyscan(void)	//50ms call
{	
	static unsigned char Su8KeyLock1; //1 号按键的自锁
	static unsigned int Su16KeyCnt1; //1 号按键的计时器
	P10 = 0;
	if(KEY_BOARD_GPIO_Y==0xff)
	{
		Su8KeyLock1=0; //按键解锁
		Su16KeyCnt1=0; //按键去抖动延时计数器清零
		key_led_on(FALSE);//熄灭按键提示led
	}
	else if(0==Su8KeyLock1)
	{
		Su16KeyCnt1++; //累加定时中断次数
		if(Su16KeyCnt1>=KEY_FILTER_TIME) //滤波的“ 稳定时间” KEY_FILTER_TIME， 长度是 25ms。
		{
			Su8KeyLock1=1; //按键的自锁,避免一直触发
			key_led_on(TRUE);//按键提示led
			KeyCode = Get_KeyVal(KEY_BOARD_GPIO_Y);
		}		
	}
}	
int Get_KeyVal(int val)
{
	int temp;
	switch(val){
		case 0xfe:
				temp =0x10;
			break;
		case 0xfd:
				temp =0x11;
				break;
		case 0xfb:
				temp =0x12;
				break;
		case 0xf7:
				temp =0x13;
				break;
		case 0xef:
				temp =0x14;
				break;
		case 0xdf:
				temp =0x15;
				break;
		case 0xbf:
				temp =0x16;
				break;
		case 0x7f:
				temp =0x17;
				break;
		default:
			break;
	}
	return temp;
}
int Get_Led8Set(void)
{
	u8 temp;
	temp = 0x7f&Display_Code[0];
	if(temp == 0x7f)
		return 1;
	else 
		return 0;
	
}
void Date_EventProcess(void)
{	
	u8 i=0;
	for(i=0;i<8;i++)
	{
		if(getbit(Display_Code[0],i))
			To_Marster_Data[i+1] = 0xfe;
		else
			To_Marster_Data[i+1] = 0x00;
	}
	puts_to_SerialPort(To_Marster_Data);
}
void Key_EventProcess(void)
{
	static u8 temp;
	static u8 Key8_status = 0;
	temp = Get_KeyVal(KeyCode);
	switch(temp){
		
	case KEY1_VAL:
			Kled_Set(OFF_ON,0);//翻转状态
		break;
	case KEY2_VAL:
			Kled_Set(OFF_ON,1);
			break;
	case KEY3_VAL:
			Kled_Set(OFF_ON,2);
			break;
	case KEY4_VAL:
			Kled_Set(OFF_ON,3);
			break;
	case KEY5_VAL:
			Kled_Set(OFF_ON,4);
			break;
	case KEY6_VAL:
			Kled_Set(OFF_ON,5);
			break;
	case KEY7_VAL:
			Kled_Set(OFF_ON,6);
			break;
	case KEY8_VAL:
		Key8_status ++;
		if(Display_Code[0]==0xff){
			Kled_Set(OFF_ALL,7);//全灭
			Key8_status = 0;
		}
		else if(mod(Key8_status,2))
		{
			Kled_Set(ON_ALL,7);//全亮
		}
		else
		{
			Kled_Set(OFF_ALL,7);//全灭
		}
		if(Key8_status >=2)
			Key8_status = 0;
		break;
	default:
	
		break;
		
		if(Get_Led8Set())//1-7全亮时,8亮
		{
			Kled_Set(ON,7);//8灯亮
		}	
		else
		{
			Kled_Set(OFF,7);//8灯灭
		}
	}
	if(KeyCode){
		vDataIn595(Display_Code[0]);
		vDataOut595();
		Date_EventProcess();//按键值转换成发给主机格式的值并串口发送;
		KeyCode = 0;//清除按键触发值
	}
}