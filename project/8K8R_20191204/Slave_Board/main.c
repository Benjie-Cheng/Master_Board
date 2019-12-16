/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/
/*************	��������˵��	**************
8K8R:8��key����8��Relay�����幦�ܼ�spec

��STC��MCU��IO��ʽ������̽����ʹ����շ����ݡ�
1��8*8 �������ʱֻ���Բ�����һ�С�
2���������а�������ʱ�����ڷ���K1-K8�������Ƶ�led ״̬��������������ʱ�޷���״̬���ӻ�������ʱ״̬����ң����Ǵ��°�һ���ָܻ�����
3���ӻ����ݰ���ֵ��ͨ��74HC595���ư���LED״̬��
4���а�������ʱ��һ�Ź��õ�LEDָʾ���������𣬲���ʱ����1Hz��˸��

����ȱ�ݣ�
1��8*8���̵�LED���ڰ����Ҳ࣬ÿ�ΰ��¾�����ָ�ڵ������Էֱ�̵���״̬�������Ҳ�����ϲࡣ
2��LED������Ϊ�ߵ�ƽ��������������������ס���ɲ����ⲿ�������͵�ƽ������ʽ

v1.1��
1���޸Ĳ�����115200->9600           20190929
2��ͳһͨ�����ݸ�ʽ                 20190929
******************************************/
#include "config.h"
#include "debug.h"

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 �ж�Ƶ��, 1000��/��
#define	Baudrate1	115200UL                                //ͨ�Ų�����115200

#define KEY_LED_GPIO P55
#define KEY_BOARD_GPIO_Y P2
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

#define CMD_ON_OFF_ALL 1
u8 twice_cmd = 0;
	
//========================================================================
/*************	���ر�������	**************/
/********************** ������ ************************/
BOOL B_1ms;	 //1ms��־
u16	msecond; //1s����
u8 cnt50ms;  //50ms����
u8 cnt10ms;  //10ms����
BOOL B_TX1_Busy;  //����æ��־

#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //���մ���1��������
u8 RX_CONT = 0; 

#define	UART1_TX_LENGTH 11
static u8 Display_Code[1]={0x00};//1��595���ư�����LED�ơ�
static u8 To_Marster_Data[UART1_TX_LENGTH]={0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,'*'};//���ذ�̵�������״̬��֡ͷ0x01,֡β0xff;
static u8 code LED_CMD[3]={0x1,0xff,'*'};
/********************** 8*8������� ************************/

static u8 KeyCode= 0x00;	//���û�ʹ�õļ���	
static u8 Key8_status = 0;

//�������ʱ�� 1�� ����ر���
volatile unsigned char vGu8TimeFlag_1=0;
volatile unsigned int vGu16TimeCnt_1=0;
BOOL flash_flag = TRUE;


#define KEY_FILTER_TIME 50 //�����˲��ġ� �ȶ�ʱ�䡱 50ms

/********************** A ��ָʾ���õ�595 ************************/
sbit	A_HC595_SER   = P3^4;	//pin 34	SER		data input
sbit	A_HC595_RCLK  = P3^7;	//pin 37	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P3^5;	//pin 35	SRCLK	Shift data clock
sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		�͵�ƽ ʹ��enable pin
sbit	A_HC595_MR    = P3^6;	//pin 36	�͵�ƽ��λ	


void uart1_config();	// ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
int Get_KeyVal(int val);
//========================================================================
// ����: u8 ms��ʱ������
//========================================================================
void  delay_ms(u8 ms)
{
     u16 i;
	 do{
	      i = MAIN_Fosc / 13000;
		  while(--i)	;   //14T per loop
     }while(--ms);
}
void timer0_init(void)
{
	AUXR = 0x80;	//Timer0 set as 1T, 16 bits timer auto-reload, 
	TH0 = (u8)(Timer0_Reload / 256);
	TL0 = (u8)(Timer0_Reload % 256);
	ET0 = 1;	//Timer0 interrupt enable
	TR0 = 1;	//Tiner0 run
	EA = 1;		//�����ж�
}	
//========================================================================
// ����: set_timer2_baudraye(u16 dat)
// ����: ����Timer2�������ʷ�������
// ����: dat: Timer2����װֵ.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================

void set_timer2_baudraye(u16 dat)
{
	AUXR &= ~(1<<4);	//Timer stop
	AUXR &= ~(1<<3);	//Timer2 set As Timer
	AUXR |=  (1<<2);	//Timer2 set as 1T mode
	TH2 = dat / 256;
	TL2 = dat % 256;
	IE2  &= ~(1<<2);	//��ֹ�ж�
	AUXR |=  (1<<4);	//Timer run enable
}

//========================================================================
// ����: Ӳ����ʼ�� ��
//========================================================================
void stc15x_hw_init(void)
{
	P0n_standard(0xff);	//����Ϊ׼˫���
	P1n_standard(0xff);	//����Ϊ׼˫���
	P2n_push_pull(0xff);	//����Ϊ׼˫���
	P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���	
	timer0_init();
	uart1_config();
	A_HC595_MR = 1;//��λ��ֹ
	A_HC595_OE = 0;//ʹ��оƬ
}
/**************** ��HC595����һ���ֽں��� ******************/
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
/**************** HC595�������溯�� ******************/
void vDataOut595()
{		
	A_HC595_RCLK = 0;
	NOP2();
	A_HC595_RCLK = 1;
}

/*****************************************************
	���м�ɨ�����
	ʹ��XY����8x8���ķ�����ֻ�ܵ������ٶȿ�

    Y  P10   P11   P12   P13   P14   P15   P16   P17
        |     |     |     |     |     |     |     |
X       |     |     |     |     |     |     |     |
P00 -- K00 - K01 - K02 - K03 - K04 - K05 - K06 - K07
        |     |     |     |     |     |     |     |
P01 -- K08 - K09 - K10 - K11 - K12 - K13 - K14 - K15
        |     |     |     |     |     |     |     |
P02 -- K16 - K17 - K18 - K19 - K20 - K21 - K22 - K23
        |     |     |     |     |     |     |     |
P03 -- K24 - K25 - K26 - K27 - K28 - K29 - K30 - K31
        |     |     |     |     |     |     |     |
P04 -- K32 - K33 - K34 - K35 - K36 - K37 - K38 - K39
        |     |     |     |     |     |     |     |
P05 -- K40 - K41 - K42 - K43 - K44 - K45 - K46 - K47
        |     |     |     |     |     |     |     |
P06 -- K48 - K49 - K50 - K51 - K52 - K53 - K54 - K55
        |     |     |     |     |     |     |     |
P07 -- K56 - K57 - K58 - K59 - K60 - K61 - K62 - K63
******************************************************/
void gpio_key_delay(void)
{
	u8 i;
	i = 60;
	while(--i)	;
}


void Kled_Set(int status,u8 Nled)
{
	u8 val;
	val = Display_Code[0];
	if(status==ON)
		Display_Code[0] = setbit(val,Nled);//�ߵ�ƽ��ͨ
	else if(status==OFF)
		Display_Code[0] = clrbit(val,Nled);	
	else if(ON_ALL == status)	
		Display_Code[0] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[0] = 0x00;
	else if(OFF_ON == status)
		Display_Code[0] = reversebit(val,Nled);//��ת
}

void Gpio_Keyscan(void)	//50ms call
{	
	static unsigned char Su8KeyLock1; //1 �Ű���������
	static unsigned int Su16KeyCnt1; //1 �Ű����ļ�ʱ��
	P10 = 0;
	if(KEY_BOARD_GPIO_Y==0xff)
	{
		Su8KeyLock1=0; //��������
		Su16KeyCnt1=0; //����ȥ������ʱ����������
		flash_flag = TRUE;
		if(!flash_flag)
			key_led_on(FALSE);//Ϩ�𰴼���ʾled
	}
	else if(0==Su8KeyLock1)
	{
		Su16KeyCnt1++; //�ۼӶ�ʱ�жϴ���
		if(Su16KeyCnt1>=KEY_FILTER_TIME) //�˲��ġ� �ȶ�ʱ�䡱 KEY_FILTER_TIME�� ������ 50ms��
		{
			Su8KeyLock1=1; //����������,����һֱ����
			key_led_on(TRUE);//������ʾled
			flash_flag = FALSE;
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
	static u8 temp = 0;
	temp = Display_Code[0];
	if((temp == 0x80) || (temp == 0x00))
		return 1;
	else if((temp == 0x7f) || (temp ==0xff))
		return 2;
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
#if defined CMD_ON_OFF_ALL
	if(KEY8_VAL == KeyCode)
		if(twice_cmd){
			print_string(LED_CMD);
			print_string(LED_CMD);
			twice_cmd = 0;
		}
		else
			print_string(LED_CMD);
	else
		print_string(To_Marster_Data);
#else
	print_string(To_Marster_Data);
#endif
		
}
void Key_EventProcess(int KeyCode)
{
	u8 LedBit = 0;
	static u8 LED_ST = 0;
	static u8 Odd_Flag = 1;//������־
	switch(KeyCode){
	case KEY1_VAL:
	case KEY2_VAL:
	case KEY3_VAL:
	case KEY4_VAL:
	case KEY5_VAL:
	case KEY6_VAL:
	case KEY7_VAL:
		LedBit = KeyCode - 0x10;
		Kled_Set(OFF_ON,LedBit);
		if(getbit(Display_Code[0],LedBit))
			LED_ST++;
		else
			LED_ST--;
		break;
	case KEY8_VAL:
		if(LED_ST==7)//������ż��ȫ��
		{
			Kled_Set(OFF_ALL,7);//ȫ��
			LED_ST = 0;//������0
			if(Odd_Flag==1)
				twice_cmd =1;
			Odd_Flag = 1;//����
			
		}
		else if(Odd_Flag)//����
		{
			Kled_Set(ON,7);//ȫ��
			LED_ST = 7;
			Odd_Flag = 0;//����Ϊż��
		}
		else if(Odd_Flag==0)//��ż��
		{
			Kled_Set(OFF_ALL,7);//ȫ��	
			LED_ST = 0;	
			Odd_Flag = 1;//����		
		}
		break;
	default:
		break;	
	}
	if(LED_ST>0)
		Kled_Set(ON,7);//8����
	else
		Kled_Set(OFF,7);//8����
	
	vDataIn595(Display_Code[0]);
	vDataOut595();
	if(KeyCode){
		Date_EventProcess();//����ֵת���ɷ���������ʽ��ֵ�����ڷ���;
		KeyCode = 0;//�����������ֵ
	}

}
//========================================================================
// ����: void main(void)
// ����: ������.
// ����: ��.
// ����: ��.
// �汾: V1.0, 2012-10-22
//========================================================================
void main(void)
{
	stc15x_hw_init();
	vDataIn595(0x00);
	vDataOut595();	//����Ĭ�Ϲر�ͨ����ʾLED
	puts_to_SerialPort("408as init uart");
	while (1)
	{
		if(B_1ms)	//1ms��
		{
			B_1ms = 0;	
			if(++msecond >= 1000)	//1�뵽
			{	
				msecond = 0;
				if(flash_flag)
					key_led_reverse();
			}
			if(++cnt50ms >= 50)		//50ms��
			{
				cnt50ms = 0;
			}
			Key_EventProcess(KeyCode);
			if(KeyCode > 0)		//�м�����
			{		
				KeyCode = 0;
			}			
		}
	}
}

//========================================================================
// ����: Timer0 1ms�жϺ�����
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	B_1ms = 1;		//1ms��־
	if(vGu8TimeFlag_1)
		vGu16TimeCnt_1++;
	else
		vGu16TimeCnt_1 = 0;
	Gpio_Keyscan();//����ɨ��
}
//========================================================================
// ����: void print_string(u8 *puts)
// ����: ����1�����ַ���������
// ����: puts:  �ַ���ָ��.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================

void print_string(u8 *puts)
{
  for (; *puts != '*';	puts++)   	//����ֹͣ��*����
	{
		SBUF = *puts;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
	}
}
void print_char(u8 dat)
{
		SBUF = dat;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
}
void puts_to_SerialPort(u8 *puts)
{
    for (; *puts != 0;	puts++)   	//����ֹͣ��0����
	{
		SBUF = *puts;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
	}
}
//========================================================================
// ����: void	uart1_config()
// ����: UART1��ʼ��������
// ����: brt: ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
#if 1
void uart1_config()
{
	/*********** ������ʹ�ö�ʱ��2 *****************/
		AUXR |= 0x01;		//S1 BRT Use Timer2;
		set_timer2_baudraye(65536UL - (MAIN_Fosc / 4) / Baudrate1);

	SCON = (SCON & 0x3f) | 0x40;	//UART1ģʽ, 0x00: ͬ����λ���, 0x40: 8λ����,�ɱ䲨����, 0x80: 9λ����,�̶�������, 0xc0: 9λ����,�ɱ䲨����
	S1_USE_P30P31();//UART1 ʹ��P30 P31��	Ĭ��
	ES  = 1;	//�����ж�
	REN = 1;	//�������
	EA = 1;

	B_TX1_Busy = 0;
}
#endif
//========================================================================
// ����: void uart1_int (void) interrupt UART1_VECTOR
// ����: UART1�жϺ�����
// ����: nine.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
void uart1_int (void) interrupt UART1_VECTOR
{
	if(RI)
	{
		RI = 0;
		Rec_Buf[RX_CONT] = SBUF;                    //�Ѵ���1����SBUF�Ĵ����������δ�ŵ�����Rec_Buf��
		RX_CONT++;                                   
		if(RX_CONT>Buf_Max)                          //���������ڶ����������������ʱ�����ǽ�������֮ǰֵ
		{
			RX_CONT = 0;
		}   
	}

	if(TI)
	{
		TI = 0;
		B_TX1_Busy = 0;
	}
}