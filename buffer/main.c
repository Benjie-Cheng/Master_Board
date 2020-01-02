/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/
/*************	��������˵��	**************

��STC��MCU��IO��ʽ������̺ʹ����շ����ݡ�
1��8*8 ������̼���Ӧ�̵����İ������롣����һ��״̬��תһ�Ρ�
2�����ڽ�ɨ�赽�ļ�ֵ���͵�����������������ǰ�̵���ͨ��״̬���ӻ���
3���ӻ������������յ������ݣ�ͨ��74HC595����LED,ָʾͨ������״̬��
4���а�������ʱ��һ�Ź��õ�LEDָʾ��������

����ȱ�ݣ�
1��8*8���̵�LED���ڰ����Ҳ࣬ÿ�ΰ��¾�����ָ�ڵ������Էֱ�̵���״̬�������Ҳ�����ϲࡣ
2��LED������Ϊ�ߵ�ƽ��������������������ס���ɲ����ⲿ�������͵�ƽ������ʽ

v1.1��
1���޸Ĳ�����115200->9600           20190929
2��ͳһͨ�����ݸ�ʽ                 20190929
******************************************/
#include "config.h"
#include "debug.h"
#include "eeprom.h"

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 �ж�Ƶ��, 1000��/��
#define	Baudrate1	9600UL                                //ͨ�Ų�����115200

#define GPIO_KEY64_BOARD
#define KEY_LED_GPIO P55
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
/********************** 8*8������� ************************/
#if defined GPIO_KEY64_BOARD
#define KEY_BOARD_GPIO_X P1
#define KEY_BOARD_GPIO_Y P2
u8 code Line_KeyTable[]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};	//������
u8 IO_KeyState =0,IO_KeyState_x =0,IO_KeyState_y =0,IO_KeyState_old = 0xff;	//���м��̱���
u8	KeyCode= 0xff;	//���û�ʹ�õļ���, 0~63��Ч
#endif


//�������ʱ�� 1�� ����ر���
volatile unsigned char vGu8TimeFlag_1=0;
volatile unsigned int vGu16TimeCnt_1=0;
#define LED_TIME_3S  3000  //ʱ���� 3000ms
#define LED_TIME_7S  7000  //ʱ���� 3000ms
#define LED_TIME_10S 10000 //ʱ���� 3000ms
#define LED_TIME_15S 15000 //ʱ���� 3000ms
#define LED_TIME_18S 18000 //ʱ���� 3000ms
#define LED_TIME_23S 23000 //ʱ���� 3000ms
#define LED_TIME_26S 26000 //ʱ���� 3000ms
#define LED_TIME_31S 31000 //ʱ���� 3000ms
#define LED_TIME_34S 34000 //ʱ���� 3000ms
#define LED_TIME_39S 39000 //ʱ���� 3000ms
#define LED_TIME_42S 42000 //ʱ���� 3000ms
#define LED_TIME_60S 60000 //ʱ���� 3000ms
#define KEY8_VAL 0x17
#define     E2PROM_LENGTH 8
u16 T0,T2,T3;
u16 U1_T1,U2_T1,U3_T1,U4_T1,U5_T1;			 //T0,  T2,  T3, U1_T1, U2_T1, U3_T1,  U4_T1,  U5_T1
static u8 	E2PROM_Strings[E2PROM_LENGTH] = {0x00,0x00,0x00,0x00,  0x00,   0x00,   0x00,   0x00};
#define GET_INIT_VAL()	{T0=E2PROM_Strings[0]*1000;T2=E2PROM_Strings[1]*1000;T3=E2PROM_Strings[2]*1000;\
U1_T1=E2PROM_Strings[3]*1000;U2_T1=E2PROM_Strings[4]*1000;U3_T1=E2PROM_Strings[5]*1000;U4_T1=E2PROM_Strings[6]*1000;U5_T1=E2PROM_Strings[7]*1000;}

static u8 Gu8Step = 0; //�����ʱ�� 1 �� switch �л�����
static u8 vGu8Key1_lock = 0;
static u8 vGu8Key2_lock = 0;
static u8 vGu8Key3_lock = 0;
static u8 vGu8Key4_lock = 0;
static u8 vGu8Key5_lock = 0;
static u8 vGu8Key6_lock = 0;
static u8 vGu8Key7_lock = 0;
static u8 vGu8Key8_lock = 0;

#define KEY_FILTER_TIME 50 //�����˲��ġ� �ȶ�ʱ�䡱 25ms

/********************** A ��ָʾ���õ�595 ************************/
sbit	A_HC595_SER   = P3^4;	//pin 34	SER		data input
sbit	A_HC595_RCLK  = P3^7;	//pin 37	RCLk	store (latch) clock
sbit	A_HC595_SRCLK = P3^5;	//pin 35	SRCLK	Shift data clock
sbit	A_HC595_OE    = P5^4;	//pin 54	OE 		�͵�ƽ ʹ��enable pin
sbit	A_HC595_MR    = P3^6;	//pin 36	�͵�ƽ��λ	


static u8 Display_Code[2]={0x00,0x00};//����595���ݡ�

void uart1_config();	// ѡ������, 2: ʹ��Timer2��������, ����ֵ: ʹ��Timer1��������.
void print_string(u8 *puts);
void puts_to_SerialPort(u8 *puts);
void print_char(u8 dat);
void Key1_Fun(void);
void Key2_Fun(void);
void Key3_Fun(void);
void Key4_Fun(void);
void Key5_Fun(void);
void Key6_Fun(void);
void Key7_Fun(void);
void Key8_Fun(void);
int Check_KeyStatus(void);
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
void Nmos_Set(BOOL en,u8 Nmos)
{
	u8 val;
	val = Display_Code[1];
	if(en)
		Display_Code[1] = setbit(val,Nmos);//�ߵ�ƽ��ͨ�����ܣ�Mos�ܹ���
	else
		Display_Code[1] = clrbit(val,Nmos);	
	if(0xff == Nmos)	
		Display_Code[1] &= 0x80;
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
#if defined GPIO_KEY64_BOARD
void gpio_key_delay(void)
{
	u8 i;
	i = 60;
	while(--i)	;
}
u16 compose(u8 x,u8 y)
{
	u16 val;
	val = y;
	return (val<<8) | x;
}	
u8 val_to_line(u8 key_val)
{
	u8 val = 0,i;
	
	for(i=0;i<8;i++)
	{
		if(key_val == Line_KeyTable[i])
			val = i;
	}
	return val;
}

#if 0
void Gpio_Keyscan(void)	//50ms call
{
	u16	j,x,y;
	
	x = val_to_line(IO_KeyState_x);			
	y = val_to_line(IO_KeyState_y);
	j = x * 8 + y + 16;
	
	KEY_BOARD_GPIO_Y = 0xff;
	KEY_BOARD_GPIO_X = 0x00;	//X�ͣ���Y
	gpio_key_delay();
	IO_KeyState_y = KEY_BOARD_GPIO_Y & 0xff;
	
	KEY_BOARD_GPIO_X =0xff;
	KEY_BOARD_GPIO_Y = 0x00;	//Y�ͣ���X
	gpio_key_delay();
	IO_KeyState_x= KEY_BOARD_GPIO_X & 0xff;
	
	IO_KeyState_x ^= 0xff;	//ȡ��
	IO_KeyState_y ^= 0xff;	//ȡ��
	

#if 1
	x = val_to_line(IO_KeyState_x);			
	y = val_to_line(IO_KeyState_y);
	//print_char(x);
	//print_char(y);
	IO_KeyState = x * 8 + y + 16;	//������룬(0~63) +16
#endif
	if(!IO_KeyState_x && !IO_KeyState_y){
		IO_KeyState_old = 0;
		key_led_on(FALSE);//Ϩ�𰴼���ʾled
	}
	else //if(j == IO_KeyState)	//�������ζ����
	{
		if(IO_KeyState != 0 &&(IO_KeyState_old != IO_KeyState))	//�м�����
		{
			IO_KeyState_old = IO_KeyState;
			x = val_to_line(IO_KeyState_x);			
			y = val_to_line(IO_KeyState_y);
			KeyCode = x * 8 + y + 16;	//������룬(0~63) +16
			print_char(KeyCode);
		}
			key_led_on(TRUE);//Ϩ�𰴼���ʾled
	}
	KEY_BOARD_GPIO_X = 0xff;
	KEY_BOARD_GPIO_Y = 0xff;
}
#else
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
void Gpio_Keyscan(void)	//50ms call
{	
	static unsigned char Su8KeyLock1; //1 �Ű���������
	static unsigned int Su16KeyCnt1; //1 �Ű����ļ�ʱ��

		P10 = 0;
		if(KEY_BOARD_GPIO_Y==0xff)
		{
			Su8KeyLock1=0; //��������
			Su16KeyCnt1=0; //����ȥ������ʱ����������
			key_led_on(FALSE);//Ϩ�𰴼���ʾled
		}
		else if(0==Su8KeyLock1)
		{
			Su16KeyCnt1++; //�ۼӶ�ʱ�жϴ���
			if(Su16KeyCnt1>=KEY_FILTER_TIME) //�˲��ġ� �ȶ�ʱ�䡱 KEY_FILTER_TIME�� ������ 25ms��
			{
				Su8KeyLock1=1; //����������,����һֱ����
				key_led_on(TRUE);//������ʾled
				KeyCode = Get_KeyVal(KEY_BOARD_GPIO_Y);
				if(Check_KeyStatus()&&(KeyCode!=KEY8_VAL))
					KeyCode = 0;
			}		
		}
}	
#endif
#endif


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
	vDataIn595(0x00);
	vDataOut595();	//����Ĭ�Ϲر�ͨ����ʾLED
	puts_to_SerialPort("init uart");
	EEPROM_read_n(IAP_ADDRESS,E2PROM_Strings,E2PROM_LENGTH);
	GET_INIT_VAL();
	while (1)
	{
		if(B_1ms)	//1ms��
		{
			B_1ms = 0;	
			if(++msecond >= 1000)	//1�뵽
			{	
				msecond = 0;
				Display_Code[1]=reversebit(Display_Code[1],7);
			}
		  if(++cnt50ms >= 50)		//50msɨ��һ�����м���
			{
				cnt50ms = 0;
		#if defined GPIO_KEY64_BOARD
				//Gpio_Keyscan();
				vDataIn595(Display_Code[1]);
				vDataIn595(Display_Code[1]);
				vDataOut595();
		#endif
			}
		#if defined GPIO_KEY64_BOARD
			if((Check_KeyStatus())&&(KeyCode==KEY8_VAL))
			{
				vGu8Key1_lock = 0;
				vGu8Key2_lock = 0;
				vGu8Key3_lock = 0;
				vGu8Key4_lock = 0;
				vGu8Key5_lock = 0;
				vGu8Key6_lock = 0;
				vGu8Key7_lock = 0;	
				vGu8Key8_lock = 0;
				vGu16TimeCnt_1 = 0;
				Gu8Step	= 0;	
			}
			/*
			else if((KeyCode == KEY8_VAL) && vGu8Key8_lock)
			{
				Key8_Fun();
				KeyCode = 0;
				print_char(0xcc);
			}
			*/
			if(KeyCode > 0)		//�м�����
			{	
					//Nmos_Set(0,6);//debug led
					switch(KeyCode)
					{		
						case 0x10:
							vGu8Key1_lock = 1;
							break; 
						case 0x11:
							vGu8Key2_lock = 1;
							break;
						case 0x12:
							vGu8Key3_lock = 1;
							break;		
						case 0x13:
							vGu8Key4_lock = 1;
							break;		
						case 0x14:
							vGu8Key5_lock = 1;
							break;	
						case 0x15:
							vGu8Key6_lock = 1;
							break;	
						case 0x16:
							vGu8Key7_lock = 1;
							break;	
						case 0x17:
							vGu8Key8_lock = 1;
							break;	
						default:
							break;
					}				
					KeyCode = 0;
					Gu8Step = 0;//step ��0
			}			
	
			if(vGu8Key8_lock)
			{
				KeyCode = 0;
				Key8_Fun();
				print_char(0x0a);
			}
			else if(vGu8Key1_lock){
				Key1_Fun();
				print_char(0x0b);
			}
			else if(vGu8Key2_lock){
				Key2_Fun();
				print_char(0x0c);
			}
			else if(vGu8Key3_lock){
				Key3_Fun();
				print_char(0x0d);
			}
			else if(vGu8Key4_lock){
				Key4_Fun();
				print_char(0x0e);
			}
			else if(vGu8Key5_lock){
				Key5_Fun();
				print_char(0x0f);
			}
			else if(vGu8Key6_lock){
				Key6_Fun();
				print_char(0xaa);
			}
			else if(vGu8Key7_lock){
				Key7_Fun();	
				print_char(0xbb);
			}
		#endif	
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
	Gpio_Keyscan();
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
		//TxSend(SBUF);
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
void Key1_Fun(void)
{
	
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			//vGu8Key1_lock = 1;//��������
			vGu8TimeFlag_1 = 1;//��ʱ��������λ
			Nmos_Set(1,6);//debug led
			Nmos_Set(1,0);
			Nmos_Set(1,1);
			Nmos_Set(1,2);
			Nmos_Set(1,3);
			Nmos_Set(1,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T0) //T0ʱ�䵽
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);	
			if(vGu16TimeCnt_1>=T3) //T3sʱ�䵽
				Gu8Step++;
			break;
		case 2:
			//Nmos_Set(1,6);//debug led
			Gu8Step = 0;//step ��0
			vGu8TimeFlag_1 = 0;//�嶨ʱ�����б�־
			vGu8Key1_lock = 0;//�ⰴ����
			break;	
		default:
		
		break;
	}	
}
void Key2_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			Nmos_Set(1,6);//debug led
			//vGu8Key2_lock=1;//��������
			vGu8TimeFlag_1 = 1;//��ʱ��������λ
			Nmos_Set(1,0);
			Nmos_Set(1,1);
			Nmos_Set(1,2);
			Nmos_Set(1,3);
			Nmos_Set(1,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T0) //3sʱ�䵽
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);//��1-5λ��
			if(vGu16TimeCnt_1>=(U5_T1+T0)) //����·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 2:
			Nmos_Set(1,4);//��5λ��
			if(vGu16TimeCnt_1>=(U5_T1+T0+T2)) //����·��ͨʱ���ֹ
				Gu8Step++;		
			break;	
		case 3:
			Nmos_Set(0,4);//��5λ��
			if(vGu16TimeCnt_1>=(U4_T1+T0)) //��5·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 4:
			Nmos_Set(1,3);//��4λ��
			if(vGu16TimeCnt_1>=(U4_T1+T0+T2)) //��4·��ͨʱ���ֹ
				Gu8Step++;
			break;
		case 5:
			Nmos_Set(0,3);//��4λ��
			if(vGu16TimeCnt_1>=(U3_T1+T0)) //��3·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 6:
			Nmos_Set(1,2);//��3λ��
			if(vGu16TimeCnt_1>=(U3_T1+T0+T2)) //��3·��ͨʱ���ֹ
				Gu8Step++;
			break;
		case 7:
			Nmos_Set(0,2);//��3λ��
			if(vGu16TimeCnt_1>=(U2_T1+T0)) //��2·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 8:
			Nmos_Set(1,1);//��2λ��
			if(vGu16TimeCnt_1>=(U2_T1+T0+T2)) //��2·��ͨʱ���ֹ
				Gu8Step++;
			break;
		case 9:
			Nmos_Set(0,1);//��2λ��
			if(vGu16TimeCnt_1>=(U1_T1+T0)) //��1·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 10:
			Nmos_Set(1,0);//��1λ��
			if(vGu16TimeCnt_1>=(U1_T1+T0+T2)) //��1·��ͨʱ���ֹ
				Gu8Step++;
			break;
		case 11:
			Nmos_Set(0,0);//��1λ��
			if(vGu16TimeCnt_1>=T3) //T3sʱ�䵽,ȫ��Ϩ����Խ��ܿ���
				Gu8Step++;
			break;
		case 12:
			Gu8Step = 0;//step ��0
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//�嶨ʱ�����б�־
			vGu8Key2_lock = 0;//�ⰴ����
			break;
		default:	
			break;
	}
}
void Key3_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			Nmos_Set(1,6);//debug led
			//vGu8Key3_lock=1;//��������
			vGu8TimeFlag_1 = 1;//��ʱ��������λ
			Nmos_Set(1,0);
			Nmos_Set(1,1);
			Nmos_Set(1,2);
			Nmos_Set(1,3);
			Nmos_Set(0,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T0) //T0ʱ�䵽
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);//��1-5λ��
			
			if(vGu16TimeCnt_1>=(U4_T1+T0)) //��4·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 2:
			Nmos_Set(1,3);//��4λ��
			if(vGu16TimeCnt_1>=(U4_T1+T0+T2)) //��4·��ͨʱ���ֹ
				Gu8Step++;		
			break;	
		case 3:
			Nmos_Set(0,3);//��4λ��
			if(vGu16TimeCnt_1>=(U3_T1+T0)) //��3·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 4:
			Nmos_Set(1,2);//��3λ��
			if(vGu16TimeCnt_1>=(U3_T1+T0+T2)) //��3·��ͨʱ���ֹ
				Gu8Step++;
			break;
		case 5:
			Nmos_Set(0,2);//��3λ��
			if(vGu16TimeCnt_1>=(U2_T1+T0)) //��2·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 6:
			Nmos_Set(1,1);//��2λ��
			if(vGu16TimeCnt_1>=(U2_T1+T0+T2)) //��2·��ͨʱ���ֹ
				Gu8Step++;
			break;
		case 7:
			Nmos_Set(0,1);//��2λ��
			if(vGu16TimeCnt_1>=(U1_T1+T0)) //��1·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 8:
			Nmos_Set(1,0);//��1λ��
			if(vGu16TimeCnt_1>=(U1_T1+T0+T2)) //��1·��ͨʱ���ֹ
				Gu8Step++;
			break;
		case 9:
			Nmos_Set(0,0);//��1λ��
			if(vGu16TimeCnt_1>=T3) //T3sʱ�䵽,ȫ��Ϩ����Խ��ܿ���
				Gu8Step++;
			break;
		case 10:
			vGu8Key3_lock = 0;//�ⰴ����
			vGu8TimeFlag_1 = 0;//�嶨ʱ�����б�־
			Gu8Step = 0;//step ��0
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//�嶨ʱ�����б�־
			vGu8Key3_lock = 0;//�ⰴ����
			break;
		default:	
			break;
	}
}

void Key4_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			Nmos_Set(1,6);//debug led
			vGu8Key4_lock=1;//��������
			vGu8TimeFlag_1 = 1;//��ʱ��������λ
			Nmos_Set(1,0);
			Nmos_Set(1,1);
			Nmos_Set(1,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T0) //T0ʱ�䵽
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);//��1-5λ��
			if(vGu16TimeCnt_1>=(U3_T1+T0)) //��3·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 2:
			Nmos_Set(1,2);//��3λ��
			if(vGu16TimeCnt_1>=(U3_T1+T0+T2)) //��3·��ͨʱ���ֹ
				Gu8Step++;		
			break;	
		case 3:
			Nmos_Set(0,2);//��3λ��
			if(vGu16TimeCnt_1>=(U2_T1+T0)) //��2·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 4:
			Nmos_Set(1,1);//��2λ��
			if(vGu16TimeCnt_1>=(U2_T1+T0+T2)) //��2·��ͨʱ���ֹ
				Gu8Step++;
			break;
		case 5:
			Nmos_Set(0,1);//��2λ��
			if(vGu16TimeCnt_1>=(U1_T1+T0)) //��1·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 6:
			Nmos_Set(1,0);//��2λ��
			if(vGu16TimeCnt_1>=(U1_T1+T0+T2)) //��1·��ͨʱ���ֹ
				Gu8Step++;
			break;
		case 7:
			Nmos_Set(0,0);//��1λ��
			if(vGu16TimeCnt_1>=T3) //T3sʱ�䵽,ȫ��Ϩ����Խ��ܿ���
				Gu8Step++;
			break;
		case 8:
			Gu8Step = 0;//step ��0
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//�嶨ʱ�����б�־
			vGu8Key4_lock = 0;//�ⰴ����
			break;
		default:	
			break;
	}
}
void Key5_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			Nmos_Set(1,6);//debug led
			//vGu8Key5_lock=1;//��������
			vGu8TimeFlag_1 = 1;//��ʱ��������λ
			Nmos_Set(1,0);
			Nmos_Set(1,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T0) //T0ʱ�䵽
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);//��1-5λ��
			if(vGu16TimeCnt_1>=(U2_T1+T0)) //��2·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 2:
			Nmos_Set(1,1);//��2λ��
			if(vGu16TimeCnt_1>=(U2_T1+T0+T2)) //��2·��ͨʱ���ֹ
				Gu8Step++;		
			break;	
		case 3:
			Nmos_Set(0,1);//��2λ��
			if(vGu16TimeCnt_1>=(U1_T1+T0)) //��1·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 4:
			Nmos_Set(1,0);//��1λ��
			if(vGu16TimeCnt_1>=(U1_T1+T0+T2)) //��1·��ͨʱ���ֹ
				Gu8Step++;
			break;
		case 5:
			Nmos_Set(0,0);//��1λ��
			if(vGu16TimeCnt_1>=T3) //T3sʱ�䵽,ȫ��Ϩ����Խ��ܿ���
				Gu8Step++;
			break;
		case 6:
			Gu8Step = 0;//step ��0
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//�嶨ʱ�����б�־
			vGu8Key5_lock = 0;//�ⰴ����		
			break;
		default:	
			break;
	}
}
void Key6_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			//vGu8Key6_lock=1;//��������
			Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 1;//��ʱ��������λ	
			Nmos_Set(1,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);
			Nmos_Set(1,5);//
			if(LED_TIME_3S<=vGu16TimeCnt_1) //3sʱ�䵽
				Gu8Step++;
			break;
		case 1:
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);//��1-5λ��
			if(vGu16TimeCnt_1>=(U1_T1+T0)) //��1·�ض�ʱ���ֹ
				Gu8Step++;
			break;
		case 2:
			Nmos_Set(1,0);//��1λ��
			if(vGu16TimeCnt_1>=(U1_T1+T0+T2)) //��1·��ͨʱ���ֹ
				Gu8Step++;		
			break;	
		case 3:
			Nmos_Set(0,0);//��1λ��
			if(vGu16TimeCnt_1>=T3) //T3sʱ�䵽,ȫ��Ϩ����Խ��ܿ���
				Gu8Step++;
			break;
		case 4:
			vGu8Key6_lock = 0;//�ⰴ����
			vGu8TimeFlag_1 = 0;//�嶨ʱ�����б�־
			Gu8Step = 0;//step ��0
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//�嶨ʱ�����б�־
			vGu8Key6_lock = 0;//�ⰴ����
			break;
		default:	
			break;
	}
}
void Key7_Fun(void)
{
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			//vGu8Key7_lock=1;//��������
			Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 1;//��ʱ��������λ
			Nmos_Set(0,0);
			Nmos_Set(0,1);
			Nmos_Set(0,2);
			Nmos_Set(0,3);
			Nmos_Set(0,4);
			Nmos_Set(1,5);//
			if(vGu16TimeCnt_1>=T3) //T3sʱ�䵽,ȫ��Ϩ����Խ��ܿ���
				Gu8Step++;
			break;
		case 1:
			//Nmos_Set(1,6);//debug led
			vGu8TimeFlag_1 = 0;//�嶨ʱ�����б�־
			vGu8Key7_lock = 0;//�ⰴ����
			Gu8Step = 0;//step ��0			
			break;
		default:	
			break;
	}
}

void Key8_Fun(void)
{	
	switch(Gu8Step)
	{
		KeyCode = 0;
		case 0:
			Nmos_Set(1,0xff);//ȫ���ر�
			Nmos_Set(0,6);//debug led
			vGu8TimeFlag_1 = 1;//��ʱ��������λ
			//print_char(0x22);
			if(vGu16TimeCnt_1>=T3) //T3sʱ�䵽,ȫ��Ϩ����Խ��ܿ���
				Gu8Step++;
			break;
		case 1:
			vGu8Key1_lock = 0;
			vGu8Key2_lock = 0;
			vGu8Key3_lock = 0;
			vGu8Key4_lock = 0;
			vGu8Key5_lock = 0;
			vGu8Key6_lock = 0;
			vGu8Key7_lock = 0;
			
			KeyCode = 0;
			vGu8TimeFlag_1 = 0;//�嶨ʱ�����б�־
			vGu8Key8_lock = 0;//�ⰴ����
			Gu8Step = 0;//step ��0			
			break;
		default:	
			break;
	}
}
int Check_KeyStatus(void)
{
	if(vGu8Key7_lock || vGu8Key6_lock || vGu8Key5_lock || vGu8Key4_lock || vGu8Key3_lock || vGu8Key2_lock || vGu8Key1_lock)
		return 1;
	else 
		return 0;	
}