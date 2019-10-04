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

#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))		//Timer 0 �ж�Ƶ��, 1000��/��
#define	Baudrate1	9600UL                                //ͨ�Ų�����115200

#define	UART1_RX_LENGTH 5
#define	UART1_TX_LENGTH 8
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

/********************** ����������Э�鶨�� ************************/
//֡ͷ+ָ��+���ݳ���+�����+���ݰ�+֡β(δ����)
//MJX+cmd+0x03+�����+�����̵���L8+�̵���H8��+�������+֡β
//-----------------------------------------------------------------
#define ID_LENGTH(x)  sizeof(x)/sizeof(u8)
#define Buf_Max 20
u8 xdata Rec_Buf[Buf_Max];       //���մ���1��������
u8 idata RxData[UART1_RX_LENGTH];//��ȡ����ר�ã������+���ֽڼ̵���״̬
u8 code SEND_ID[] ="MJX";
u8 RX_CONT = 0;  
BOOL Hand(u8 *a);

enum {
	CMD_1 = 0x01,  //�̵���״ָ̬��
	CMD_2,		   //����ֵָ��
	CMD_3,//
	CMD_4,//
	CMD_MAX
};
/********************** ����Э�鶨�� ************************/
//MJX+cmd+0x02+�����+����ֵ+�����)+֡β(δ����)
//-----------------------------------------------------------------
u8 tx_buffer_vale[UART1_TX_LENGTH]={'M','J','X',CMD_2,0x02,0x00,0x00,'*'};	 //��������,����λkeyֵ 0~63+16
/********************** 8*8������� ************************/
#if defined GPIO_KEY64_BOARD
#define KEY_BOARD_GPIO_X P1
#define KEY_BOARD_GPIO_Y P2
u8 code Line_KeyTable[]={0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};	//������
u8 IO_KeyState =0,IO_KeyState_x =0,IO_KeyState_y =0,IO_KeyState_old = 0xff;	//���м��̱���
u8	KeyCode= 0xff;	//���û�ʹ�õļ���, 0~63��Ч
#endif

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
/**************************************
�������������ֳɹ������
��ڲ�����uint8 *a
����ֵ��λ
***************************************/
BOOL Hand(u8 *a)
{ 
  if(strstr(Rec_Buf,a)!=NULL)     //�ж��ַ���a�Ƿ����ַ���Rec_Buf���Ӵ�
		return 1;                     //����ַ���a���ַ���Rec_Buf���Ӵ�
	else
		return 0;                     //����ַ���a�����ַ���Rec_Buf���Ӵ�
}

/**************************************
��������������������ݺ���
��ڲ�������
����ֵ����
***************************************/
void CLR_Buf(void)
{
	u8 k;
	for(k=0;k<Buf_Max;k++)        //������2���������ֵ����Ϊ��  
	{
		Rec_Buf[k] = 0;
	}
    RX_CONT = 0;                    
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
	P2n_standard(0xff);	//����Ϊ׼˫���
	P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���	
	timer0_init();
	uart1_config();
	A_HC595_MR = 1;//��λ��ֹ
	A_HC595_OE = 0;//ʹ��оƬ
}
/**************** ��HC595����һ���ֽں��� ******************/
void Data_In595(u8 dat)
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
void Data_Out595()
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
			//print_char(KeyCode);
		}
			key_led_on(TRUE);//Ϩ�𰴼���ʾled
	}
	KEY_BOARD_GPIO_X = 0xff;
	KEY_BOARD_GPIO_Y = 0xff;
}
void Do_Keyboard_event(u8 key_code)
{	
	if(key_code){
		tx_buffer_vale[5] = creat_random();//���������
		tx_buffer_vale[6] = KeyCode - 16 + tx_buffer_vale[5];//��������ڼ�ֵ�з��ͣ�������Ҫ����
		//print_char(KeyCode);
		print_string(tx_buffer_vale);
		
	}
}
#endif
//========================================================================
// ����: void Rcev_DataAnalysis(void)
// ����: uart��ȡ���ݰ�У�鼰����
// ����: nine.
// ����: none.
// �汾: VER1.0
// ����: 2019-9-20
// ��ע: 
//========================================================================
//cmd��MJX+cmd1+���ݳ���+�����+595ֵ;
void Switch_Led_DataProce(void)
{
	u8 i=0;
	//Fake_PrintString("chengronghe\n");
	for(i=0;i<Rec_Buf[ID_LENGTH(SEND_ID)];i++)
	{
		RxData[i] = Rec_Buf[i+ID_LENGTH(SEND_ID)+1];//ȡ�����ݳ����ֽں�������
		if(i==0){
			//do nothing
		}
		else{
			RxData[i] = RxData[i] - RxData[0];
			Data_In595(RxData[i]);//�ȷ���λ���ٷ���λ
		}
	}
	Data_Out595();
}
void Rcev_DataAnalysis(void)
{
	u8 recv_cmd = 0;
	//Fake_PrintString("74hc595 LED CMD!\r\n");
	if(Hand(SEND_ID))
	{
#if defined FAKE_SERIAL
		Fake_PrintString(SEND_ID);
		Fake_PrintString(":");
		Fake_PrintString("\r\n");
#endif
		recv_cmd = Rec_Buf[ID_LENGTH(SEND_ID)-1];//����Ϊ�ַ�+\0,��ȡָ��
		switch(recv_cmd)
		{
			case CMD_1://LED Channel Data
			#if defined FAKE_SERIAL
					Fake_PrintString("74hc595 LED CMD!\r\n");
			#endif
					Switch_Led_DataProce();
				break;
			default:
			#if defined FAKE_SERIAL
				Fake_PrintString("ERROR CMD!\r\n");
			#endif
				break;
		}		
		CLR_Buf();
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
	Data_In595(0x00);
	Data_In595(0x00);
	Data_Out595();	//����Ĭ�Ϲر�ͨ����ʾLED
	CLR_Buf();      //������ջ���
	Fake_PrintString("STC15W408as fake UART init OK!\r\n");
	while (1)
	{
		if(B_1ms)	//1ms��
		{
			B_1ms = 0;
			if(++msecond >= 1000)	//1�뵽
			{	
				msecond = 0;
				//key_led_reverse();
			}
		  if(++cnt50ms >= 50)		//50msɨ��һ�����м���
			{
				cnt50ms = 0;
		#if defined GPIO_KEY64_BOARD
				Gpio_Keyscan();
		#endif
			}
		#if defined GPIO_KEY64_BOARD
			if(KeyCode > 0)		//�м�����
			{
				Do_Keyboard_event(KeyCode);//send KeyCode to master
				KeyCode = 0;
			}
		#endif
			Rcev_DataAnalysis();//֡ͷ���飬���ݷ�������
		}
	}
}

//========================================================================
// ����: Timer0 1ms�жϺ�����
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	B_1ms = 1;		//1ms��־
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
		//Fake_PrintString("74hc595 LED CMD!\r\n");
		RI = 0;
		Rec_Buf[RX_CONT] = SBUF;                    //�Ѵ���1����SBUF�Ĵ����������δ�ŵ�����Rec_Buf��
		TxSend(SBUF);
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
