/*---------------------------------------------------------------------*/
/* --- Fuctin: Multi-control system -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/

/*************	��������˵��	**************
��STC��MCU����������wifi�����ߡ�����ָ�ʵ�̵ּ������ƣ���ˮ�ƴ���ҵ��

��Դ��
��ʱ��0 ����ģ�⴮�ڽ���ʹ��;
��ʱ��1 ���ڴ���1�����ʷ���,������ʹ��,����Ĭ��9600;
��ʱ��2 ��������1ms��ʱ

1���������ڽ��մӻ�8*8������̰�������Ƽ̵����� 
	֡ͷ+ָ��+���ݳ���+�����+������+֡β(��δ����) //��������Ҫ����
2�������������ݡ�����1���жϷ�ʽ,115200
3��wifi�������ݡ�
4�����߽������ݡ�
5�����Ƽ̵�����

����ȱ�ݣ�
1��ģ�⴮��uart3�շ����ݲ��ȶ����Ż���
2��1�������Ѿ�ʹ�ö�ʱ��0ģ���·���ڽ��ս��

V1.1��¼��
1���޸��������ֳɹ���ʾ������                         20190930
2�����Ӱ�������������595�Ŀ��أ�����ͨ��ֵ����������  20190930
V1.2��¼��
1������Э���޸ĳ�����APP��ʽ������ͨ�Žӿ���ʱ���䡣  20191008
2���ڲ��洢4K SRAM ��ʱδ��ͨ��                       20191008
�����ԣ�
1���ڲ�EEPROM ��ʱδ��ͨ��                            20191008
2��2000·595��·��                                    20191008
******************************************/
#include "config.h"
#include "debug.h"
#include "led_lib.h"

//========================================================================
/*************	���ر�������	**************/
BOOL B_1ms;	 //1ms��־
u16	msecond; //1s����
u8 cnt50ms;  //50ms����
u8 cnt10ms;  //10ms����
u8 code SEND_ID[] ="MJX";
u8 code SEND_END[] ="0xFF";
#define ID_LENGTH(x)  sizeof(x)/sizeof(u8)
u8 xdata RxData[5];
enum {
	CMD_1 = 0x01,//����
	CMD_2,//Key_board
	CMD_3,//wifi
	CMD_4,//PT2272
	CMD_MAX
};
//========================================================================
//����: Uart1 ����ģ��ʹ�ö��塣
#define UART1_TIMER1

#if defined UART1_TIMER1
#define	Baudrate1	115200UL 
BOOL B_TX1_Busy;  //����æ��־
void uart1_config(void);
#endif

//========================================================================
// ����: B ���̵����õ�595
//�����-��������-����-����ͨ�̵�����LED����
sbit	B_HC595_SER   = P0^4;	//pin 04	SER		data input
sbit	B_HC595_RCLK  = P0^1;	//pin 01	RCLk	store (latch) clock
sbit	B_HC595_SRCLK = P0^3;	//pin 03	SRCLK	Shift data clock
sbit	B_HC595_OE    = P0^0;	//pin 00	OE 		�͵�ƽ ʹ��enable pin
sbit	B_HC595_MR    = P0^2;	//pin 02	�͵�ƽ��λ	

enum {
	CMD1 = 0x01,  //��������led ��״ָ̬��
	CMD2,//
	CMD3,//
	CMD4,//
};
#define	UART1_TX_LENGTH 9
u8 tx_buffer_vale[UART1_TX_LENGTH]={'M','J','X',0x01,0x03,0x00,0x00,0x00,'*'};	 //��������,���������+�̵���״̬
u8 switch_status0 = 0x00;//��һ���̵���״̬���߹ض�
u8 switch_status1 = 0x00;//�ڶ����̵���״̬���߹ض�
//================================================================
// ����: MLT_BT05,����1��P3.6,P3.7
/****************************************************************
ATָ�
*****************************************************************/	
char code str1[]="AT\r\n";                                    		    //  AT���ԣ�����"OK"
char code str2[]="AT+DEFAULT\r\n";                         		        //  �ָ�����
char code str3[]="AT+BAUD\r\n";     			      					//  ��ѯ������+BAUD=8��115200 +BAUD=8��9600 Ĭ��
//char code str4[]="AT+RESET\r\n";                                   	// 	��λ
char code test[]="debug test\r\n";   									//  
#define Buf_Max 32
u8 xdata Rec_Buf[Buf_Max];     //���մ��ڻ�������

BOOL Hand(u8 *RxBuf,u8 *a);
void CLR_Buf(void);
//========================================================================
/*************	ģ�⴮�ڵ�����	**************/
#define BaudRate		9600		//ģ�⴮�ڲ�����
#define Timer0_Reload		(65536 - MAIN_Fosc / BaudRate / 3)
#define RxLength		32		//���ջ��峤��
//���尴�� ģ�⴮��
sbit P_RXB_30 = P3^0;			//����ģ�⴮�ڽ���IO
sbit P_RXB_36 = P3^4;			//����ģ�⴮�ڽ���IO


bit	B_Rx_OK;	 		//������ı�־λ, �յ����ݿ�ϵͳ����1, �û��������ݺ������0			
u8 RX_CONT = 0;  //���յ����ֽ���, �û��������ݺ������0

//=====================================================================
//===== ������ЩΪϵͳʹ�õĳ��������, �û����ɼ��������޸� =============
#define	RxBitLenth	9		//8������λ+1��ֹͣλ
#define	TxBitLenth	9		//8������λ+1��ֹͣλ
u8  TxShiftReg,RxShiftReg;	//���� ���� ��λ
u8  RxSample;		//���ͺͽ��ռ�� ����������(3�����ʼ��)
u8  TxBitCnt,RxBitCnt;		//���ͺͽ��յ����� λ������
u8	RxTimeOut;		//���ճ�ʱ����
bit	RxStartFlag;			//���ڽ���һ���ֽ�(���յ���ʼλ)
u8 uart_flag =0;

//=====================================================================
u16 led_stat_cont = 0;
BOOL Recv_Ok = 0;
BOOL Recv_start =0;

//#define BOOL_WS2811_LED
#if defined(BOOL_WS2811_LED)
#define bit_to_u8(x) rem(x,8) ? (mod(x,8)+1) : mod(x,8)
u16 LED_StatByteSize = bit_to_u8(nWs);
u8 xdata LED_StatByte[bit_to_u8(nWs)];//nWs��ΪLED����
u16 bool_led_count = 0;//���յ�LED���ݵĸ�����
void Clear_LedStatByte(void);
void WS2811_Display(void);
#else
u8 xdata LED_Buff[nWs];
void Clear_LedBuff(void);
#endif
/*************  �ⲿ�����ͱ������� *****************/
void delay_ms(u8 ms)
{
   u16 i;
	 do{
	      i = MAIN_Fosc / 13000;
		  while(--i)	;   //14T per loop
     }while(--ms);
}
void DelayMS(u16 t)
{
	while(t--)
	{
		delay_ms(1);
	}
}


//========================================================================
// ����: void UART_INIT(void)
// ����: UARTģ��ĳ�ʼ����.
// ����: ��.
// ����: ��.
// �汾: V1.0, 2012-10-22
//========================================================================
void UART_INIT(void)
{
	RxStartFlag = 0;
	RxSample = 4;
	RxTimeOut = 0;
	B_Rx_OK = 0;
	RX_CONT = 0;
}
BOOL Hand(u8 *RxBuf,u8 *a)
{ 
  if(strstr(RxBuf,a)!=NULL)           //�ж��ַ���a�Ƿ����ַ���Rec_Buf���Ӵ�
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
	for(k=0;k<Buf_Max;k++)        //�����ڻ��������ֵ����Ϊ��  
	{
		Rec_Buf[k] = 0;
	}
    RX_CONT = 0;                    
}
void timer0_init(void)//����ģ���жϵĶ�ʱ��
{
	Timer0_1T();
	Timer0_AsTimer();
	Timer0_16bitAutoReload();
	Timer0_Load(Timer0_Reload);
	Timer0_InterruptEnable();
	PT0 = 1;	//��ʱ��0�����ȼ��ж�
	Timer0_Run();
	EA = 1;					//�����ж�					open global interrupt switch
}	
void Timer2Init(void)		//
{
	Timer2_AsTimer();//ʹ�ö�ʱ��2
	Timer2_InterruptDisable();
	Timer2_Stop();	
	Timer2_1T();
	T2L = 0x66;		//1ms��ʱ
	T2H = 0x7E;		//1ms��ʱ
	Timer2_InterruptEnable();
	Timer2_Run();
}
/**************** ��̵���HC595����һ���ֽں��� ******************/
void Switch_DataIn595(u8 dat)
{		
	u8	i;
	for(i=0; i<8; i++)
	{
		dat <<= 1;
		B_HC595_SER   = CY;
		B_HC595_SRCLK = 0;
		NOP2();
		B_HC595_SRCLK = 1;
	}
}
/**************** �̵���HC595�������溯�� ******************/
void Switch_DataOut595()
{		
	B_HC595_RCLK = 0;
	NOP2();
	B_HC595_RCLK = 1;
}
/**************** �̵�����ʼ�� ******************/
void Switch_ChannelIint()
{
	Switch_DataIn595(switch_status1);//�رո�8·�̵���
	Switch_DataIn595(switch_status0);//�رյ�8λ�̵���
	Switch_DataOut595();   //����Ĭ�Ϲر�8·�̵���
	//print_string(tx_buffer_vale);ģ�⴮�ڱ����������ָʾ��״̬
}

/*********************����ֵ���Ƽ̵���*********************/
/*Function List: ����ֵ���Ƽ̵���
*switch_cmd���ǿ��Ƶ�ͨ��ֵ�����������İ�����ֵ
*ͨ��ֵ��Ҫȡ�����͵�595��595���������������
*��ʱֻ֧��16λ��
*switch_status0�������8λֵ
*/
void Switch_ChannelCtrl(u8 switch_cmd)
{
	if(switch_cmd < 8)
		reversebit(switch_status0,switch_cmd);//����λȡ��
	else if((switch_cmd-8) < 8)
		reversebit(switch_status1,switch_cmd-8);//����λȡ��
	else {
		
	}
	Switch_DataIn595(switch_status1);//���͸�8λ
	Switch_DataIn595(switch_status0);//���͵�8λ
	Switch_DataOut595();	
	//LED��ʾ���͵�ƽ��ʾ����״̬
	tx_buffer_vale[5] = creat_random();//���������
	tx_buffer_vale[6] = switch_status1 + tx_buffer_vale[5];//�ȷ���λ
	tx_buffer_vale[7] = switch_status0 + tx_buffer_vale[5];//�ٷ���λ
	//Fake_PrintString1(TX_P31,tx_buffer_vale);//ģ�⴮���ȶ�����
}

#if 0
/****************LED_HC595����һ���ֽ� ******************/
void LED_DataIn595(u8 dat)
{		
	u8	i;
	for(i=0; i<8; i++)
	{
		dat >>= 1;//�ȷ���λ
		B_HC595_SER   = CY;
		B_HC595_SRCLK = 0;
		NOP2();
		B_HC595_SRCLK = 1;
	}
}
/**************** LED_HC595��������******************/
void LED_DataOut595()
{		
	B_HC595_RCLK = 0;
	NOP2();
	B_HC595_RCLK = 1;
}
void HC595_Display(void)//�����Ѿ���������ɨ�輴��
{
	u16 i;
	for(i=0;i<LED_StatByteSize;i++){
		LED_DataIn595(LED_StatByte[i]);
	}
	LED_DataOut595();
}
#endif
//========================================================================
// ����: Ӳ����ʼ�� ��
//========================================================================
void stc15x_hw_init(void)
{
	P0n_standard(0xff);	//����Ϊ׼˫���
	P1n_standard(0xff);	//����Ϊ׼˫���
	P2n_standard(0xff);	//����Ϊ׼˫���
	//P3n_standard(0xff);	//����Ϊ׼˫���
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���	

	//InternalRAM_enable();
	timer0_init();
	uart1_config();//��ʱ��1���������ʴ��ڳ�ʼ��,��������
	UART_INIT();				//UARTģ��ĳ�ʼ����
	Timer2Init();//����1ms��ʱ
}
//========================================================================
// ����: set_timer1_baudraye(u16 dat)
// ����: ����Timer1�������ʷ�������
// ����: dat: Timer1����װֵ.
// ����: none.
// �汾: VER1.0
// ����: 2014-11-28
// ��ע: 
//========================================================================
#if defined UART1_TIMER1
void set_timer1_baudraye(u16 dat)
{
	Timer1_Stop();     //Timer stop
	Timer1_AsTimer();  //Timer2 set As Timer
	Timer1_1T();       //Timer2 set as 1T mode
	Timer1_Load(dat);  //����������
	Timer1_InterruptDisable();  //��ֹ�ж�
	Timer1_Run();      //Timer run enable
}
#define	BaudRate1		9600UL	//?????
#define	Timer1_Reload	(65536UL -(MAIN_Fosc / 4 / BaudRate1))		//Timer 1 ???, ??300KHZ
void uart1_config(void)
{
	/*********** ������ʹ�ö�ʱ��1 *****************/

	S1_BRT_UseTimer1();//S1 BRT Use Timer1;
	Timer1_16bitAutoReload(); //��ʱ��1Ϊģʽ0(16λ�Զ�����)
	set_timer1_baudraye(65536UL - (MAIN_Fosc / 4) / Baudrate1);
	//SCON = (SCON & 0x3f) | 0x40;	//UART1ģʽ, 0x00: ͬ����λ���, 0x40: 8λ����,�ɱ䲨����, 0x80: 9λ����,�̶�������, 0xc0: 9λ����,�ɱ䲨����
	S1_SHIFT();
	S1_8bit();
	S1_RX_Enable();
	//S1_USE_P30P31();//UART1 ʹ��P30 P31��	Ĭ��
	S1_USE_P36P37();  //UART1 ʹ��P36_TX P37_RX��
	ES  = 1;	//�����ж�
	REN = 1;	//�������
	EA = 1;
	B_TX1_Busy = 0;
}

void print_char(u8 dat)
{
		SBUF = dat;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
}
void print1_string(u8 *puts)
{
  for (; *puts != 0;	puts++)   	//����ֹͣ��0����
	{
		SBUF = *puts;
		B_TX1_Busy = 1;
		while(B_TX1_Busy);
	}
}
#endif
//cmd��MJX+cmd1+���ݳ���+�����+ͨ��ֵ;
void Bluetooth_DataProce(void)
{
	u8 i=0;
	for(i=0;i<Rec_Buf[ID_LENGTH(SEND_ID)];i++)
	{
		RxData[i] = Rec_Buf[i+ID_LENGTH(SEND_ID)+1];//ȡ�����ݳ����ֽں�������
		if(i==0){
			//do nothing
		}
		else
			RxData[i] = RxData[i] - RxData[0];
	}
	if(RxData[1] == 1)
	{
		Fake_PrintString(TX_P54,"recv success BT05 0x01!\r\n");
	}
	if(RxData[1] == 0x02)
	{
		Fake_PrintString(TX_P54,"recv success BT05 0x02!\r\n");
	}
}
//cmd��MJX+cmd2+���ݳ���+�����+ͨ��ֵ;
void Keyboard_DataProce(void)
{
	u8 i=0;
	
	for(i=0;i<Rec_Buf[ID_LENGTH(SEND_ID)];i++)
	{
		RxData[i] = Rec_Buf[i+ID_LENGTH(SEND_ID)+1];//ȡ�����ݳ����ֽں�������
		if(i==0){
			//do nothing
		}
		else{
			RxData[i] = RxData[i] - RxData[0];
			Switch_ChannelCtrl(RxData[i]);
			//Fake_PrintString(TX_P54,"recv success Key_Code to contral switch!\r\n");
		}
	}	
}

void Rcev_DataAnalysis(void)
{
	u8 recv_cmd = 0;
	if(Hand(Rec_Buf,SEND_ID)&&Hand(Rec_Buf,SEND_END))
	{
		Fake_PrintString(TX_P54,SEND_ID);
		Fake_PrintString(TX_P54,":");
		Fake_PrintString(TX_P54,"\r\n");
		recv_cmd = Rec_Buf[ID_LENGTH(SEND_ID)-1];//����Ϊ�ַ�+\0,��ȡָ��
		switch(recv_cmd)
		{
			case CMD_1://����
					Fake_PrintString(TX_P54,"BT05 CMD!\r\n");	
					Bluetooth_DataProce();
				break;
			case CMD_2://����
					Fake_PrintString(TX_P54,"Key_Code CMD!\r\n");
					Keyboard_DataProce();
				break;
			default:
				Fake_PrintString(TX_P54,"ERROR CMD!\r\n");
				break;
				CLR_Buf();
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
	u8 j,k;
	CLR_Buf();//������ջ���
	Timer0_Stop();
	Clear_WS2811();
	Timer0_Run();
	stc15x_hw_init();
	B_HC595_MR = 1;//��λ��ֹ
	B_HC595_OE = 0;//ʹ��оƬ
	Switch_ChannelIint();//�̵�����ʼ����ȫ�ر�
#if defined(BOOL_WS2811_LED)
	Clear_LedStatByte();//��մ洢BOOL��LED״̬��buff
#else
	Clear_LedBuff();//��մ��Ҷȵ�WS2811״̬�Ļ�������
#endif
	Fake_PrintString(TX_P54,"STC15W408as fake UART init OK!\r\n");	//ģ�⴮��9600������һ���ַ���

	do{
		print1_string(str1);	//����ָ��
		DelayMS(500);
		if(++j>5){
			Fake_PrintString(TX_P54,"BT05 hand failed\r\n");
			j = 0;				//Can add error led display
			break;
		}
	}while(!Hand(Rec_Buf,"OK"));			//�ж��Ƿ����ֳɹ�,������ɹ���ʱһ��,�ٷ���AT����ָ��
	CLR_Buf(); 
	Fake_PrintString(TX_P54,"BT05 hand OK\r\n");
	
	while (1)
	{
	//	Timer0_Stop();
	//	liushui123x(1,60);
	//	Timer0_Run();
		if(Recv_Ok)
		{
			Timer0_Stop();//����WS2811�������ݣ�����ʱ���ᵼ������˸
#if defined(BOOL_WS2811_LED)
			WS2811_Display();
			Clear_LedStatByte();
#else
			for(k = 0;k < nWs;k++){
				WS2811_SendByte(LED_Buff[k]);
			}
			Clear_LedBuff();
#endif
			WS2811_Reset();
			Timer0_Run();
			//key_led_reverse();
			Recv_Ok = 0;
			k = 0;
		}
		if (B_Rx_OK)	//������ı�־λ, �յ����ݿ�ϵͳ����1, �û��������ݺ������0
		{
			if(RX_CONT > 0)	//ȷ��������
			{
			//	for(i=0; i<RX_CONT; i++)	TxSend(TX_P54,Rec_Buf[i]);	//���յ�������ԭ������,���ڲ���
			}			
			if(Hand(Rec_Buf,"MJX"))
			{
				//Fake_PrintString(TX_P54,"STC15W408as fake UART init OK!\r\n");	//ģ�⴮��9600������һ���ַ���
			}
			Rcev_DataAnalysis();
			RX_CONT  = 0;	//����ֽ���
			B_Rx_OK = 0;	//���������ɱ�־
		}

		if(B_1ms)	//1ms��ʱ
		{
			B_1ms = 0;
			if(++msecond >= 1000)	//1s��ʱ
			{	
				msecond = 0;	
				key_led_reverse();
				//Fake_PrintString1(TX_P31,test);//ģ�⴮���ȶ�����
			}
		    if(++cnt50ms >= 50)		//50msɨ��һ�����м���
			{
				cnt50ms = 0;	
			}
			if(++cnt10ms >= 10)		//10msɨ���Ƿ���������Ҫ����
			{
				cnt10ms = 0;
			}
		}
	}
}


//========================================================================
// ����: void tm0(void) interrupt 1
// ����: ��ʱ��0�жϳ���, for UART �Բ�����3�����ٶȲ����ж� ��ʼλ.
// ����: ��.
// ����: ��.
// �汾: V1.0, 2012-10-22
//========================================================================

void timer0_int(void) interrupt TIMER0_VECTOR
{
//====================== ģ�⴮�ڽ��ճ��� ========================================

	if (RxStartFlag)			//�ѽ��յ���ʼλ
	{
		if (--RxSample == 0)			//���������Զ�ʱ����1/3������
		{
			RxSample = 3;               //���ý��ռ�����  ���������Զ�ʱ����1/3������	reset send baudrate counter
			if (--RxBitCnt == 0)		//������һ֡����
			{
				RxStartFlag = 0;        //ֹͣ����			stop receive
				if((P_RXB_30||P_RXB_36) && !B_Rx_OK)	//ȷ��ֹͣλ��ȷ,���һ����ѿ�
				{
					Rec_Buf[RX_CONT] = RxShiftReg;     //�洢���ݵ�������	save the data to RBUF
					RX_CONT++;                                   
					if(RX_CONT>Buf_Max)                          //���������ڶ����������������ʱ�����ǽ�������֮ǰֵ
					{
						RX_CONT = 0;
					}  
					RxTimeOut = 105;				//��ʱ������ֵ, 35��λ��ʱ��(��Ӧ5���ֽ�), �ο�MODBUSЭ��
				}
			}
			else
			{
				RxShiftReg >>= 1;			  //�ѽ��յĵ�b���� �ݴ浽 RxShiftReg(���ջ���)
				if(uart_flag ==1){
					if (P_RXB_30)	RxShiftReg |= 0x80;  //shift RX data to RX buffer
				}
				else if (uart_flag ==2){
					if (P_RXB_36)	RxShiftReg |= 0x80;  //shift RX data to RX buffer
				}
			}
		}
	}
	else if(!P_RXB_36)
	{
		uart_flag = 2;
		RxStartFlag = 1;       //����������ÿ�ʼ���ձ�־λ 	set start receive flag
		RxSample = 4;       //��ʼ�����ղ����ʼ�����       	initial receive baudrate counter
		RxBitCnt = RxBitLenth;       //��ʼ�����յ�����λ��(8������λ+1��ֹͣλ)    initial receive bit number (8 data bits + 1 stop bit)
		
	}
	else if (!P_RXB_30)		//�ж��ǲ��ǿ�ʼλ P_RXB_30=0;
	{
		RxStartFlag = 1;       //����������ÿ�ʼ���ձ�־λ 	set start receive flag
		RxSample = 4;       //��ʼ�����ղ����ʼ�����       	initial receive baudrate counter
		RxBitCnt = RxBitLenth;       //��ʼ�����յ�����λ��(8������λ+1��ֹͣλ)    initial receive bit number (8 data bits + 1 stop bit)
		uart_flag = 1;
	}

	if(RxTimeOut > 0)	//���ճ�ʱ����
	{
		if(--RxTimeOut == 0)	B_Rx_OK = 1;	//��־���յ�һ֡����
	}
}
#if defined(BOOL_WS2811_LED)
u8 Setbit1_InByte(u8 i,u8 ubyte)
{
	setbit(ubyte,i);
	return ubyte;
}
u8 Setbit0_InByte(u8 i,u8 ubyte)
{
	clrbit(ubyte,i);
	return ubyte;
}
//========================================================================
// ����: Write_LedStatBuff(u8 val,u16 i)
// ����: ���ж��յ���led ״̬ת��Ϊ�ֽڵ�bit ��ʾ��
// ����: val �յ�����ֵ��val��u16 i ��i ��LED.
// ����: none.
// �汾: VER1.0
// ����: 2019-10-08
// ��ע: 
//========================================================================
void Write_LedStatBuff(u8 val,u16 i)
{
    u16 mu,yu;
    mu = LED_StatByteSize-mod(i,8)-1;//������������
    yu = rem(i,8);//0-7ѭ��
    if(val > 0)
			LED_StatByte[mu] = Setbit1_InByte(yu,LED_StatByte[mu]);
    else if(!val)
			LED_StatByte[mu] = Setbit0_InByte(yu,LED_StatByte[mu]);
	//printf("mu=%d,a = %d,LED_StatByte =%x\n",mu,yu,LED_StatByte[mu]);
}
void Clear_LedStatByte(void)
{
	u16 i;
	for(i=0;i<LED_StatByteSize;i++)
		LED_StatByte[i] = 0;
}
void WS2811_Display(void)
{
	u16 i;
	u8 val,j;
	for(i=0;i<LED_StatByteSize;i++){
		for(j=0;j<8;j++){
			val = LED_StatByte[LED_StatByteSize-i-1]&(0x01<<j);
			if(val)
				val = 0xfe;
			else
				val = 0x00; 
			WS2811_SendByte(val);
		}
	}
}
#else
//���Ҷȵ�LED ����
void Clear_LedBuff(void)
{	
	u16 i;
	for(i=0;i<nWs;i++)
		LED_Buff[i] = 0;	
}
#endif
#if defined UART1_TIMER1
void uart1_int (void) interrupt UART1_VECTOR
{
	u8 temp;
	Timer0_Stop();

	if(RI)//�����ж�
	{
		RI = 0;
		temp = SBUF;
		if(temp==0x05){
			Recv_start = 1;
		}
		else if(temp == 0xff){
			Recv_start = 0;
			//key_led_reverse();
#if defined(BOOL_WS2811_LED)
			//bool_led_count++;
			//for(;bool_led_count<nWs;bool_led_count++)
			//	Write_LedStatBuff(0,bool_led_count);//�յ���LED·��С��nWs���Զ���ȫ0
			bool_led_count = 0;
#else	
			//while(led_stat_cont<nWs)
			//	LED_Buff[led_stat_cont++] = 0;
			led_stat_cont = 0;
#endif
			Recv_Ok=1;
		}
		else if(Recv_start){ 
#if defined(BOOL_WS2811_LED)	
		Write_LedStatBuff(SBUF,bool_led_count++);//�յ���״̬ת����bit�����뻺������������1
#else
			LED_Buff[led_stat_cont] = SBUF ? SBUF : 0;
			led_stat_cont++;			
#endif
		}
	  if(RX_CONT>Buf_Max)                          //���������ڶ����������������ʱ�����ǽ�������֮ǰֵ
		{
			RX_CONT = 0;
		}           
	}
	if(TI)//�����ж�
	{
		TI = 0;
		B_TX1_Busy = 0;
	}
	Timer0_Run();
}
#endif
/********************* Timer2��ʱ�ж�************************/
void timer2_int (void) interrupt TIMER2_VECTOR
{
	B_1ms = 1;
}
