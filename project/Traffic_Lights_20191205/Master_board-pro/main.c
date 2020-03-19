/*---------------------------------------------------------------------*/
/* --- Fuctin: Mini traffic lights- -----------------------------------*/
/* --- Author: ronghe.cheng -------------------------------------------*/
/* --- version: V1.0 Copyright (2019) by chengronghe ------------------*/
/* --- Mobile: (86)15901856750 ----------------------------------------*/
/* --- mail:  1017649596@qq.com ---------------------------------------*/
/* ���Ҫ�ڳ�����ʹ�ô˴���,���ڳ�����ע��ʹ���˰�Ȩ ---------------   */
/*---------------------------------------------------------------------*/
/*************	��������˵��	**************
��ͨ�ƣ�0.28�繲��������ܣ�TXD,RXD P32��P55�����Զ��忪����
��Ƭ����STC15W201AS-SOP16,���ö�̬ɨ�跽ʽ��ԼIO
�����ʣ�115200
������־��
1��
2��
3��
4��
******************************************/
#include "config.h"
#include "debug.h"
#define	Timer0_Reload	(65536UL -(MAIN_Fosc / 1000))	//Timer 0 �ж�Ƶ��, 1000��/��
volatile unsigned char vGu8TimeFlag_1=0;
volatile u32 vGu32TimeCnt_1=0;	
//bit B_1ms;	 //1ms��־
#define LED_TIME_1S  1000  //ʱ���� 10000ms
/*-----------------�Զ���ʱ������---------------------*/
#define RIGHT_LED_TIME 15
#define FRONT_LED_TIME 25
#define LEFT_LED_TIME 35	
#define LowTimeS 5
/*----------------------------------------------------*/
volatile unsigned int g_count = LEFT_LED_TIME ;
volatile unsigned int y_count = FRONT_LED_TIME;
volatile unsigned int r_count = RIGHT_LED_TIME;
BOOL FlashFlag = FALSE;
BOOL LowTime = FALSE;

static u8 Gu8Step = 0; //�����ʱ�� 1 �� switch �л�����

#define	INDEX_MAX 3	//��ʾλ����
static u8 	Display_Code[1]={0x00};		    //1��com ��״̬;
static u8 	LED8[3] = {0x00,0x00,0xff};		//��ʾ����֧��2λ,ʮλ����λ,LED״̬��
static u8	display_index = 0;				//��ʾλ����
/*************	IO����	**************/
sbit	LED8_A = P1^1;
sbit	LED8_B = P1^0;
sbit	LED8_C = P1^5;
sbit	LED8_D = P1^4;
sbit	LED8_E = P1^3;
sbit	LED8_F = P1^2;
sbit	LED8_G = P5^4;

sbit	P_COM0 = P3^6;
sbit	P_COM1 = P3^7;
sbit	P_COM2 = P3^3;

enum{
	LEDL_RED=5, 
	LEDL_GREEN=1,
	LEDR_RED=3, 
	LEDR_GREEN=2, 
	LEDG_RED=0,
	LEDG_GREEN=4,  	
};
typedef enum{
	ON=1,
	OFF=0,
	ON_ALL=0xff,
	OFF_ALL=0xfe,
	OFF_ON=0xfd	
}LogicType;

u8 code t_display[]={						//������
//	 0    1    2    3    4    5    6    7    8    9
	0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90,
//black
	0xff,
};	
//========================================================================
// ����: Ӳ����ʼ�� ��
//========================================================================
void timer0_init(void)
{
	AUXR = 0x80;	//Timer0 set as 1T, 16 bits timer auto-reload, 
	TH0 = (u8)(Timer0_Reload / 256);
	TL0 = (u8)(Timer0_Reload % 256);
	ET0 = 1;	//Timer0 interrupt enable
	TR0 = 1;	//Tiner0 run
	EA = 1;		//�����ж�
}	
void stc15x_hw_init(void)
{
	P0n_standard(0xff);	//����Ϊ׼˫���
	//P1n_standard(0xff);	//����Ϊ׼˫���
	P2n_standard(0xff);//����Ϊ׼˫���
	P1n_push_pull(0xff);//����Ϊǿ����
	P3n_push_pull(0xc8);//����Ϊǿ����
	P4n_standard(0xff);	//����Ϊ׼˫���
	P5n_standard(0xff);	//����Ϊ׼˫���	
	timer0_init();
}
void Kled_Set(LogicType type,u8 Nled)//LED����
{
	u8 val;
	val = LED8[2];
	if(type==ON)
		LED8[2] = setbit(val,Nled);//�ߵ�ƽ��ͨ
	else if(type==OFF)
		LED8[2] = clrbit(val,Nled);	
	else if(ON_ALL == type)	
		LED8[2] = 0xff;
	else if(OFF_ALL == type)
		LED8[2] = 0x00;
	else if(OFF_ON == type)
		LED8[2] = reversebit(val,Nled);//��ת
	//LED8[2] = ~LED8[2];
}
void BitX_Set(LogicType type,u8 Xbit)
{
	u8 val=0;
	if(type==ON)
		Display_Code[0] = setbit(val,Xbit);//�ߵ�ƽ��ͨ
	else if(type==OFF)
		Display_Code[0] = clrbit(val,Xbit);	
	else if(ON_ALL == type)	
		Display_Code[0] = 0xff;
	else if(OFF_ALL == type)
		Display_Code[0] = 0x00;
	else if(OFF_ON == type)
		Display_Code[0] = reversebit(val,Xbit);//��ת
	
}
void TaskDisplayScan(void)//10ms ˢ��һ��
{ 
	if(FlashFlag)
	{
		P_COM0 = 0;
		P_COM1 = 0;
		P_COM2 = 0;
		return;
	}
	for(display_index = 0;display_index < INDEX_MAX;display_index++){
		BitX_Set(ON,display_index);//com ��ɨ�裬������λ
#if 0
	_nop_();
	_nop_();
	BitX_Set(OFF,display_index);//�ı�����
#endif	
		P_COM0=P_COM1=P_COM2=0;//��ʾ����ǰ�ȹر�λ����Ӱ;
		_nop_();
		//������ʾ����ͨ����ʾ
		LED8_A = getbit(LED8[display_index],0);
		LED8_B = getbit(LED8[display_index],1);
		LED8_C = getbit(LED8[display_index],2);
		LED8_D = getbit(LED8[display_index],3);
		LED8_E = getbit(LED8[display_index],4);
		LED8_F = getbit(LED8[display_index],5);
		LED8_G = getbit(LED8[display_index],6);

		//�������ʾ
		P_COM0 = getbit(Display_Code[0],0);
		P_COM1 = getbit(Display_Code[0],1);
		P_COM2 = getbit(Display_Code[0],2);
	}
}
void Traffic_Led(void)
{
	switch(Gu8Step)
	{
		case 0://��ת��35s
			vGu8TimeFlag_1 = 1;
			//Turn_Direction(Gu8Step);
			LED8[2]=0x34;
			r_count = RIGHT_LED_TIME;
			LED8[0] = t_display[mod(g_count,10)];
			LED8[1] = t_display[rem(g_count,10)];
			if(g_count<=LowTimeS)
				LowTime = TRUE;
			else
				LowTime = FALSE;
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				g_count--;
				vGu32TimeCnt_1 = 0;
				if(g_count == 0)
					Gu8Step++;
			}	
			break;
		case 1://��ǰ,25s
			g_count = LEFT_LED_TIME;
			//g_count = FRONT_LED_TIME;
			//Turn_Direction(Gu8Step);
			LED8[2]=0x07;
			LED8[0] = t_display[mod(y_count,10)];
			LED8[1] = t_display[rem(y_count,10)];	
			if(y_count<=LowTimeS)
				LowTime = TRUE;
			else
				LowTime = FALSE;
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				y_count--;	
				vGu32TimeCnt_1 = 0;
				if(y_count == 0)
					Gu8Step++;
			}
			break;
		case 2://����,15s
			y_count = FRONT_LED_TIME;
			LED8[2]=0x1A;
			//Turn_Direction(Gu8Step);
			LED8[0] = t_display[mod(r_count,10)];
			LED8[1] = t_display[rem(r_count,10)];	
			if(r_count<=LowTimeS)
				LowTime = TRUE;
			else
				LowTime = FALSE;
			if(vGu32TimeCnt_1>=LED_TIME_1S)
			{
				r_count--;		
				vGu32TimeCnt_1 = 0;
				if(r_count == 0)
				{
					Gu8Step=0;		
					vGu32TimeCnt_1 = 0;
					vGu8TimeFlag_1 = 0;
				}
			}
			break;	
		default:	
			break;
	}
}
void main(void)
{
	stc15x_hw_init();
	while(1)
	{
		Traffic_Led();//ָʾ��״̬�л�	
		TaskDisplayScan();
	}
}
//========================================================================
// ����: Timer0 1ms�жϺ�����
//========================================================================
void timer0 (void) interrupt TIMER0_VECTOR
{
	//B_1ms = 1;		//1ms��־
	if(vGu8TimeFlag_1){
		vGu32TimeCnt_1++;
	}
	else
		vGu32TimeCnt_1 = 0;
}
