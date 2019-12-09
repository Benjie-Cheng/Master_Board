/**
  ***************************************************************
  * @file   : XX_SLC204S_J3_V1.0 Driver
  * @author ��chengronghe 
  * @version��V1.0
  * @date   : 2019/12/09 19:37:28.50
  * @brief   
  * @attention	
  **************************************************************
  */
    //204S���ʿ�����
 /*************	��������˵��	**************
�����ܣ��������ʾ+��������+���߿���
1�������ͨ�ͶϿ�ʱ����ʾ����λ����ܣ�ǰ����ʾ��ͨʱ�䣬����λ��ʾ�Ͽ�ʱ�䡣
2�������ͨ����Key1,�Ͽ�����key2:��ס����1-99���������������ɿ���ʱ��ֵ���档
3����ʱģʽkey3,�綯���أ���Ϊ˫�֣����֣�˫�롣
	˫�֣��Է���Ϊ��λ�������趨��ʱ���ͨ�ͶϿ���ѭ����
	���֣��Է���Ϊ��λ��ֻ��һ�ν�ͨ��ʱ�䣬��ͨʱ�䵽����Ͽ������������ܿ��أ������¶�ʱ��
	˫�룺����Ϊ��λ�������趨��ʱ���ͨ�ͶϿ���ѭ����
4���������߿��ء�	


Ӳ�����ƣ�
1������������ܵ���ʱ��595��ʾ
2��3·�̵�����595����
3��
**********************************************/
/*
 *----------------------------������־---------------------------
 * 1�������ܴ
 * 2�� 
 * 3��				
 * 4��
 * 5��					
 *	  					
 *	
 *
 *---------------------------------------------------------------
 * ����ʱ�䣺			2019/12/09 ��һ 19:56:54.44  (date /t & time)
 *---------------------------------------------------------------
 *****************************************************************
*/
 
#include"stc15wxx_conf.h"

#define GPIO_FILTER_TIME 50 //�˲��ġ� �ȶ�ʱ�䡱 50ms

#define LED_TIME_1S  1000  //ʱ���� 10000ms
#define LED_TIME_60S 60000 //ʱ���� 60000ms
#define LED_TIME_65S 65000 //ʱ���� 65000ms
#define LED_TIME_95S 95000 //ʱ���� 95000ms

static u8 Gu8Step = 0; //�����ʱ�� 1 �� switch �л�����

#define	INDEX_MAX 4	//��ʾλ����
static u8 	Display_Code[3]={0x00,0x00,0x00};		//1��595���ư�����LED��,����595����ܵ���ʱ��
static u8 	LED8[4] = {0x00,0x00,0x00,0x00};		//��ʾ����֧����λ
static u8	display_index = 0;						//��ʾλ����

u8 code t_display[]={						//��������׼�ֿ⣬����ȡ��
//	 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,
//black	 -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e,
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	//0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1

typedef struct _TASK_COMPONENTS
{
	u8 Run;                  // �������б�ǣ�0-�����У�1����
	u16 Timer;               // ��ʱ��
	u16 ItvTime;             // �������м��ʱ��
	void (*TaskHook)(void);  // Ҫ���е�������
} TASK_COMPONENTS;   
static TASK_COMPONENTS TaskComps[] =
{
	{0, 1000,  1000, vTaskfFlashLed},           // ����ɨ��1s
//	{0, 10, 10, TaskDisplayScan},         		// ��ʾʱ��,LED 10msˢ��	
//	{0, 50, 50, vKey_Service}					// �����������50ms
//	{0, 10, 10, TaskRTC}				        // RTC����ʱ
//	{0, 30, 30, TaskDispStatus}
	// ��������������
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // ����LED
//	TAST_DISP_TIME,        // ��ʾʱ��
//	TASK_KEY_SERV,
//	TASK_RTC,
//	TASK_DISP_WS,             // ����״̬��ʾ// �������������񡣡�����
	// ��������������
	TASKS_MAX                 // �ܵĿɹ�����Ķ�ʱ������Ŀ
} TASK_LIST;
void TaskRemarks(void)
{
	u8 i;    
	for (i=0; i<TASKS_MAX; i++)           // �������ʱ�䴦��
	{
		if (TaskComps[i].Timer)           // ʱ�䲻Ϊ0
		{
			TaskComps[i].Timer--;         // ��ȥһ������
			if (TaskComps[i].Timer == 0)  // ʱ�������
			{
				TaskComps[i].Timer = TaskComps[i].ItvTime;       // �ָ���ʱ��ֵ��������һ��
				TaskComps[i].Run = 1;           // �����������	
			}
		}
	}
}
void TaskProcess(void)
{
	u8 i;    
	for (i=0; i<TASKS_MAX; i++)           // �������ʱ�䴦��
	{
		if (TaskComps[i].Run)           // ʱ�䲻Ϊ0
		{
			TaskComps[i].TaskHook();         // ��������
			TaskComps[i].Run = 0;          // ��־��0
		}
	}
}
void BitX_Set(int status,u8 Xbit)//�����com λ����
{
/*
	u8 val;
	Display_Code[2] = 0x00;//������λ����0������λ��ÿ��ֻ����һλ
	val = Display_Code[2];
	if(En)
		Display_Code[2] = setbit(val,Xbit);//�������ߵ�ƽ��ͨ������	
*/
	u8 val;
	//val = Display_Code[2];
	val =0x00;
	if(status==ON)
		Display_Code[2] = setbit(val,Xbit);//�ߵ�ƽ��ͨ
	else if(status==OFF)
		Display_Code[2] = clrbit(val,Xbit);	
	else if(ON_ALL == status)	
		Display_Code[2] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[2] = 0x00;
	else if(OFF_ON == status)
		Display_Code[2] = reversebit(val,Xbit);//��ת
}
void Kled_Set(int status,u8 Nled)//����������
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
void main(void)
{
	vInit_MCU();
	while(1)
	{
		TaskProcess();
	}	
}
/********************* Timer2��ʱ�ж�************************/
void timer2_int (void) interrupt TIMER2_VECTOR   //��ʱ��2��Ϊϵͳʱ��
{
	B_1ms = 1;
	TaskRemarks();
}







