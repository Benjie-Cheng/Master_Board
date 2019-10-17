/**
  ***************************************************************
  * @file   : XX_SLC204S_J3_V1.0 Driver
  * @author ：chengronghe 
  * @version：V1.0
  * @date   : 2019/10/16 11:04:18.89
  * @brief   
  * @attention	
  **************************************************************
  */
  //204大功率控制器
/*************	本程序功能说明	**************
程序功能：
1、10ms定时刷新缓存数组，595送显。


硬件控制：
1、共阳极数码管倒计时：595显示
2、3路继电器：595控制
**********************************************/
/****************************************************************

 *----------------------------程序日志---------------------------
 * 1，初版框架搭建
 * 2， 
 *					
 * 4，
 * 5，					
 *	  					
 *	
 *
 *---------------------------------------------------------------
 * 更新时间：			2019/10/16 11:04:18.89
 *---------------------------------------------------------------
 *****************************************************************
*/

#include"stc15wxx_conf.h"

/*************	本地变量声明	**************/
BOOL B_1ms;	 //1ms标志

//数码管公共端控制
#define COM1 0
#define COM2 1
#define COM3 2
#define COM4 3
// 继电器控制
#define L1 4
#define L2 5
#define L3 6
//LED 控制
#define DEBUG_LED 7
//小数点
#define DIS_DOT 7
//display_code[0]:数码管断码，display_code[1]: H4（继电器+LED）L4（1111com位）
static u8 Display_Code[2]={0x00,0x00};//两个595显示数据。
u8 	LED8[4] = [0,1,2,3];		//显示缓冲
u8	display_index = 0;	//显示位索引
u8 code t_display[]={						//共阴极标准字库，共阳取反
//	 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,
//black	 -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e,
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	//0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1
/**	
//EEPROM 数据格式：
01:时--------倒计时
02:分--------倒计时
03:LED灯个数
04:流水花样
05:延时长度--时间
06:(1,2);2 defalut
*/
enum {
	HourBit,
	MinuBit,
	LedsBit,
	ModeBit,
	SpeedBit
}
u8 code EEPROM_DATA_DEF[5]={4,0,20,1,10};//4h00min,20个led,模式1,速度10
u8 xdata EEPROM_DATA[10]={0};
u8 Mode_KeyStep = 0;
u8 LED_Speed = 0;
BOOL Mode_Seting = FALSE;
#define Mode_KeyStepMin 0
#define Mode_KeyStepMax 3
#define LED_SpeedMax    100
#define LED_SpeedMin    1

void ComX_Set(BOOL en,u8 Xcom);
void RelayX_Set(BOOL en,u8 Xrelay);
void Debug_LedSet(BOOL en);
void Dis_DotLedSet(BOOL en,BOOL flash);
//void DisplayChange(void);

void TaskRemarks(void);
void TaskDisplayScan(void);
//void TaskKeySan(void);
void TaskDispStatus(void);
typedef struct _TASK_COMPONENTS
{
	u8 Run;                 // 程序运行标记：0-不运行，1运行
	u8 Timer;               // 计时器
	u8 ItvTime;             // 任务运行间隔时间
	void (*TaskHook)(void); // 要运行的任务函数
} TASK_COMPONENTS;   
static TASK_COMPONENTS TaskComps[] =
{
	{0, 10, 10, TaskDisplayScan},         		// 显示时钟,继电器控制，LED 10ms刷新
	{0,  2,  2, vTaskKeySan},               	// 按键扫描2ms
	{0, 50, 50, vKey_Service}					// 按键服务程序50ms
	{0, 30, 30, TaskDispStatus}
	// 这里添加你的任务
};
typedef enum _TASK_LIST
{
	TAST_DISP_LED,            // 显示时钟
	TAST_KEY_SCAN,             // 按键扫描
	TASK_KEY_SERV,
	TASK_DISP_WS,             // 工作状态显示// 这里添加你的任务。。。。
	// 这里添加你的任务
	TASKS_MAX                 // 总的可供分配的定时任务数目
} TASK_LIST;
void TaskRemarks(void)
{
	u8 i;    
	for (i=0; i<TASKS_MAX; i++)           // 逐个任务时间处理
	{
		if (TaskComps[i].Timer)           // 时间不为0
		{
			TaskComps[i].Timer--;         // 减去一个节拍
			if (TaskComps[i].Timer == 0)  // 时间减完了
			{
				TaskComps[i].Timer = TaskComps[i].ItvTime;       // 恢复计时器值，从新下一次
				TaskComps[i].Run = 1;           // 任务可以运行	
			}
		}
	}
}
void TaskProcess(void)
{
	u8 i;    
	for (i=0; i<TASKS_MAX; i++)           // 逐个任务时间处理
	{
		if (TaskComps[i].Run)           // 时间不为0
		{
			TaskComps[i].TaskHook();         // 运行任务
			TaskComps[i].Run = 0;          // 标志清0
		}
	}
}
/********************** 显示扫描函数 ************************/
void TaskDisplayScan(void)
{ 
	for(display_index=0;display_index<4;display_index++){
		//DisplayChange();//赋值缓存数值，并改变数码管com位
		ComX_Set(1,display_index);//com 口扫描，点亮该位
		Display_Code[0] = LED8[display_index];
										//送显即可
		vDataIn595(Display_Code[1]);	//输出位码+继电器+LED
		vDataIn595(Display_Code[0]);	//输出段码
		vDataOut595();		            //锁存输出数据
	}
}
/********************** 按键服务函数 ************************/
void vKey_Service(void) //按键服务的应用程序
{
	u16 temp;
	switch(ucKeySec) //按键服务状态切换
	{
		case 1:// Speed键的短按 
		
			if(Mode_Seting){
				switch(Mode_KeyStep)
				{
					case 1:
						temp = EEPROM_DATA[0];
						EEPROM_DATA[0] = temp+1;//小时+1
						if(EEPROM_DATA[0] >24)
							EEPROM_DATA[0] = 0;
						break;
					case 2:
						temp = EEPROM_DATA[1];
						EEPROM_DATA[1] = temp+1;//分+1
						if(EEPROM_DATA[1] >59)
							EEPROM_DATA[1] = 0;
						break;	
					case 3:
						temp = EEPROM_DATA[2];
						EEPROM_DATA[2] = temp+8;//LED+1
						if(EEPROM_DATA[2] >2000)
							EEPROM_DATA[2] = 2000;
						break;						
				}
			}
			else{
				if(++LED_Speed > LED_SpeedMax)
					LED_Speed = LED_SpeedMin;
				EEPROM_DATA[2] = LED_Speed;//速度放置缓存，准备写入EEPROM
			}
			ucKeySec=0;  
			break;        
		case 2:// Speed键的长按  
			ucKeySec=0;  
			break;      
		case 3:// Mode键的短按 
			ucKeySec=0;  
			break;        
		case 4:// Mode键的长按
			Mode_Seting = TRUE;
			if(++Mode_KeyStep>Mode_KeyStepMax)
				Mode_KeyStep = Mode_KeyStepMin;
			switch(Mode_KeyStep){
				case 1://时写入，第一位显示H
					LED8[0] = t_display[18];
					break; 
				case 2://分写入,第一位显示F
					LED8[0] = t_display[15];
					break;
				case 3://LED个数写入,第一位显示L
					LED8[0] = t_display[21];
					break;		
			}
				ucKeySec=0;  
			break;
		case 5:// OK键的短按
			break;
		case 6:// OK键的长按
			Mode_Seting = FALSE;
			Mode_KeyStep = 0;
			ucKeySec=0;
			break;   
		case 7:// DEL键的短按
			if(Mode_Seting){
				switch(Mode_KeyStep)
				{
					case 1:
						temp = EEPROM_DATA[0];
						EEPROM_DATA[0] = temp-1;//小时+1
						if(EEPROM_DATA[0] < 0)
							EEPROM_DATA[0] = 23;
						break;
					case 2:
						temp = EEPROM_DATA[1];
						EEPROM_DATA[1] = temp-1;//分+1
						if(EEPROM_DATA[1] < 0)
							EEPROM_DATA[1] = 59;
						break;	
					case 3:
						temp = EEPROM_DATA[2];
						EEPROM_DATA[2] = temp-8;//LED+1
						if(EEPROM_DATA[2] < 10)
							EEPROM_DATA[2] = 10;
						break;						
				}
			}
			else{
				;
			}
			break;
		case 8:// DEL键的长按
			ucKeySec=0;
			break;  
	} 	
}
void TaskDispStatus(void)
{ 
	//printf("---------TaskDispStatus\n");
}
void vMian_Init(void)
{
	//开机读取EEPROM 参数
	if(EEPROM_DATA[5]  == 1){//写过EEPROM,去读EEPROM
		nWs = EEPROM_DATA[LedsBit];
		LED_Speed = EEPROM_DATA[SpeedBit];
	}
	else if(EEPROM_DATA[5]  == 2){//未写过，使用默认参数。
		nWs = EEPROM_DATA[LedsBit];
		LED_Speed = EEPROM_DATA[SpeedBit];
	}
}

void main(void)
{
	vInit_MCU();
	while(1)
	{
		TaskProcess();
	}
	
}

/********************* Timer2定时中断************************/
void timer2_int (void) interrupt TIMER2_VECTOR   //定时器2作为系统时钟
{
	B_1ms = 1;
	TaskRemarks();
}





void ComX_Set(BOOL en,u8 Xcom)
{
	u8 val;
	val = display_code[1] & 0xf0;//将低四位先清0，再置位，每次只能亮一位
	if(en)
		display_code[1] = setbit(val,Xcom);//共阳极高电平导通三极管	
}
void RelayX_Set(BOOL en,u8 Xrelay)
{
	u8 val;
	val = display_code[1];
	if(en)
		display_code[1] = setbit(val,Xrelay);//高电平导通三极管，继电器工作
	else
		display_code[1] = clrbit(val,Xrelay);		
}
void Debug_LedSet(BOOL en)
{
	u8 val;
	val = display_code[1];
	if(en)
		display_code[1] = setbit(val,DEBUG_LED);//高电平导通三极管,点亮LED。（设计上兼容4路继电器）
	else
		display_code[1] = clrbit(val,DEBUG_LED);		
}
void Dis_DotLedSet(BOOL en,BOOL flash)
{
	u8 val;
	val = ~display_code[0];//共阳极数码管，取反后操作
	if(flash){
		display_code[0] = reversebit(val,DIS_DOT);
		return;
	}
	if(en)
		display_code[0] = clrbit(val,DIS_DOT);//共阳极,低电平点亮小数点LED。（设计上兼容4路继电器）
	else
		display_code[0] = setbit(val,DIS_DOT);		
}
/*
void DisplayChange(void)
{
	ComX_Set(1,display_index);//com 口扫描，点亮该位
	Display_Code[0] = LED8[display_index];
	//Dis_DotLedSet(1,1);//闪烁
	if(++display_index >= 4)	display_index = 0;	//4位结束回0	
}
*/


