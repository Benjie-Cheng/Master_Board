/**
  ***************************************************************
  * @file   : XX_SLC204S_J3_V1.0 Driver
  * @author ：chengronghe 
  * @version：V1.0
  * @date   : 2019/10/17 15:49:32.52
  * @brief   
  * @attention	
  **************************************************************
  */
    //204大功率控制器
/*************	本程序功能说明	**************
程序功能：
1、10ms定时刷新缓存数组，595送显。
2、


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
 * 更新时间：			2019/10/17 周四  14:13:06.08
 *---------------------------------------------------------------
 *****************************************************************
*/

#include"stc15wxx_conf.h"

/*************	本地变量声明	**************/
BOOL B_1ms;	 //1ms标志
int8 hour,minu;
u8 rtc_conut = 0;

//display_code[0]:数码管断码，display_code[1]: H4（继电器+LED）L4（1111com位）
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
static u8 Display_Code[2]={0x00,0x00};//两个595显示数据。
u8 	LED8[4] = [0,1,2,3];		//显示缓冲
u8	display_index = 0;	//显示位索引
#define	INDEX_MAX 4	//显示位索引
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
	SpeedBit,
	WriteBit,
};
u8 code EEPROM_DATA_DEF[6]={4,0,20,1,10,0};//4h00min,20个led,模式1,速度10
u8 xdata EEPROM_DATA[10]={0};
u8 Mode_KeyStep = 0;
u8 LED_Speed = 0;
BOOL Mode_Seting = FALSE;
BOOL Write_KeyOk = FALSE;
BOOL Fash_DisplayFlag = FALSE;
#define Mode_KeyStepMin 0
#define Mode_KeyStepMax 3
#define LED_SpeedMax    100
#define LED_SpeedMin    1

u8 Relay_State = 0xff;

void ComX_Set(BOOL en,u8 Xcom);
void RelayX_Set(BOOL en,u8 Xrelay);
void Debug_LedSet(BOOL en);
void Dis_DotLedSet(BOOL en,BOOL flash);
//void DisplayChange(void);

void TaskRemarks(void);
void TaskDisplayScan(void);
//void TaskKeySan(void);
void TaskDispStatus(void);
void Write_Eeprom(void);
void vMain_Init(void);
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
	{0, 10, 10, TaskRTC}				        // RTC倒计时
	{0, 30, 30, TaskDispStatus}
	// 这里添加你的任务
};
typedef enum _TASK_LIST
{
	TAST_DISP_LED,            // 显示时钟
	TAST_KEY_SCAN,             // 按键扫描
	TASK_KEY_SERV,
	TASK_RTC,
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
void TaskRTC(void)//10ms once
{
	u16 Temp;

	if(Write_KeyOk){
		vMain_Init();//如果重新写入了EPPROM,将重新读取EEPROM中的值。
		Write_KeyOk = FALSE;
		rtc_conut = 0;
	}
	if(Reset_Work)
	{
		Reset_Work = FALSE;
		vMain_Init();//如果复位将重新读取EEPROM中的值。
		rtc_conut = 0;
	}
	
	Temp = hour * 60 + minu;
	
	if(rtc_conut == 50)//50*10ms
		Fash_DisplayFlag = ~Fash_DisplayFlag;//闪烁标记
	
	if(++rtc_conut > 100){//1000ms 减一分钟
		Fash_DisplayFlag = ~Fash_DisplayFlag;//闪烁标记
		if(!Mode_Seting)//设置的时候不允许倒计时
			Temp--;
		rtc_conut = 0;
		if(temp == 0){
			//倒计时时间到，关闭工作
			return;
		}
	}
	hour = mod(Temp,60);
	minu = rem(temp,60);
		
	if(!Mode_Seting}{//非设置模式正常倒计时
		LED8[0] = ~t_display[mod(hour,10)];
		LED8[1] = ~t_display[rem(hour,10)];
		LED8[2] = ~t_display[mod(minu,10)];
		LED8[3] = ~t_display[rem(minu,10)];
		if((mod(hour,10))==0)
			LED8[0] = 0xff;//关闭最高位
		return;
	}else{//设置模式闪烁显示
		if(Fash_DisplayFlag){//0.5s闪烁第一位
			LED8[0] = 0xff;//灭
			switch(Mode_KeyStep){
				case 1://时写入，第一位显示H
					LED8[1] = 0xff;//第二位不显示
					LED8[2] = ~t_display[mod(hour,10)];
					LED8[3] = ~t_display[rem(hour,10)];
					break; 
				case 2://分写入,第一位显示F
					LED8[1] = 0xff;//第二位不显示
					LED8[2] = ~t_display[mod(minu,10)];
					LED8[3] = ~t_display[rem(minu,10)];
					break;
				case 3://LED个数写入,第一位显示L
					LED8[1] = ~t_display[mod(nWs,100)];
					LED8[2] = ~t_display[(nWs%10/10)];
					LED8[3] = ~t_display[rem(nWs,10)];
					break;		
			}
		}else{
			switch(Mode_KeyStep){
				case 1://时写入，第一位显示H
					LED8[0] = ~t_display[18];
					LED8[1] = 0xff;//第二位不显示
					LED8[2] = ~t_display[mod(hour,10)];
					LED8[3] = ~t_display[rem(hour,10)];
					break; 
				case 2://分写入,第一位显示F
					LED8[0] = ~t_display[15];
					LED8[1] = 0xff;//第二位不显示
					LED8[2] = ~t_display[mod(minu,10)];
					LED8[3] = ~t_display[rem(minu,10)];
					break;
				case 3://LED个数写入,第一位显示L
					LED8[0] = ~t_display[21];
					LED8[1] = ~t_display[mod(nWs,100)];
					LED8[2] = ~t_display[(nWs%10/10)];
					LED8[3] = ~t_display[rem(nWs,10)];
					break;		
			}
		}
	}	
}
void TaskDisplayScan(void)
{ 
	for(display_index = 0;display_index < INDEX_MAX;display_index++){
		//DisplayChange();//赋值缓存数值，并改变数码管com位
		ComX_Set(1,display_index);//com 口扫描，点亮该位
#if 0
	nop();
	nop();
	ComX_Set(0,display_index);//改变亮度
	nop();
#endif		
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
						hour++;//小时+1
						if(hour >24)
							hour = 0;
						break;
					case 2:
						minu ++;
						if(minu >59)//分+1
							minu = 0;
						break;	
					case 3:
						nWs = nWs+8;//LED+1
						if(nWs >2000)
							nWs = 2000;
						break;						
				}
			}
			else{
				if(++LED_Speed > LED_SpeedMax)
					LED_Speed = LED_SpeedMin;
				//EEPROM_DATA[2] = LED_Speed;//速度放置缓存，准备写入EEPROM
			}
			ucKeySec=0;  
			break;        
		case 2:// Speed键的长按  
			ucKeySec=0;  
			break;      
		case 3:// Mode键的短按
			//复位
			ucKeySec=0;  
			break;        
		case 4:// Mode键的长按
			Mode_Seting = TRUE;
			if(++Mode_KeyStep>Mode_KeyStepMax)
				Mode_KeyStep = Mode_KeyStepMin;
			/*
			switch(Mode_KeyStep){
				case 1://时写入，第一位显示H
					LED8[0] = ~t_display[18];
					break; 
				case 2://分写入,第一位显示F
					LED8[0] = ~t_display[15];
					break;
				case 3://LED个数写入,第一位显示L
					LED8[0] = ~t_display[21];
					break;		
			}
			*/
				ucKeySec=0;  
			break;
		case 5:// OK键的短按
			Reset_Work = TRUE;//短按复位
			ucKeySec=0;
			break;
		case 6:// OK键的长按
			Write_Eeprom();//EEPROM 写参数
			Mode_Seting = FALSE;
			Write_KeyOk = TRUE;
			ucKeySec=0;
			break;   
		case 7:// DEL键的短按
			if(Mode_Seting){
				switch(Mode_KeyStep)
				{
					case 1:
						hour --;//小时-1
						if(hour<0)
							hour = 23;
						break;
					case 2:
						minu-- ;//分-1
						if(minu<0)
							minu =59;
						break;	
					case 3:
						nWs = nWs-8;//LED-8
						if(nWs < 10)
							nWs = 10;
						break;						
				}
			}
			else{
				;
			}
			break;
		case 8:// DEL键的长按
			if(++Relay_State > 4)
				LED_Speed = 0;
				switch(Relay_State)
				{
					case 1:
						RelayX_Set(1,L2);
						RelayX_Set(0,L3);
						break;
					case 2:
						RelayX_Set(0,L2);
						RelayX_Set(1,L3);
						break;	
					case 3:
						RelayX_Set(1,L2);
						RelayX_Set(1,L3);
						break;	
					case 0:
						RelayX_Set(0,L2);
						RelayX_Set(0,L3);
						break;						
				}	
			ucKeySec=0;
			break;  
	} 	
}
void TaskDispStatus(void)
{ 
	//printf("---------TaskDispStatus\n");
}
void vMain_Init(void)
{
	//开机读取EEPROM 参数
	//Add
	if(EEPROM_DATA[WriteBit]  == 1){//写过EEPROM,去读EEPROM
		hour = EEPROM_DATA[HourBit];
		minu = EEPROM_DATA[MinuBit];
		nWs = EEPROM_DATA[LedsBit];
		//EEPROM_DATA[ModeBit];
		LED_Speed = EEPROM_DATA[SpeedBit];
	}
	else if(EEPROM_DATA[WriteBit]  != 1){//未写过，使用默认参数。
		hour = EEPROM_DATA_DEF[HourBit];
		minu = EEPROM_DATA_DEF[MinuBit];
		nWs = EEPROM_DATA_DEF[LedsBit];
		//EEPROM_DATA[ModeBit];
		LED_Speed = EEPROM_DATA_DEF[SpeedBit];
	}
}
void Write_Eeprom(void)
{
	if(Mode_Seting){//时分和个数需要在设置模式下才能设置
		EEPROM_DATA[HourBit] = hour;
		EEPROM_DATA[MinuBit] = minu;
		EEPROM_DATA[LedsBit] = nWs;
		EEPROM_DATA[ModeBit] = 1;
		EEPROM_DATA[SpeedBit] = LED_Speed;
		EEPROM_DATA[WriteBit] = 1;//写入标志
	}else{
		return;
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


