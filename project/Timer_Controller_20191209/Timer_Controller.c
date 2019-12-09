/**
  ***************************************************************
  * @file   : XX_SLC204S_J3_V1.0 Driver
  * @author ：chengronghe 
  * @version：V1.0
  * @date   : 2019/12/09 19:37:28.50
  * @brief   
  * @attention	
  **************************************************************
  */
    //204S大功率控制器
 /*************	本程序功能说明	**************
程序功能：数码管显示+按键输入+接线开关
1、输出接通和断开时间显示，四位数码管，前两显示接通时间，后两位显示断开时间。
2、输出接通按键Key1,断开按键key2:按住不放1-99连续递增触发，松开手时间值保存。
3、延时模式key3,电动开关，分为双分，单分，双秒。
	双分：以分钟为单位，按照设定的时间接通和断开，循环。
	单分：以分钟为单位，只定一次接通的时间，接通时间到输出断开，重新启动总开关，再重新定时。
	双秒：以秒为单位，按照设定的时间接通和断开，循环。
4、引出接线开关。	


硬件控制：
1、共阳极数码管倒计时：595显示
2、3路继电器：595控制
3、
**********************************************/
/*
 *----------------------------程序日志---------------------------
 * 1，初版框架搭建
 * 2， 
 * 3，				
 * 4，
 * 5，					
 *	  					
 *	
 *
 *---------------------------------------------------------------
 * 更新时间：			2019/12/09 周一 19:56:54.44  (date /t & time)
 *---------------------------------------------------------------
 *****************************************************************
*/
 
#include"stc15wxx_conf.h"

#define GPIO_FILTER_TIME 50 //滤波的“ 稳定时间” 50ms

#define LED_TIME_1S  1000  //时间是 10000ms
#define LED_TIME_60S 60000 //时间是 60000ms
#define LED_TIME_65S 65000 //时间是 65000ms
#define LED_TIME_95S 95000 //时间是 95000ms

static u8 Gu8Step = 0; //软件定时器 1 的 switch 切换步骤

#define	INDEX_MAX 4	//显示位索引
static u8 	Display_Code[3]={0x00,0x00,0x00};		//1个595控制按键板LED灯,两个595数码管倒计时。
static u8 	LED8[4] = {0x00,0x00,0x00,0x00};		//显示缓冲支持四位
static u8	display_index = 0;						//显示位索引

u8 code t_display[]={						//共阴极标准字库，共阳取反
//	 0    1    2    3    4    5    6    7    8    9    A    B    C    D    E    F
	0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F,0x77,0x7C,0x39,0x5E,0x79,0x71,
//black	 -     H    J	 K	  L	   N	o   P	 U     t    G    Q    r   M    y
	0x00,0x40,0x76,0x1E,0x70,0x38,0x37,0x5C,0x73,0x3E,0x78,0x3d,0x67,0x50,0x37,0x6e,
	0xBF,0x86,0xDB,0xCF,0xE6,0xED,0xFD,0x87,0xFF,0xEF,0x46};	//0. 1. 2. 3. 4. 5. 6. 7. 8. 9. -1

typedef struct _TASK_COMPONENTS
{
	u8 Run;                  // 程序运行标记：0-不运行，1运行
	u16 Timer;               // 计时器
	u16 ItvTime;             // 任务运行间隔时间
	void (*TaskHook)(void);  // 要运行的任务函数
} TASK_COMPONENTS;   
static TASK_COMPONENTS TaskComps[] =
{
	{0, 1000,  1000, vTaskfFlashLed},           // 按键扫描1s
//	{0, 10, 10, TaskDisplayScan},         		// 显示时钟,LED 10ms刷新	
//	{0, 50, 50, vKey_Service}					// 按键服务程序50ms
//	{0, 10, 10, TaskRTC}				        // RTC倒计时
//	{0, 30, 30, TaskDispStatus}
	// 这里添加你的任务
};
typedef enum _TASK_LIST
{
	TASK_FLASH_LED,        // 运行LED
//	TAST_DISP_TIME,        // 显示时钟
//	TASK_KEY_SERV,
//	TASK_RTC,
//	TASK_DISP_WS,             // 工作状态显示// 这里添加你的任务。。。。
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
void BitX_Set(int status,u8 Xbit)//数码管com 位设置
{
/*
	u8 val;
	Display_Code[2] = 0x00;//将低四位先清0，再置位，每次只能亮一位
	val = Display_Code[2];
	if(En)
		Display_Code[2] = setbit(val,Xbit);//共阳极高电平导通三极管	
*/
	u8 val;
	//val = Display_Code[2];
	val =0x00;
	if(status==ON)
		Display_Code[2] = setbit(val,Xbit);//高电平导通
	else if(status==OFF)
		Display_Code[2] = clrbit(val,Xbit);	
	else if(ON_ALL == status)	
		Display_Code[2] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[2] = 0x00;
	else if(OFF_ON == status)
		Display_Code[2] = reversebit(val,Xbit);//翻转
}
void Kled_Set(int status,u8 Nled)//按键灯设置
{
	u8 val;
	val = Display_Code[0];
	if(status==ON)
		Display_Code[0] = setbit(val,Nled);//高电平导通
	else if(status==OFF)
		Display_Code[0] = clrbit(val,Nled);	
	else if(ON_ALL == status)	
		Display_Code[0] = 0xff;
	else if(OFF_ALL == status)
		Display_Code[0] = 0x00;
	else if(OFF_ON == status)
		Display_Code[0] = reversebit(val,Nled);//翻转
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







