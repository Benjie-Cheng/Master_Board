#include<stdio.h>
#include <stdlib.h>

typedef 	unsigned int	u16;
typedef 	unsigned char	u8;

void TaskRemarks(void);
void TaskDisplayScan(void);
void TaskKeySan(void);
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
	{0, 10, 10, TaskDisplayScan},         		// 显示时钟
	{0, 20, 20, TaskKeySan},               		// 按键扫描
	{0, 30, 30, TaskDispStatus}
	// 这里添加你的任务
};
typedef enum _TASK_LIST
{
	TAST_DISP_LED,            // 显示时钟
	TAST_KEY_SAN,             // 按键扫描
	TASK_DISP_WS,             // 工作状态显示// 这里添加你的任务。。。。
	// 这里添加你的任务
	TASKS_MAX                 // 总的可供分配的定时任务数目
} TASK_LIST;
void TaskRemarks(void)
{
	u8 i;    
	for (i=0; i<TASKS_MAX; i++)          // 逐个任务时间处理
	{
		if (TaskComps[i].Timer)          // 时间不为0
		{
			TaskComps[i].Timer--;         // 减去一个节拍
			if (TaskComps[i].Timer == 0)       // 时间减完了
			{
				TaskComps[i].Timer = TaskComps[i].ItvTime;       // 恢复计时器值，从新下一次
				TaskComps[i].Run = 1;           // 任务可以运行	
			}
			printf("TaskComps[%d].Run=%d\n",i,TaskComps[i].Run);
		}
	}
	//printf("\n");
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
void TaskDisplayScan(void)
{ 
	printf("---------TaskDisplayScan\n");
}
void TaskKeySan(void)
{ 
	printf("---------TaskKeySan\n");
}
void TaskDispStatus(void)
{ 
	printf("---------TaskDispStatus\n");
}
void main(void)
{
	u8 i;
	do{
		
		TaskRemarks();
		TaskProcess();             // 任务处理
		printf("i = %d\n",i);
		i++;
	}
	while(i<61);
}
