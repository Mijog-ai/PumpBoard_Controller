/********************************************************************************************************
*	@file 	    Task.c
*	@brief
*	@author     
*	@version    
*	@date       
*	@warning
*********************************************************************************************************/
#include "application.h"
#include "Task.h"
#include "Hw_ADC.h"
/******************************************* variable define ********************************************/
volatile unsigned int u32_TimeTick_1ms = 0;
volatile unsigned int u32_TimeTick_5ms = 0;
volatile unsigned int u32_TimeTick_10ms = 0;
volatile unsigned int u32_TimeTick_100ms = 0;


/******************************************* function define ********************************************/

/*********************************************************************************************************
 * FunctionName   : Task1ms_v_g()
 * Description    : 1ms任务处理，用作时间管理，不执行其他任务。
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void Task1ms_v_g(void)
{	

}

/*********************************************************************************************************
 * FunctionName   : Task5ms_v_g()
 * Description    : 5ms任务处理
 * EntryParameter : None
 * ReturnValue    : None
***********************************************************************************************************/
void Task5ms_v_g(void)
{
	UpdateMapCmdData(&Angle_Loop, &Pressure_Loop);
    Save_Parameter_Process();//
}
/*********************************************************************************************************
 * FunctionName   : Task10ms_v_g()
 * Description    : 10ms任务处理
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void Task10ms_v_g(void)
{

}
/*********************************************************************************************************
 * FunctionName   : Task100ms_v_g()
 * Description    : 100ms任务处理
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void Task100ms_v_g(void)
{

}
/*********************************************************************************************************
* Description    : 任务组件创建及初始化
*********************************************************************************************************/
static TASK_COMPONENTS TaskComps[] =
{
    {0, 10,   10,    Task1ms_v_g},
    {0, 50,   50,    Task5ms_v_g},
    {0, 100,  100,   Task10ms_v_g},
    {0, 1000, 1000,  Task100ms_v_g}
};
/*********************************************************************************************************
 * FunctionName   : TaskProcess()
 * Description    : 任务处理
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void TaskParmInit(void)
{
	unsigned char i;

    for (i=0; i<TASKS_MAX; i++)
    {
        TaskComps[i].Timer = TaskComps[i].ItvTime;
        TaskComps[i].Run = 0;
    }
}
/*********************************************************************************************************
 * FunctionName   : TaskProcess()
 * Description    : 任务处理
 * EntryParameter : None
 * ReturnValue    : None
*********************************************************************************************************/
void TaskProcess(void)
{
    unsigned char i;

    for (i=0; i<TASKS_MAX; i++)
    {
        if (TaskComps[i].Run)
        {
            TaskComps[i].TaskHook();
            TaskComps[i].Run = 0;
        }
    }
}
/*********************************************************************************************************
* FunctionName   : TaskRemarks()
* Description    : 任务标志处理，由中1ms断函数调用
* EntryParameter : None
* ReturnValue    : None
*********************************************************************************************************/
void TaskRemarks(void)
{
    unsigned char i;

    for (i=0; i<TASKS_MAX; i++)
    {
        if (TaskComps[i].Timer)
        {
            TaskComps[i].Timer--;
            if (TaskComps[i].Timer == 0)
            {
                TaskComps[i].Timer = TaskComps[i].ItvTime;
                TaskComps[i].Run = 1;
            }
        }
    }
}

/*************  COPYRIGHT (C) 2023 ******************* END OF FILE **************************************/
