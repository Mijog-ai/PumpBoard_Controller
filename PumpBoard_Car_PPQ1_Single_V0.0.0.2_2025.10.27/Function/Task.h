/********************************************************************************************************
*	@file 	    Task.h
*	@brief
*	@author     
*	@version    
*	@date       
*	@warning
*********************************************************************************************************/

/****************************** Define to prevent recursive inclusion **********************************/
#ifndef _TASK_H
#define _TASK_H
#include <stdint.h>

/******************************************* variable define ********************************************/
#define	DEBUG_MODE		1

#define FL_BOOT_MODE    ((volatile uint32 *)0x2000F000)

extern uint8_t SignalDelayFlag_500ms;
extern uint8_t Delay_1000ms;
extern uint8_t Delay_2000ms;
extern uint8_t Delay_500ms;


 typedef struct _TASK_COMPONENTS
{
    unsigned char  Run;
    unsigned short Timer;
    unsigned short ItvTime;
    void (*TaskHook)(void);
}TASK_COMPONENTS;


typedef enum _TASK_LIST
{
    TASK_1MS,
    TASK_5MS,
    TASK_10MS,
    TASK_100MS,
    TASKS_MAX
} TASK_LIST;


/******************************************* function define ********************************************/
extern void TaskParmInit(void);
extern void TaskProcess(void);
extern void TaskRemarks(void);


#endif/* _TASK_H */
/*************  COPYRIGHT (C) 2019 ******************* END OF FILE **************************************/
