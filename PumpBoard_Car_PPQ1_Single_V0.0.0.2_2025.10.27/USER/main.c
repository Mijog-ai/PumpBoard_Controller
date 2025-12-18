#include "application.h"
#include "Task.h"
#include "AD7689.h"
#include "Hw_spi.h"
#include "Hw_ADC.h"
#include "gpio.h"



#define NUMBER_TASKTIMERS	(8u)
#define TASKTIMER_IDLE  	(0u)
#define TASKTIMER_RUNNING   (1u)
#define TASKTIMER_EXPIRED   (2u)
#define TASKTIMER_FAULT     (0xF0)

#define TASK_IDLE_MINIMUM	(0u)
#define TASK_IDLE_LAST		(1u)
#define TASK_IDLE_MAXIMUM	(2u)

s32                     g_HLCmdMap[HL_CMDMAP_LEN] = {0};                            //???????s32???????????????float???l???
u16                     g_PCSetCmd[HL_WRCMD_LEN] = {0};
u32                     g_PCRdCmd[HL_RDCMD_LEN] = {0};
_TIM_PARA               TimParametr = _TIM_PARA_DEFAULT;
_LOW_PASS_FILTER_PARA   FilterTimConstant = _LOW_PASS_FILTER_PARA_DEFAULT;
//u8 Escape_flag=0,Four_Driver_Flag=0,EconomicMode_flag=0,EconomicMode=0;

/**
  * @brief  Configures the SysTick.
  * @param  None
  * @retval None
  */
static void SysTickConfig(void)
{
  /* Setup SysTick Timer for 100 msec interrupts  */
  if (SysTick_Config((SystemCoreClock) / 10000))
  { 
    /* Capture error */ 
    while (1);
  }

  NVIC_SetPriority(SysTick_IRQn, 0);
}


//??'?????????Z?
//prer:?????:0~7(????3????!)
//rlr:?????????,0~0XFFF.
//???????=4*2^prer.???????????256!
//rlr:?????J????:??11????.
//??????(???):Tout=((4*2^prer)*rlr)/32 (ms).
void IWDG_Init(u8 prer,u16 rlr)
{
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); //'???IWDG->PR IWDG->RLR???
	
	IWDG_SetPrescaler(prer); //????IWDG??????

	IWDG_SetReload(rlr);   //????IWDG????

	IWDG_ReloadCounter(); //reload
	
	IWDG_Enable();       //'????Z?
}

//????????Z?
void IWDG_Feed(void)
{
	IWDG_ReloadCounter();//reload
}

u16 g_u16_TestTim2_CCR2 = 1000, g_u16_TestTim2_CCR4 = 10000;

// Simulation mode flag - set to 1 to prevent ADC from overwriting test values
u8 g_u8_SimulationMode = 0;

int main(void)
{
	/*--------------------------------------------------------------*/
	/*2023 06 05 suqf	
	????????APP?????????t??????????????????????????-Target??
	??IROM1??start???????APP???????'???*/
//	NVIC_SetVectorTable(NVIC_VectTab_FLASH,APP_START_OFFSET_ADDR);
	/*--------------------------------------------------------------*/

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//?????????????????
		
	GPIO_Init_Func();               //?????GPIO??'??	
	
	__disable_irq();	            //??????????
	
  InitUserParaProcess();          //????????

	SysTickConfig();                //???d??????'??   ???? 100us 

  SPI3_Int();						//SPI3??'??

	TIM2_init(PWM_ARR,TIM2_PSC);    //TIM2??'??  ??y????????

	Adc1_Analog_Input_Init();       //AD??'??
	   
  AD7689InitFunc();               //AD7689??'??
    
  SensorModeChangeFunc();         //??????g????    DAC??????OR????

	CurDetectionRangeChose();       //??y???????????????
	
	__enable_irq();//???????
	
	TaskParmInit();

	//==========================================================
	// SIMULATION TEST CODE - Set to 0 for real hardware!
	//==========================================================
	#if 1  // Set to 0 for real hardware
	{
		// Enable simulation mode (prevents ADC from overwriting values)
		g_u8_SimulationMode = 1;

		// Enable all control loops
		User_Parameter.PpqEna->PumpStrt_Ena = 1;
		User_Parameter.PpqEna->PrsLoop_Ena = 1;
		User_Parameter.PpqEna->AngLeak_Ena = 1;
		User_Parameter.PpqEna->PwrLoop_Ena = 1;

		// Set target references
		InstructionSet.TiltAngRef = 5000;        // 50% angle (0-10000)
		InstructionSet.g_u16_PerUnitPrsVal = 3000;
		InstructionSet.Cur_A_Ref = 1500;         // 1500mA current A
		InstructionSet.Cur_B_Ref = 1500;         // 1500mA current B

		// Simulate sensor feedback (creates error for PID)
		DetectorFdbVal.g_u32_AngFdb = 2000;      // 20% angle feedback
		DetectorFdbVal.g_s32_CurFdb_A = 800;     // 800mA current A
		DetectorFdbVal.g_s32_CurFdb_B = 800;     // 800mA current B
	}
	#endif
	//==========================================================

    while(1)
	{
        TaskProcess();
	}	
}