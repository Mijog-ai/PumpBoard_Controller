/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    04-August-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"

#include "Task.h"
#include "application.h" 

#include "AD7689.h"
#include "Hw_spi.h"
#include "Hw_Adc.h"


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */

void PendSV_Handler(void)
{
}

void TIM1_UP_TIM10_IRQHandler(void)
{

}

void ADC_IRQHandler(void)//延后4us  共执行2.16us
{
	
}
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */

extern uint8_t sincos_flag;
extern float freq;        // 50Hz
extern uint16_t center;    // 中心值1500
extern uint16_t amp;        // 振幅500
//u16 g_u16AD7949_rxBuf[ADC_CHANNEL_NUM * ADC_FILTER_FIFO] = {0};
void SysTick_Handler(void)//100us
{ 
	
			Cal_MCU_AvgAdcVal_Fun();      // 模拟输入检测
            TransAD7689DataIntoFIFO();   // AD数据搬运
			CalAD7689AvgAngPrsData();    // 滤波计算
            Cal_SolenoidCur();           // 电流计算
			Cal_PrsFbkDiff_Func();       // 压力微分
            Cal_AngFbkDiff_Func();       // 角度微分
            CalAngFbkDiff_InPrsLoop_Func();
			TiltAngleLoop_Func();        // 控制算法
			TimPeriodCnt_Func();

            #ifndef PPQ_1

			#endif

			TaskRemarks();
}


//捕获状态
void TIM3_IRQHandler(void)
{ 		
	
}
//基波控制
void TIM5_IRQHandler(void)
{	
	
}
void TIM1_BRK_TIM9_IRQHandler(void)
{
		
}
		    
void CAN1_RX0_IRQHandler(void)//18fe3082
{
//		 CAN_Receive_j1939(CAN1, 0, &canopen_msg);
}
void CAN2_RX0_IRQHandler(void)
{
//		 CAN_Receive(CAN2, 0, &can_j1939_msg);
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
