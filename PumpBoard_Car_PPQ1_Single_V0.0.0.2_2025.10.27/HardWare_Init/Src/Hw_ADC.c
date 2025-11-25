
#include "gpio.h"
#include "Hw_ADC.h"
/*******************************************************************************
* Function Name : Adc1_Analog_Input_Init()
* Description   : ADC initialization
                407 has 2 DMA controllers, each DMA has 8 streams, each stream has multiple channels that can manage 8 channels, and there is almost no conflict between each channel
* Input         :
* Output        :
* Return        :
*******************************************************************************/

void Adc1_Analog_Input_Init(void)
{
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    DMA_InitTypeDef       DMA_InitStructure;
    GPIO_InitTypeDef      GPIO_InitStructure;

    /* Enable ADCx, DMA and GPIO clocks ****************************************/ 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOG, ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);


    /* DMA2 Stream0 channel2 configuration **************************************/
    DMA_InitStructure.DMA_Channel = ADC_DMA_CH;  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC1->DR));
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&g_ADC_Original_Value;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = MCU_ADC_CHANNEL_NUM * MCU_ADC_AVG_NUM;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(ADC_DMA, &DMA_InitStructure);
    DMA_Cmd(ADC_DMA, ENABLE);

    //2. Initialize gpio
    GPIO_InitStructure.GPIO_Pin = ANALOG_02_OUTPUT_CHECK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;                                                        //Analog input pin 2
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;                                                   //No pull-up/pull-down
    GPIO_Init(GPIOA, &GPIO_InitStructure);                                                              //Initialize

	GPIO_InitStructure.GPIO_Pin = ANALOG_01_OUTPUT_CHECK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;                                                        //Analog input pin 1
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;                                                   //No pull-up/pull-down
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SENSOR_OVER_CURRENT_PIN | CURRENT_A_CHECK_PIN | CURRENT_B_CHECK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;                                                        //
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;                                                   //No pull-up/pull-down
    GPIO_Init(GPIOE, &GPIO_InitStructure);                                                              //Initialize  

    /* ADC Common Init **********************************************************/
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1; 
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    /* ADC1 Init ****************************************************************/
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = MCU_ADC_CHANNEL_NUM;
    ADC_Init(ADC1, &ADC_InitStructure);
  
    /* ADC1 regular channel4 5 configuration **************************************/
    ADC_RegularChannelConfig(ADC1,SENSOR_OVER_CURRENT_CHANNEL,	1,ADC_SampleTime_28Cycles);//ADC_SampleTime_112Cycles);                    
    ADC_RegularChannelConfig(ADC1,ANALOG_01_OUTPUT_CHECK_CHANNEL,2,ADC_SampleTime_28Cycles);
    ADC_RegularChannelConfig(ADC1,ANALOG_02_OUTPUT_CHECK_CHANNEL,3,ADC_SampleTime_28Cycles);
    ADC_RegularChannelConfig(ADC1,CURRENT_A_CHECK_CHANNEL,	 	4,ADC_SampleTime_28Cycles);
    ADC_RegularChannelConfig(ADC1,CURRENT_B_CHECK_CHANNEL,		5,ADC_SampleTime_28Cycles);


    //  ADC_RegularChannelConfig(ADC1,ADC_Channel_18,13,ADC_SampleTime_480Cycles);//V_bat
    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    ADC_SoftwareStartConv(ADC1);//Direct start conversion
}
