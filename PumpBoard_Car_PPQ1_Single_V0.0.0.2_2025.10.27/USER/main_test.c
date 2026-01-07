/**
 * @file main_test.c
 * @brief Hardware test version of main.c for STM32F407ZE PumpCtrl
 * 
 * This file runs hardware tests instead of the normal application.
 * Use this to verify hardware components before running full application.
 */

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_can.h"

// Dummy variables to satisfy linker (not used in tests)
s32 g_HLCmdMap[10] = {0};
u16 g_PCSetCmd[10] = {0};
u32 g_PCRdCmd[10] = {0};
typedef struct { int dummy; } _TIM_PARA;
typedef struct { int dummy; } _LOW_PASS_FILTER_PARA;
_TIM_PARA TimParametr = {0};
_LOW_PASS_FILTER_PARA FilterTimConstant = {0};

// Test mode selection - uncomment ONE test at a time
#define TEST_MODE_GPIO      1
// #define TEST_MODE_ADC       1
// #define TEST_MODE_PWM       1
// #define TEST_MODE_SPI       1
// #define TEST_MODE_CAN       1
// #define TEST_MODE_ALL       1

// Global test results
volatile uint16_t g_test_adc_value = 0;
volatile uint8_t g_test_spi_result = 0;
volatile uint32_t g_test_counter = 0;

/**
 * @brief Simple delay function
 */
void Delay_ms(uint32_t ms)
{
    for(volatile uint32_t i = 0; i < ms * 8000; i++);
}

/**
 * @brief Test GPIO by toggling LED
 */
void Test_GPIO_LED(void)
{
    // Enable GPIOF clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    
    // Configure PF9 as output (adjust pin based on your board)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    // Blink LED pattern: Fast = Test Running
    while(1)
    {
        GPIO_SetBits(GPIOF, GPIO_Pin_9);
        Delay_ms(100);
        GPIO_ResetBits(GPIOF, GPIO_Pin_9);
        Delay_ms(100);
        
        GPIO_SetBits(GPIOF, GPIO_Pin_10);
        Delay_ms(100);
        GPIO_ResetBits(GPIOF, GPIO_Pin_10);
        Delay_ms(100);
        
        g_test_counter++;
    }
}

/**
 * @brief Test ADC by reading a channel continuously
 */
void Test_ADC_Read(void)
{
    // Enable clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    
    // Configure PA0 as analog input
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Configure LED for status
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    // Configure ADC
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
    
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // Configure channel
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_84Cycles);
    
    // Enable ADC
    ADC_Cmd(ADC1, ENABLE);
    
    // Continuous reading
    while(1)
    {
        // Start conversion
        ADC_SoftwareStartConv(ADC1);
        
        // Wait for conversion
        while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
        
        // Read value
        g_test_adc_value = ADC_GetConversionValue(ADC1);
        
        // Blink LED based on ADC value
        if(g_test_adc_value > 2048)
        {
            GPIO_SetBits(GPIOF, GPIO_Pin_9);
        }
        else
        {
            GPIO_ResetBits(GPIOF, GPIO_Pin_9);
        }
        
        Delay_ms(100);
        g_test_counter++;
    }
}

/**
 * @brief Test PWM output on TIM2
 */
void Test_PWM_Output(void)
{
    // Enable clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    
    // Configure PA1 as TIM2_CH2 (alternate function)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
    
    // Configure status LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    // Configure TIM2
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // Configure PWM mode
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 5000;  // 25% duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    
    // Enable timer
    TIM_Cmd(TIM2, ENABLE);
    
    // Sweep duty cycle
    uint16_t duty = 0;
    uint8_t direction = 1;
    
    while(1)
    {
        TIM2->CCR2 = duty;
        
        if(direction)
        {
            duty += 100;
            if(duty >= 20000) direction = 0;
        }
        else
        {
            duty -= 100;
            if(duty == 0) direction = 1;
        }
        
        // Toggle LED to show test is running
        GPIO_ToggleBits(GPIOF, GPIO_Pin_9);
        
        Delay_ms(10);
        g_test_counter++;
    }
}

/**
 * @brief Test SPI communication
 */
void Test_SPI_Loopback(void)
{
    // Enable clocks
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    
    // Configure SPI pins (PC10=SCK, PC11=MISO, PC12=MOSI)
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SPI3);
    
    // Configure status LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    // Configure SPI
    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI3, &SPI_InitStructure);
    
    // Enable SPI
    SPI_Cmd(SPI3, ENABLE);
    
    // Test pattern
    uint8_t test_data = 0xA5;
    
    while(1)
    {
        // Send test byte
        while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);
        SPI_I2S_SendData(SPI3, test_data);
        
        // Receive byte
        while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);
        g_test_spi_result = SPI_I2S_ReceiveData(SPI3);
        
        // Check result (will only match if MOSI connected to MISO)
        if(g_test_spi_result == test_data)
        {
            GPIO_SetBits(GPIOF, GPIO_Pin_9);    // Success - Green LED
            GPIO_ResetBits(GPIOF, GPIO_Pin_10);
        }
        else
        {
            GPIO_ResetBits(GPIOF, GPIO_Pin_9);
            GPIO_SetBits(GPIOF, GPIO_Pin_10);   // Fail - Red LED
        }
        
        test_data++;
        Delay_ms(100);
        g_test_counter++;
    }
}

/**
 * @brief Test all hardware sequentially
 */
void Test_All_Hardware(void)
{
    // Enable LED GPIO
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    
    while(1)
    {
        // Test 1: GPIO - Blink pattern
        for(int i = 0; i < 10; i++)
        {
            GPIO_ToggleBits(GPIOF, GPIO_Pin_9);
            Delay_ms(100);
        }
        
        // Test 2: ADC - Quick read
        // (Add ADC test code here if needed)
        
        // Test 3: PWM - Brief output
        // (Add PWM test code here if needed)
        
        // Indicate test cycle complete
        GPIO_SetBits(GPIOF, GPIO_Pin_9 | GPIO_Pin_10);
        Delay_ms(500);
        GPIO_ResetBits(GPIOF, GPIO_Pin_9 | GPIO_Pin_10);
        Delay_ms(500);
        
        g_test_counter++;
    }
}

/**
 * @brief Main function for hardware testing
 */
int main(void)
{
    // Configure system
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    
    // Run selected test
    #ifdef TEST_MODE_GPIO
        Test_GPIO_LED();
    #elif defined(TEST_MODE_ADC)
        Test_ADC_Read();
    #elif defined(TEST_MODE_PWM)
        Test_PWM_Output();
    #elif defined(TEST_MODE_SPI)
        Test_SPI_Loopback();
    #elif defined(TEST_MODE_ALL)
        Test_All_Hardware();
    #else
        // Default: GPIO test
        Test_GPIO_LED();
    #endif
    
    // Should never reach here
    while(1);
}
