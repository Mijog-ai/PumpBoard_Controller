#ifndef HW_ADC_H
#define HW_ADC_H

#include "stm32f4xx.h"

//#define ADC_CALIB_OK			    0

//#define VOLT_FACTOR               9120

//采样时间 = 保持时间13.5+12.5个周期的转换时间=26周期，ADC时钟10.66M，单个通道时间2.44us
//一共有5个通道，采样一个周期是12.2us，200Hz（5ms）的频率可以做400次
//ADC求平均配置
//ADC_DMA
#define ADC_DMA					    DMA2_Stream0
#define ADC_DMA_CH	 			    DMA_Channel_0

#define MCU_ADC_CHANNEL_NUM         (5)   									//ADC通道数
#define MCU_ADC_AVG_NUM             (32)		                            //ADC取平均的总数

//传感器过流监测
#define SENSOR_OVER_CURRENT_GPIO  	 	GPIOC
#define SENSOR_OVER_CURRENT_PIN   	 	GPIO_Pin_8
#define SENSOR_OVER_CURRENT_CHANNEL  	ADC_Channel_0

//模拟输出1故障监测
#define ANALOG_01_OUTPUT_CHECK_GPIO  	GPIOD
#define ANALOG_01_OUTPUT_CHECK_PIN   	GPIO_Pin_10
#define ANALOG_01_OUTPUT_CHECK_CHANNEL  ADC_Channel_1

//模拟输出2故障检测
#define ANALOG_02_OUTPUT_CHECK_GPIO  	GPIOC
#define ANALOG_02_OUTPUT_CHECK_PIN   	GPIO_Pin_10
#define ANALOG_02_OUTPUT_CHECK_CHANNEL  ADC_Channel_2

//电磁阀电流A检测
#define CURRENT_A_CHECK_GPIO  			GPIOE
#define CURRENT_A_CHECK_PIN   			GPIO_Pin_14
#define CURRENT_A_CHECK_CHANNEL  	    ADC_Channel_3

//电磁阀电流B检测
#define CURRENT_B_CHECK_GPIO  			GPIOE
#define CURRENT_B_CHECK_PIN   			GPIO_Pin_15
#define CURRENT_B_CHECK_CHANNEL   		ADC_Channel_4


//对应的模拟量在数组g_ADC_Original_Value[]中保存的位置
#define SENSOR_OVER_CURRENT         0
#define ANLOG_01_OUTPUT_CHECK       1
#define ANLOG_02_OUTPUT_CHECK       2
#define CURRENT_A_CHECK             3
#define CURRENT_B_CHECK             4


typedef struct
{
	u32 g_u32_Vbus;
	u32 g_u32_SwivelAngle_01;
	u32 g_u32_SwivelAngle_02;
    u32 g_u32_Pressure_01;
    u32 g_u32_Pressure_02;
	u32 g_u32_CURRENT_A;
    u32 g_u32_CURRENT_B;
	u32 g_u32_NONEED;

    u16 g_u16_SENSOR_OVER_CURRENT;
    u16 g_u16_ANALOG_01_OUTPUT_CHECK;
    u16 g_u16_ANALOG_02_OUTPUT_CHECK;
    u16 g_u16_CURRENT_A_CHECK;
    u16 g_u16_CURRENT_B_CHECK;
    
    u16 g_u16_TorqueLmt;
}_ANALOG_INPUT_VALUE;

typedef struct
{
    u16 g_u16_ValveCtrlCur_4mA_Among_20mA;
    s32 g_s32_ValveCtrlVolt_Neg10V_Among_Pos10V;
}_ANALOG_OUTPUT_VALUE;

#define _ANALOG_INPUT_VALUE_DEFAULT     {0,0,0,0,0,0,0,0,\
                                         0,0,0,0,0,\
                                         0}

#define _ANALOG_OUTPUT_VALUE_DEFAULT    {0,0}

extern _ANALOG_INPUT_VALUE     AnalogInput;
extern volatile u16 g_ADC_Original_Value[MCU_ADC_CHANNEL_NUM * MCU_ADC_AVG_NUM];


u16 GetOriginalADCVal(u16);
void Cal_AngFbkDiff_Func(void);
void CalAngFbkDiff_InPrsLoop_Func(void);
void TransAD7689DataIntoFIFO(void);
void Cal_AD7689_AvgAdcVal_Fun(void);
void Cal_MCU_AvgAdcVal_Fun(void);
void Test_LowPassFilterFunc(void);
void LowPassFilterFunc(float *P, float, float);

//硬件初始化
void Adc1_Analog_Input_Init(void);




#endif
