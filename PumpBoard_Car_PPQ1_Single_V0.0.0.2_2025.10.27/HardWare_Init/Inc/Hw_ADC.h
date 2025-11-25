#ifndef HW_ADC_H
#define HW_ADC_H

#include "stm32f4xx.h"

//#define ADC_CALIB_OK			    0

//#define VOLT_FACTOR               9120

//Sampling time = Hold time 13.5+12.5 cycles conversion time=26 cycles, ADC clock 10.66M, single channel time 2.44us
//Total 5 channels, sampling one cycle is 12.2us, 200Hz (5ms) frequency can do 400 times
//ADC averaging configuration
//ADC_DMA
#define ADC_DMA					    DMA2_Stream0
#define ADC_DMA_CH	 			    DMA_Channel_0

#define MCU_ADC_CHANNEL_NUM         (5)   									//ADC channel count
#define MCU_ADC_AVG_NUM             (32)		                            //ADC averaging total count

//Sensor overcurrent monitoring
#define SENSOR_OVER_CURRENT_GPIO  	 	GPIOC
#define SENSOR_OVER_CURRENT_PIN   	 	GPIO_Pin_8
#define SENSOR_OVER_CURRENT_CHANNEL  	ADC_Channel_0

//Analog output 1 fault monitoring
#define ANALOG_01_OUTPUT_CHECK_GPIO  	GPIOD
#define ANALOG_01_OUTPUT_CHECK_PIN   	GPIO_Pin_10
#define ANALOG_01_OUTPUT_CHECK_CHANNEL  ADC_Channel_1

//Analog output 2 fault detection
#define ANALOG_02_OUTPUT_CHECK_GPIO  	GPIOC
#define ANALOG_02_OUTPUT_CHECK_PIN   	GPIO_Pin_10
#define ANALOG_02_OUTPUT_CHECK_CHANNEL  ADC_Channel_2

//Solenoid current A detection
#define CURRENT_A_CHECK_GPIO  			GPIOE
#define CURRENT_A_CHECK_PIN   			GPIO_Pin_14
#define CURRENT_A_CHECK_CHANNEL  	    ADC_Channel_3

//Solenoid current B detection
#define CURRENT_B_CHECK_GPIO  			GPIOE
#define CURRENT_B_CHECK_PIN   			GPIO_Pin_15
#define CURRENT_B_CHECK_CHANNEL   		ADC_Channel_4


//Corresponding analog values position saved in array g_ADC_Original_Value[]
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

//Hardware initialization
void Adc1_Analog_Input_Init(void);




#endif
