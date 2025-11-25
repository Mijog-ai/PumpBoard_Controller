#ifndef __GPIO_H
#define __GPIO_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"

/*CAN通信引脚 */
/*CANL--J1-7  CANH--J1-8*/
#define  CAN1_RX_PORT           GPIOA
#define  CAN1_TX_PORT           GPIOA

#define  CAN1_RX_PIN            GPIO_Pin_11
#define  CAN1_TX_PIN            GPIO_Pin_12

/*ADC DAC片选引脚 */
//AD7689 16bit
#define  ADC7689_NSS_GPIO       GPIOE
#define  ADC7689_NSS_PIN        GPIO_Pin_1

/*模拟量输入*/
//传感器5V供电控制
#define SENSOR_POWER_5V_GPIO  GPIOA
#define SENSOR_POWER_5V_PIN   GPIO_Pin_10

//传感器电压电流模式切换
//角度_01 IA1
#define ANGLE_01_SENSOR_MODE_GPIO  GPIOC
#define ANGLE_01_SENSOR_MODE_PIN   GPIO_Pin_14
//角度_02 IA2
#define ANGLE_02_SENSOR_MODE_GPIO  GPIOE
#define ANGLE_02_SENSOR_MODE_PIN   GPIO_Pin_6

//压力_01 IA3
#define PT_01_SENSOR_MODE_GPIO  GPIOE
#define PT_01_SENSOR_MODE_PIN   GPIO_Pin_4
//压力_02 IA4
#define PT_02_SENSOR_MODE_GPIO  GPIOE
#define PT_02_SENSOR_MODE_PIN   GPIO_Pin_2

//传感器增益控制
//角度_01 IA1
#define ANGLE_01_SENSOR_GAIN_GPIO  GPIOC
#define ANGLE_01_SENSOR_GAIN_PIN   GPIO_Pin_15
//角度_02 IA2
#define ANGLE_02_SENSOR_GAIN_GPIO  GPIOE
#define ANGLE_02_SENSOR_GAIN_PIN   GPIO_Pin_5

//压力_01 IA3
#define PT_01_SENSOR_GAIN_GPIO  GPIOE
#define PT_01_SENSOR_GAIN_PIN   GPIO_Pin_3
//压力_02 IA4
#define PT_02_SENSOR_GAIN_GPIO  GPIOB
#define PT_02_SENSOR_GAIN_PIN   GPIO_Pin_8

/*模拟量输出*/
//模拟量输出(电压或者电流模式)--高电平时是电流模式
#define ANALOG_OUTPUT_1_MODE_GPIO GPIOC
#define ANALOG_OUTPUT_1_MODE_PIN  GPIO_Pin_12
#define ANALOG_OUTPUT_2_MODE_GPIO GPIOC
#define ANALOG_OUTPUT_2_MODE_PIN  GPIO_Pin_10


//电磁阀增益控制
#define CUR_A_GAIN_GPIO        GPIOD
#define CUR_A_GAIN_PIN         GPIO_Pin_2

#define CUR_B_GAIN_GPIO        GPIOD
#define CUR_B_GAIN_PIN         GPIO_Pin_1


//5V对外供电控制
#define SENSOR_5V_CONTROL_GPIO  GPIOA
#define SENSOR_5V_CONTROL_PIN   GPIO_Pin_10

//强制关机控制
#define POWER_OFF_CONTROL_GPIO  GPIOA
#define POWER_OFF_CONTROL_PIN   GPIO_Pin_9

//蓝牙模块供电控制
#define BLE_POWER_CONTROL_GPIO  GPIOA
#define BLE_POWER_CONTROL_PIN   GPIO_Pin_8

//蓝牙模块使能
#define BLE_EN_CONTROL_GPIO  GPIOC
#define BLE_EN_CONTROL_PIN   GPIO_Pin_9


/*EEPROM片选引脚*/
//SPI1_NSS
#define  EEPROM_NSS_GPIO       GPIOA
#define  EEPROM_NSS_PIN        GPIO_Pin_4


///*传感器过流反馈引脚 高低电平*/
//#define SENSOR_OVER_CURRENT_GPIO  GPIOC
//#define SENSOR_OVER_CURRENT_PIN   GPIO_Pin_8

///*模拟量输出反馈引脚 高低电平*/
//#define ANALOG_OUTPUT_1_FDB_GPIO  GPIOD
//#define ANALOG_OUTPUT_1_FDB_PIN   GPIO_Pin_0

//#define ANALOG_OUTPUT_2_FDB_GPIO  GPIOC
//#define ANALOG_OUTPUT_2_FDB_PIN   GPIO_Pin_10

///*功率输出反馈引脚 高低电平*/
//#define CUR_A_FDB_GPIO  GPIOE
//#define CUR_A_FDB_PIN   GPIO_Pin_14

//#define CUR_B_FDB_GPIO  GPIOE
//#define CUR_B_FDB_PIN   GPIO_Pin_15

void GPIO_Init_Func(void); 
void TIM2_init(u32, u32);

#endif

