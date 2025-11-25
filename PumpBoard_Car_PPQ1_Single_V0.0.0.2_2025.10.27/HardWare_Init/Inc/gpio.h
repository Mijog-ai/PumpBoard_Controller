#ifndef __GPIO_H
#define __GPIO_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"

/*CAN communication pins */
/*CANL--J1-7  CANH--J1-8*/
#define  CAN1_RX_PORT           GPIOA
#define  CAN1_TX_PORT           GPIOA

#define  CAN1_RX_PIN            GPIO_Pin_11
#define  CAN1_TX_PIN            GPIO_Pin_12

/*ADC DAC chip select pins */
//AD7689 16bit
#define  ADC7689_NSS_GPIO       GPIOE
#define  ADC7689_NSS_PIN        GPIO_Pin_1

/*Analog sensor inputs*/
//Sensor 5V power control
#define SENSOR_POWER_5V_GPIO  GPIOA
#define SENSOR_POWER_5V_PIN   GPIO_Pin_10

//Sensor voltage/current mode switching
//Angle_01 IA1
#define ANGLE_01_SENSOR_MODE_GPIO  GPIOC
#define ANGLE_01_SENSOR_MODE_PIN   GPIO_Pin_14
//Angle_02 IA2
#define ANGLE_02_SENSOR_MODE_GPIO  GPIOE
#define ANGLE_02_SENSOR_MODE_PIN   GPIO_Pin_6

//Pressure_01 IA3
#define PT_01_SENSOR_MODE_GPIO  GPIOE
#define PT_01_SENSOR_MODE_PIN   GPIO_Pin_4
//Pressure_02 IA4
#define PT_02_SENSOR_MODE_GPIO  GPIOE
#define PT_02_SENSOR_MODE_PIN   GPIO_Pin_2

//Sensor gain control
//Angle_01 IA1
#define ANGLE_01_SENSOR_GAIN_GPIO  GPIOC
#define ANGLE_01_SENSOR_GAIN_PIN   GPIO_Pin_15
//Angle_02 IA2
#define ANGLE_02_SENSOR_GAIN_GPIO  GPIOE
#define ANGLE_02_SENSOR_GAIN_PIN   GPIO_Pin_5

//Pressure_01 IA3
#define PT_01_SENSOR_GAIN_GPIO  GPIOE
#define PT_01_SENSOR_GAIN_PIN   GPIO_Pin_3
//Pressure_02 IA4
#define PT_02_SENSOR_GAIN_GPIO  GPIOB
#define PT_02_SENSOR_GAIN_PIN   GPIO_Pin_8

/*Analog output*/
//Analog output (voltage or current mode)--High level is current mode
#define ANALOG_OUTPUT_1_MODE_GPIO GPIOC
#define ANALOG_OUTPUT_1_MODE_PIN  GPIO_Pin_12
#define ANALOG_OUTPUT_2_MODE_GPIO GPIOC
#define ANALOG_OUTPUT_2_MODE_PIN  GPIO_Pin_10


//Solenoid gain control
#define CUR_A_GAIN_GPIO        GPIOD
#define CUR_A_GAIN_PIN         GPIO_Pin_2

#define CUR_B_GAIN_GPIO        GPIOD
#define CUR_B_GAIN_PIN         GPIO_Pin_1


//5V sensor power control
#define SENSOR_5V_CONTROL_GPIO  GPIOA
#define SENSOR_5V_CONTROL_PIN   GPIO_Pin_10

//Force shutdown control
#define POWER_OFF_CONTROL_GPIO  GPIOA
#define POWER_OFF_CONTROL_PIN   GPIO_Pin_9

//Bluetooth module power control
#define BLE_POWER_CONTROL_GPIO  GPIOA
#define BLE_POWER_CONTROL_PIN   GPIO_Pin_8

//Bluetooth module enable
#define BLE_EN_CONTROL_GPIO  GPIOC
#define BLE_EN_CONTROL_PIN   GPIO_Pin_9


/*EEPROM chip select pins*/
//SPI1_NSS
#define  EEPROM_NSS_GPIO       GPIOA
#define  EEPROM_NSS_PIN        GPIO_Pin_4


///*Sensor overcurrent protection high/low level*/
//#define SENSOR_OVER_CURRENT_GPIO  GPIOC
//#define SENSOR_OVER_CURRENT_PIN   GPIO_Pin_8

///*Analog output fault detection high/low level*/
//#define ANALOG_OUTPUT_1_FDB_GPIO  GPIOD
//#define ANALOG_OUTPUT_1_FDB_PIN   GPIO_Pin_0

//#define ANALOG_OUTPUT_2_FDB_GPIO  GPIOC
//#define ANALOG_OUTPUT_2_FDB_PIN   GPIO_Pin_10

///*Solenoid fault detection high/low level*/
//#define CUR_A_FDB_GPIO  GPIOE
//#define CUR_A_FDB_PIN   GPIO_Pin_14

//#define CUR_B_FDB_GPIO  GPIOE
//#define CUR_B_FDB_PIN   GPIO_Pin_15

void GPIO_Init_Func(void); 
void TIM2_init(u32, u32);

#endif

