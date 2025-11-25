/*
* Implement low-level GPIO register read and write
*
*/
#include "gpio.h"

void GPIO_Init_Func(void)
{
	GPIO_InitTypeDef          GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);       //Enable PORTA clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);       //Enable PORTB clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);       //Enable PORTC clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);       //Enable PORTD clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);       //Enable PORTE clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);       //Enable PORTF clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);       //Enable PORTG clock


	/*GPIO configuration*/
//    //Sensor 5V power control
//	GPIO_InitStructure.GPIO_Pin = SENSOR_POWER_5V_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
//	GPIO_Init(SENSOR_POWER_5V_GPIO,&GPIO_InitStructure);       //Initialize
//	SENSOR_POWER_5V_GPIO->BSRRL |= SENSOR_POWER_5V_PIN;			//Enable

    //Angle sensor 1 voltage/current mode selection
    GPIO_InitStructure.GPIO_Pin = ANGLE_01_SENSOR_MODE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANGLE_01_SENSOR_MODE_GPIO,&GPIO_InitStructure);       //Initialize
	//Angle sensor 2 voltage/current mode selection
	GPIO_InitStructure.GPIO_Pin = ANGLE_02_SENSOR_MODE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANGLE_02_SENSOR_MODE_GPIO,&GPIO_InitStructure);       //Initialize
    //Pressure sensor 1 voltage/current mode selection
    GPIO_InitStructure.GPIO_Pin = PT_01_SENSOR_MODE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(PT_01_SENSOR_MODE_GPIO,&GPIO_InitStructure);       //Initialize
    //Pressure sensor 2 voltage/current mode selection
    GPIO_InitStructure.GPIO_Pin = PT_02_SENSOR_MODE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(PT_02_SENSOR_MODE_GPIO,&GPIO_InitStructure);       //Initialize

	//Angle sensor 1 gain control
    GPIO_InitStructure.GPIO_Pin = ANGLE_01_SENSOR_GAIN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANGLE_01_SENSOR_GAIN_GPIO,&GPIO_InitStructure);       //Initialize
	//Angle sensor 2 gain control
    GPIO_InitStructure.GPIO_Pin = ANGLE_02_SENSOR_GAIN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANGLE_02_SENSOR_GAIN_GPIO,&GPIO_InitStructure);       //Initialize
	//Pressure sensor 1 gain control
    GPIO_InitStructure.GPIO_Pin = PT_01_SENSOR_GAIN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(PT_01_SENSOR_GAIN_GPIO,&GPIO_InitStructure);       //Initialize
    //Pressure sensor 2 gain control
    GPIO_InitStructure.GPIO_Pin = PT_02_SENSOR_MODE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(PT_02_SENSOR_GAIN_GPIO,&GPIO_InitStructure);       //Initialize

	//Analog output 2
    GPIO_InitStructure.GPIO_Pin = ANALOG_OUTPUT_2_MODE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANALOG_OUTPUT_2_MODE_GPIO,&GPIO_InitStructure);       //Initialize
    //Analog output 1
    GPIO_InitStructure.GPIO_Pin = ANALOG_OUTPUT_1_MODE_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANALOG_OUTPUT_1_MODE_GPIO,&GPIO_InitStructure);       //Initialize

	//Solenoid A gain control
	GPIO_InitStructure.GPIO_Pin = CUR_A_GAIN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(CUR_A_GAIN_GPIO,&GPIO_InitStructure);       //Initialize

	//Solenoid B gain control
	GPIO_InitStructure.GPIO_Pin = CUR_B_GAIN_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(CUR_B_GAIN_GPIO,&GPIO_InitStructure);       //Initialize

	//Bluetooth module power control
    GPIO_InitStructure.GPIO_Pin = BLE_POWER_CONTROL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(BLE_POWER_CONTROL_GPIO,&GPIO_InitStructure);       //Initialize
	BLE_POWER_CONTROL_GPIO->BSRRH |= BLE_POWER_CONTROL_PIN;  	 //Disable

	//Bluetooth module enable
    GPIO_InitStructure.GPIO_Pin = BLE_EN_CONTROL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //Speed 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(BLE_EN_CONTROL_GPIO,&GPIO_InitStructure);       	 //Initialize
	BLE_EN_CONTROL_GPIO->BSRRL |= BLE_EN_CONTROL_PIN;  			 //Enable


	GPIO_InitStructure.GPIO_Pin = SENSOR_POWER_5V_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;      // Input mode
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // Speed can be omitted in input mode, no significant impact
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  // No pull-up/pull-down (not using internal pull-up/pull-down)
	GPIO_Init(SENSOR_POWER_5V_GPIO, &GPIO_InitStructure);
	
	
}
