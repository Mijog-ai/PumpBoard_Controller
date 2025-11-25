/*	
* 实现底层GPIO驱动的编写
* 
*/
#include "gpio.h"	

void GPIO_Init_Func(void)
{
	GPIO_InitTypeDef          GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);       //使能PORTA时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);       //使能PORTB时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);       //使能PORTC时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);       //使能PORTD时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);       //使能PORTE时钟	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);       //使能PORTF时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);       //使能PORTF时钟
	
	
	/*GPIO输出*/
//    //传感器5V供电控制	
//	GPIO_InitStructure.GPIO_Pin = SENSOR_POWER_5V_PIN;          
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
//	GPIO_Init(SENSOR_POWER_5V_GPIO,&GPIO_InitStructure);       //初始化	
//	SENSOR_POWER_5V_GPIO->BSRRL |= SENSOR_POWER_5V_PIN;			//拉低

    //角度传感器1电压电流模式选择
    GPIO_InitStructure.GPIO_Pin = ANGLE_01_SENSOR_MODE_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANGLE_01_SENSOR_MODE_GPIO,&GPIO_InitStructure);       //初始化	
	//角度传感器2电压电流模式选择
	GPIO_InitStructure.GPIO_Pin = ANGLE_02_SENSOR_MODE_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANGLE_02_SENSOR_MODE_GPIO,&GPIO_InitStructure);       //初始化	
    //压力传感器1电压电流模式选择
    GPIO_InitStructure.GPIO_Pin = PT_01_SENSOR_MODE_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(PT_01_SENSOR_MODE_GPIO,&GPIO_InitStructure);       //初始化	
    //压力传感器2电压电流模式选择
    GPIO_InitStructure.GPIO_Pin = PT_02_SENSOR_MODE_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(PT_02_SENSOR_MODE_GPIO,&GPIO_InitStructure);       //初始化
	
	//角度传感器1增益控制
    GPIO_InitStructure.GPIO_Pin = ANGLE_01_SENSOR_GAIN_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANGLE_01_SENSOR_GAIN_GPIO,&GPIO_InitStructure);       //初始化
	//角度传感器2增益控制
    GPIO_InitStructure.GPIO_Pin = ANGLE_02_SENSOR_GAIN_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANGLE_02_SENSOR_GAIN_GPIO,&GPIO_InitStructure);       //初始化
	//压力传感器11增益控制
    GPIO_InitStructure.GPIO_Pin = PT_01_SENSOR_GAIN_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(PT_01_SENSOR_GAIN_GPIO,&GPIO_InitStructure);       //初始化	
    //压力传感器21增益控制
    GPIO_InitStructure.GPIO_Pin = PT_02_SENSOR_MODE_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(PT_02_SENSOR_GAIN_GPIO,&GPIO_InitStructure);       //初始化
	
	//模拟输出2
    GPIO_InitStructure.GPIO_Pin = ANALOG_OUTPUT_2_MODE_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANALOG_OUTPUT_2_MODE_GPIO,&GPIO_InitStructure);       //初始化	
    //模拟输出1
    GPIO_InitStructure.GPIO_Pin = ANALOG_OUTPUT_1_MODE_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(ANALOG_OUTPUT_1_MODE_GPIO,&GPIO_InitStructure);       //初始化	
    
	//电磁阀A输出控制
	GPIO_InitStructure.GPIO_Pin = CUR_A_GAIN_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(CUR_A_GAIN_GPIO,&GPIO_InitStructure);       //初始化	
	
	//电磁阀B输出控制
	GPIO_InitStructure.GPIO_Pin = CUR_B_GAIN_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(CUR_B_GAIN_GPIO,&GPIO_InitStructure);       //初始化	
	
	//蓝牙模块供电控制
    GPIO_InitStructure.GPIO_Pin = BLE_POWER_CONTROL_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(BLE_POWER_CONTROL_GPIO,&GPIO_InitStructure);       //初始化
	BLE_POWER_CONTROL_GPIO->BSRRH |= BLE_POWER_CONTROL_PIN;  	 //拉低
	
	//蓝牙模块使能
    GPIO_InitStructure.GPIO_Pin = BLE_EN_CONTROL_PIN;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;                //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         //速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;               //
	GPIO_Init(BLE_EN_CONTROL_GPIO,&GPIO_InitStructure);       	 //初始化
	BLE_EN_CONTROL_GPIO->BSRRL |= BLE_EN_CONTROL_PIN;  			 //拉高


	GPIO_InitStructure.GPIO_Pin = SENSOR_POWER_5V_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;      // 输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 输入模式速度可省略但保留无错
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;  // 浮空输入（不使能内部电阻）
	GPIO_Init(SENSOR_POWER_5V_GPIO, &GPIO_InitStructure);
	
	
}
