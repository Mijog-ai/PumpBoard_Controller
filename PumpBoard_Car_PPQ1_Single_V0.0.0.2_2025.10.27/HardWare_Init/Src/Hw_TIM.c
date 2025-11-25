
#include "stm32f4xx.h"
#include "gpio.h"	

/*******************************************************************************
* Function Name : TIM2_init()
* Description   :  
* Input         : 
* Output        : 
* Return        : 
*******************************************************************************/
void TIM2_init(u32 arr,u32 psc)
{
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
	GPIO_InitTypeDef          GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2 clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //Enable PORTA clock

	//2. Set alternate function    Note: STM32F103 has default alternate functions, but 407 must explicitly configure alternate functions. To use a peripheral, must set the corresponding alternate function using GPIO_PinAFConfig() function
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_TIM2);   //PB10<-->TIM2-CH3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_TIM2);  //PAB11<-->TIM2-CH4

	//3. Configure GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //Set alternate function
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //Speed 100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //Push-pull alternate output
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //Pull-up
	GPIO_Init(GPIOB,&GPIO_InitStructure);                  //Initialize
	

	TIM_TimeBaseStructure.TIM_Prescaler = psc;             //Timer prescaler
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;//TIM_CounterMode_CenterAligned1; //Center-aligned mode
	TIM_TimeBaseStructure.TIM_Period = arr;   //Auto-reload value
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//Initialize timer 2

	//Initialize TIM2 PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_OCMode_PWM2;              //Select timer mode: TIM pulse width modulation mode 2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //Compare output enable, output is
//    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
//    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
//    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_SetCompare3(TIM2, 0);
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable); // Enable TIM2 preload register on CCR3
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);

    TIM_SetCompare4(TIM2, 0);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); // Enable TIM2 preload register on CCR4
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

	TIM_ARRPreloadConfig(TIM2,ENABLE);                 //ARPE enable

	TIM_Cmd(TIM2, ENABLE);            //Enable TIM2
	
}
