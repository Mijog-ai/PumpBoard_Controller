
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
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2时钟使能 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);  //使能PORTA时钟	
	
	//2.设置复用    相较于STM32F103而言，407的外设没有默认引脚（103有default Alternate Functions），要使用某外设，便需要对相应引脚做复用，调用GPIO_PinAFConfig()函数
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_TIM2);   //PB10<-->TIM2-CH3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_TIM2);  //PAB11<-->TIM2-CH4

	//3.设置GPIO	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;           //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	   //速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;         //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;           //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure);                  //初始化
	
	
	TIM_TimeBaseStructure.TIM_Prescaler = psc;             //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;//TIM_CounterMode_CenterAligned1; //中央对齐模式
	TIM_TimeBaseStructure.TIM_Period = arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);//初始化定时器2
	
	//初始化TIM2 PWM模式		
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//TIM_OCMode_PWM2;              //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能，无需输出
//    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
//    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
//    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
//    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
//    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_SetCompare3(TIM2, 0); 
    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable); // 使能TIM2在CCR3上的预装载寄存器
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);

    TIM_SetCompare4(TIM2, 0); 
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable); // 使能TIM2在CCR4上的预装载寄存器
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	
	TIM_ARRPreloadConfig(TIM2,ENABLE);                 //ARPE使能 
		
	TIM_Cmd(TIM2, ENABLE);            //使能TIM2
	
}
