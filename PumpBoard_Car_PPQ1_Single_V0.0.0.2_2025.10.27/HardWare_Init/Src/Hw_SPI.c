#include "stm32f4xx.h"
#include "Hw_spi.h"
#include "gpio.h"




void SPI3_Int(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);                     //使能SPI3时钟
   
    //复用功能打开
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI3);
	//PB5--MOSI      PB3--SCK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;                        //复用输出口也需要配置端口速度	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//PB4--MISO  SDO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//ADC--PE1--NSS  CNV
    GPIO_InitStructure.GPIO_Pin = ADC7689_NSS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
    GPIO_Init(ADC7689_NSS_GPIO, &GPIO_InitStructure);//
    //默认拉高
    ADC7689_NSS_GPIO->BSRRL |=  ADC7689_NSS_PIN;

   //////SPI模块配置//////
	SPI_Cmd(SPI3, DISABLE);                                                 //必须先禁能,才能改变MODE
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;		//全双工模式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;							//主
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;						//SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//SPI_CPOL_Low; 							    //CPOL=1 CLK的空闲电平               2024.04.08将高电平改为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;///SPI_CPHA_2Edge;							//CPHA=1 在第二个时钟沿捕获数据
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;								//软件NSS（如果是硬件NSS的话注意需要调用SPI_SSOutputCmd）
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;//SPI_BaudRatePrescaler_128;//SPI_BaudRatePrescaler_16;	    //5.25Mhz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;						//高位在前
	SPI_InitStructure.SPI_CRCPolynomial = 7;								//CRC7
	SPI_Init(SPI3, &SPI_InitStructure);
	SPI_Cmd(SPI3, ENABLE);
	
}




