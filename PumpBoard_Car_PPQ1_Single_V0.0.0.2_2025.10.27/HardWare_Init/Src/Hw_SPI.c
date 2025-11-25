#include "stm32f4xx.h"
#include "Hw_spi.h"
#include "gpio.h"




void SPI3_Int(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);                     //Enable SPI3 clock

    //Enable alternate function
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI3);
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI3);
	//PB5--MOSI      PB3--SCK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;                        //Alternate output also needs port speed configuration
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
    //Default pull high
    ADC7689_NSS_GPIO->BSRRL |=  ADC7689_NSS_PIN;

   //////SPI module configuration//////
	SPI_Cmd(SPI3, DISABLE);                                                 //Must disable first before changing MODE
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;		//Full duplex mode
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;							//Master
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;						//SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//SPI_CPOL_Low; 							    //CPOL=1 CLK idle level               2024.04.08 changed from high to low
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;///SPI_CPHA_2Edge;							//CPHA=1 Capture data on second clock edge
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;								//Software NSS (if hardware NSS, need to call SPI_SSOutputCmd)
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;//SPI_BaudRatePrescaler_128;//SPI_BaudRatePrescaler_16;	    //5.25Mhz
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;						//MSB first
	SPI_InitStructure.SPI_CRCPolynomial = 7;								//CRC7
	SPI_Init(SPI3, &SPI_InitStructure);
	SPI_Cmd(SPI3, ENABLE);
	
}




