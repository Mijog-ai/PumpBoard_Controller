/********************************************************************************
 Author : CAC (CustomerApplications Center, Asia)

 Date :	Jan,2013 

 File name : AD7949.h

 Description : Use the SPI(simulated by ADuC7026 GPIO) to communicate with AD7949

 Hardware plateform : ADuC7026 Eval Board Rev.B1 and EVAL-AD7949EDZ 	
********************************************************************************/

#ifndef _AD7689_H_
#define _AD7689_H_


#define ADC_CHANNEL_NUM                     8
#define ADC_FILTER_FIFO                     40

#define CFG_KEEP				            0x0
#define CFG_OVERWRITE			            0x2000  //10 0000 0000 0000 强制更新配置

#define INCC_BIPOLAR_DIFFERENTIAL_PAIRS		0x0
#define INCC_BIPOLAR						0x800
#define INCC_TEMPERATURE_SENSOR				0xC00
#define INCC_UNIPOLAR_DIFFERENTIAL_PAIRS	0x1000
#define INCC_UNIPOLAR_TO_COM				0x1800	//01 1000 0000 0000 单端模式，INx 参考 COM = GND ± 0.1 V
#define INCC_UNIPOLAR_TO_GND				0x1C00

//通道选择
#define IN0									0x0
#define IN1									0x80	//00 0000 1000 0000
#define IN2									0x100	//00 0001 0000 0000
#define IN3									0x180
#define IN4									0x200
#define IN5									0x280
#define IN6									0x300
#define IN7									0x380

#define BW_0_25								0x0
#define BW_FULL								0x40 //00 0000 0100 0000 全带宽模式

#define REF_IN_2V5							0x0	 //00 0000 0000 0000 参考电压2.5V	
#define REF_IN_4V096						0x8  //00 0000 0000 1000 参考电压4.096V
#define REF_EX_TEM_ENABLE					0x10
#define REF_EX_IN_BUFFER_TEM_ENABLE			0x18
#define REF_EX_TEM_DISABLE					0x30
#define REF_EX_IN_BUFFER_TEM_DISABLE		0x38

#define SEQ_DISABLE									0x0	 		//Disable sequencer
#define SEQ_UPDATA_CONFIGURATION_DURING_SEQUENCE	0x2
#define SEQ_SCAN_TEM								0x4
#define SEQ_SCAN									0x6

#define RB_DISABLE									0x1			//1 = do not read back contents of configuration.
#define RB_ENABLE									0x0				
//11 1000 0100 0001

#define SET_CS()		GP2DAT = (GP2DAT | 0x00800000)	//P2.7->/CS
#define CLR_CS()		GP2DAT = (GP2DAT & 0xFF7FFFFF)

#define	SET_SCL()		GP1DAT = (GP1DAT | 0x00100000)	//P1.4->SCLK
#define	CLR_SCL()		GP1DAT = (GP1DAT & 0xffEFffff)

#define SET_SDO()		GP1DAT = (GP1DAT | 0x00400000)	//P1.6->SDO
#define CLR_SDO()		GP1DAT = (GP1DAT & 0xffBFffff)	

void AD7689_reg_write(unsigned int RegisterData);
void AD7689InitFunc(void);
unsigned int AD7689_data_read(void);
u8 SPI_ReadWriteMultiBytes(u16 TxData, u8* pRxData, u16 Size);

#endif
