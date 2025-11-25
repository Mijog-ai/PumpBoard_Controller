#include "gpio.h"
#include "AD7689.h"	
#include "Hw_spi.h"


#define ADC7689_SPI         SPI3

#define SPI_TIMEOUTCNT		5000



/* 修改初始化函数 */
void AD7689InitFunc()
{
	u16 AD7689_CFG_Reg = CFG_OVERWRITE | INCC_UNIPOLAR_TO_COM | IN0 | BW_FULL | REF_IN_2V5 | SEQ_DISABLE | RB_DISABLE;
	
	u16 AD7689_rxBuf = 0;
	
	SPI_4WriteByte(AD7689_CFG_Reg, &AD7689_rxBuf);
    
    SPI_4WriteByte(AD7689_CFG_Reg, &AD7689_rxBuf);
    
    SPI_4WriteByte(AD7689_CFG_Reg, &AD7689_rxBuf);

}


void AD7689_NSS_H(void)
{	
	ADC7689_NSS_GPIO->BSRRL = ADC7689_NSS_PIN;
	__DSB();
}

void AD7689_NSS_L(void)
{
    ADC7689_NSS_GPIO->BSRRH = ADC7689_NSS_PIN;
	__DSB();
}

u8 SPI_4WriteByte(u16 Addr, u16* data)
{
	u32 AD7689_CS_L_Delay = 80;
    u16 timeoutcnt = 0;
    u8 result = 0;
    
    timeoutcnt = ADC7689_SPI->DR;     //就是为了清溢出标志
    
	AD7689_NSS_H();                 //拉高，转换
	
	
    while(AD7689_CS_L_Delay > 1)
    {
        AD7689_CS_L_Delay --;       
    }
    AD7689_NSS_L();                 //拉低

    
    AD7689_CS_L_Delay = 80;     //拉低之后继续延时，等待数据可用
    while(AD7689_CS_L_Delay > 1)
    {
        AD7689_CS_L_Delay --;       
    }
    
	
	result = SPI_ReadWriteByte(Addr, data);		//写数据
    
	if(result != SPI_SUCCESS)
	{
		return result;
	}
	AD7689_NSS_H();

	return SPI_SUCCESS;
}


u8 SPI_ReadWriteByte(u16 TxData, u16* pRxData)
{
	u16 timeoutcnt = 0;
	
	//等待发送缓冲区空
	timeoutcnt = SPI_TIMEOUTCNT;
	while (((ADC7689_SPI->SR & SPI_I2S_FLAG_TXE) == RESET) && timeoutcnt)
	{
		timeoutcnt--;
	}
	if(!timeoutcnt)
		return SPI_TIMEOUT;// */
	//发两个字节
	ADC7689_SPI->DR = TxData;
	//等待数据接收
	timeoutcnt = SPI_TIMEOUTCNT;
	while (((ADC7689_SPI->SR & SPI_I2S_FLAG_RXNE) == RESET) && timeoutcnt)
	{
		timeoutcnt--;
	}
    
	if(!timeoutcnt)
		return SPI_TIMEOUT;
	//取数据（标志自动清零）
	timeoutcnt = ADC7689_SPI->DR;
	if(pRxData != 0)
		*pRxData = timeoutcnt;
	return SPI_SUCCESS;
}

/**
  * @brief  SPI连续读写多字节（16位数据帧模式）
  * @param  TxData: 发送的16位数据（首次传输）
  * @param  pRxData: 接收数据缓冲区（需确保足够大）
  * @param  Size: 要接收的总字节数
  * @retval SPI状态（SPI_SUCCESS/SPI_TIMEOUT）
  */
u8 SPI_ReadWriteMultiBytes(u16 TxData, u8* pRxData, u16 Size)
{
    u16 timeoutcnt = 0;
    u16 tx_word = TxData;  // 首次发送的数据
    u16 rx_word;
    u8* pBuf = pRxData;
    u16 bytes_remaining = Size;

    // 首次传输：发送TxData并接收第一个16位数据
    // 等待发送缓冲区空
    timeoutcnt = SPI_TIMEOUTCNT;
    while (((ADC7689_SPI->SR & SPI_I2S_FLAG_TXE) == RESET) && timeoutcnt)
        timeoutcnt--;
    if (!timeoutcnt)
        return SPI_TIMEOUT;
    
    ADC7689_SPI->DR = tx_word;  // 启动首次传输

    // 循环处理所有待接收字节
    while (bytes_remaining > 0)
    {
        // 等待接收完成
        timeoutcnt = SPI_TIMEOUTCNT;
        while (((ADC7689_SPI->SR & SPI_I2S_FLAG_RXNE) == RESET) && timeoutcnt)
            timeoutcnt--;
        if (!timeoutcnt)
            return SPI_TIMEOUT;

        // 读取接收到的16位数据
        rx_word = ADC7689_SPI->DR;

        // 拆分为两个字节存入缓冲区（MSB First）
        if (bytes_remaining >= 2)
        {
            *pBuf++ = (rx_word >> 8) & 0xFF;  // 高字节
            *pBuf++ = rx_word & 0xFF;         // 低字节
            bytes_remaining -= 2;
        }
        else  // 处理奇数长度（最后1字节）
        {
            *pBuf++ = (rx_word >> 8) & 0xFF;
            bytes_remaining--;
        }

        // 准备下次发送（哑数据0x0000）
        if (bytes_remaining > 0)
        {
            // 等待发送缓冲区空
            timeoutcnt = SPI_TIMEOUTCNT;
            while (((ADC7689_SPI->SR & SPI_I2S_FLAG_TXE) == RESET) && timeoutcnt)
                timeoutcnt--;
            if (!timeoutcnt)
                return SPI_TIMEOUT;
            
            ADC7689_SPI->DR = 0x0000;  // 后续发送哑数据以维持SCK
        }
    }

    return SPI_SUCCESS;
}


u8 SPI_BatchWrite(u16 *txBuffer, u8 bytesToWrite)
{
    int i;
    u16 timeoutcnt = 0;
    u8 result = 0;

    for (i = 0; i < bytesToWrite; i++)
    {        
        if (i == bytesToWrite - 1)                              //△注意可能会‘死’在其中
        {
            timeoutcnt = SPI_TIMEOUTCNT;
            while ((ADC7689_SPI->SR & SPI_I2S_FLAG_RXNE) == RESET && timeoutcnt); //目的就是个延时，防止CS信号提前被拉高
        }
    }   
    return SPI_SUCCESS;
}

