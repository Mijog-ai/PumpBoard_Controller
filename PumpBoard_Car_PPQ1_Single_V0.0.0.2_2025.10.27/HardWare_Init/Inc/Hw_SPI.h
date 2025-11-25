#ifndef SPI_H
#define SPI_H

#include "stdint.h"

#define	SPI_SUCCESS							0
#define	SPI_TIMEOUT							1
#define	SPI_ILLEGAL							2
#define SPI_DATAERROR						3
#define SPI_FIRSTTIME						4

void SPI_Int(void);					
//uint8_t SPI_4ReadByte(uint8_t Addr, uint8_t* pRxData);
uint8_t SPI_4WriteByte(uint16_t Addr, uint16_t* data);
uint8_t SPI_ReadWriteByte(uint16_t TxData, uint16_t* pRxData);

uint8_t SPI_BatchWrite(uint16_t *txBuffer, uint8_t bytesToWrite);

void AD7689_NSS_H(void);
void AD7689_NSS_L(void);

void SPI3_Int(void);
#endif
