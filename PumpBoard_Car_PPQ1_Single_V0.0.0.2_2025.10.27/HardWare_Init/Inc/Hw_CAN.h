/********************************************************************************
 * File name    : Hw_CAN.h
 * Description  : CAN bus hardware initialization and low-level driver
 * Author       : PumpBoard Team
 * Date         : 2025-12-01
 * Hardware     : STM32F407ZE
 *               CAN1: PA11 (RX), PA12 (TX)
 *               CAN2: PB12 (RX), PB13 (TX) [Optional]
 ********************************************************************************/

#ifndef __HW_CAN_H
#define __HW_CAN_H

#include "stm32f4xx.h"

/* CAN Baud Rate Options */
#define CAN_BAUDRATE_125K    125
#define CAN_BAUDRATE_250K    250
#define CAN_BAUDRATE_500K    500
#define CAN_BAUDRATE_1000K   1000

/* CAN Status Return Codes */
#define CAN_OK               0
#define CAN_ERROR            1
#define CAN_TIMEOUT          2

/* CAN Message Buffer Sizes */
#define CAN_TX_BUFFER_SIZE   16
#define CAN_RX_BUFFER_SIZE   32

/* Function Prototypes */
void CAN1_GPIO_Config(void);
void CAN2_GPIO_Config(void);
u8 CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode);
u8 CAN2_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode);
u8 CAN_Transmit_Msg(CAN_TypeDef* CANx, u32 id, u8* msg, u8 len, u8 id_type);
u8 CAN_Receive_Msg(CAN_TypeDef* CANx, u32* id, u8* buf, u8* len);
void CAN1_NVIC_Config(void);
void CAN2_NVIC_Config(void);

#endif /* __HW_CAN_H */
