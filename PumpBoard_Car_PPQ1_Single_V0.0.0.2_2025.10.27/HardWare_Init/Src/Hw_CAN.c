/********************************************************************************
 * File name    : Hw_CAN.c
 * Description  : CAN bus hardware initialization and low-level driver
 * Author       : PumpBoard Team
 * Date         : 2025-12-01
 ********************************************************************************/

#include "Hw_CAN.h"
#include "gpio.h"

/*******************************************************************************
* Function Name : CAN1_GPIO_Config
* Description   : Configure CAN1 GPIO pins (PA11=RX, PA12=TX)
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void CAN1_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOA clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* Enable CAN1 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* Connect CAN1 pins to AF9 */
    GPIO_PinAFConfig(CAN1_RX_PORT, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(CAN1_TX_PORT, GPIO_PinSource12, GPIO_AF_CAN1);

    /* Configure CAN1 RX pin */
    GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(CAN1_RX_PORT, &GPIO_InitStructure);

    /* Configure CAN1 TX pin */
    GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
    GPIO_Init(CAN1_TX_PORT, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name : CAN2_GPIO_Config
* Description   : Configure CAN2 GPIO pins (PB12=RX, PB13=TX)
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void CAN2_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOB clock */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    /* Enable CAN1 and CAN2 clocks (CAN2 needs CAN1 clock) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

    /* Connect CAN2 pins to AF9 */
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);

    /* Configure CAN2 RX pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Configure CAN2 TX pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*******************************************************************************
* Function Name : CAN1_NVIC_Config
* Description   : Configure CAN1 NVIC interrupts
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void CAN1_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* CAN1 RX0 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* CAN1 TX interrupt (optional) */
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name : CAN2_NVIC_Config
* Description   : Configure CAN2 NVIC interrupts
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void CAN2_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* CAN2 RX0 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name : CAN1_Mode_Init
* Description   : Initialize CAN1 with specified timing parameters
* Input         : tsjw  - Synchronization jump width
*                 tbs2  - Time segment 2
*                 tbs1  - Time segment 1
*                 brp   - Baud rate prescaler
*                 mode  - CAN mode (Normal, Loopback, Silent, etc.)
* Output        : None
* Return        : 0=Success, 1=Failure
* Example       : CAN1_Mode_Init(CAN_SJW_1tq, CAN_BS2_6tq, CAN_BS1_7tq, 6, CAN_Mode_Normal)
*                 -> 500kbps at 42MHz APB1 clock
*******************************************************************************/
u8 CAN1_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    /* Configure GPIO pins */
    CAN1_GPIO_Config();

    /* Configure NVIC */
    CAN1_NVIC_Config();

    /* CAN cell configuration */
    CAN_DeInit(CAN1);

    CAN_InitStructure.CAN_TTCM = DISABLE;           /* Time-triggered communication mode */
    CAN_InitStructure.CAN_ABOM = ENABLE;            /* Automatic bus-off management */
    CAN_InitStructure.CAN_AWUM = ENABLE;            /* Automatic wake-up mode */
    CAN_InitStructure.CAN_NART = DISABLE;           /* Non-automatic retransmission */
    CAN_InitStructure.CAN_RFLM = DISABLE;           /* Receive FIFO locked mode */
    CAN_InitStructure.CAN_TXFP = ENABLE;            /* Transmit FIFO priority */
    CAN_InitStructure.CAN_Mode = mode;              /* CAN operating mode */

    /* CAN Baudrate configuration */
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;

    if (CAN_Init(CAN1, &CAN_InitStructure) != CAN_InitStatus_Success)
    {
        return 1; /* Initialization failed */
    }

    /* Configure CAN filter - Accept all messages */
    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* Enable CAN RX interrupt */
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    return 0; /* Success */
}

/*******************************************************************************
* Function Name : CAN2_Mode_Init
* Description   : Initialize CAN2 with specified timing parameters
* Input         : Same as CAN1_Mode_Init
* Output        : None
* Return        : 0=Success, 1=Failure
*******************************************************************************/
u8 CAN2_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode)
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    /* Configure GPIO pins */
    CAN2_GPIO_Config();

    /* Configure NVIC */
    CAN2_NVIC_Config();

    /* CAN cell configuration */
    CAN_DeInit(CAN2);

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = ENABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = ENABLE;
    CAN_InitStructure.CAN_Mode = mode;

    /* CAN Baudrate configuration */
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;

    if (CAN_Init(CAN2, &CAN_InitStructure) != CAN_InitStatus_Success)
    {
        return 1;
    }

    /* Configure CAN2 filter - Must use filter banks 14-27 */
    CAN_FilterInitStructure.CAN_FilterNumber = 14;  /* CAN2 starts at filter 14 */
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    /* Enable CAN RX interrupt */
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

    return 0;
}

/*******************************************************************************
* Function Name : CAN_Transmit_Msg
* Description   : Transmit a CAN message
* Input         : CANx     - CAN1 or CAN2
*                 id       - CAN identifier
*                 msg      - Pointer to message data
*                 len      - Message length (0-8 bytes)
*                 id_type  - 0=Standard ID (11-bit), 1=Extended ID (29-bit)
* Output        : None
* Return        : 0=Success, 1=Error, 2=Timeout
*******************************************************************************/
u8 CAN_Transmit_Msg(CAN_TypeDef* CANx, u32 id, u8* msg, u8 len, u8 id_type)
{
    CanTxMsg TxMessage;
    u8 mbox;
    u16 timeout = 0;

    /* Configure message */
    if (id_type == 0)
    {
        TxMessage.StdId = id;           /* Standard identifier */
        TxMessage.IDE = CAN_Id_Standard;
    }
    else
    {
        TxMessage.ExtId = id;           /* Extended identifier */
        TxMessage.IDE = CAN_Id_Extended;
    }

    TxMessage.RTR = CAN_RTR_Data;       /* Data frame */
    TxMessage.DLC = len;                /* Data length */

    /* Copy data */
    for (u8 i = 0; i < len && i < 8; i++)
    {
        TxMessage.Data[i] = msg[i];
    }

    /* Transmit message */
    mbox = CAN_Transmit(CANx, &TxMessage);

    /* Wait for transmission complete (with timeout) */
    while ((CAN_TransmitStatus(CANx, mbox) != CAN_TxStatus_Ok) && (timeout < 0xFFF))
    {
        timeout++;
    }

    if (timeout >= 0xFFF)
    {
        return CAN_TIMEOUT;
    }

    return CAN_OK;
}

/*******************************************************************************
* Function Name : CAN_Receive_Msg
* Description   : Receive a CAN message (polling mode)
* Input         : CANx - CAN1 or CAN2
* Output        : id   - Pointer to store received ID
*                 buf  - Pointer to buffer for received data
*                 len  - Pointer to store received data length
* Return        : 0=Success, 1=No message available
*******************************************************************************/
u8 CAN_Receive_Msg(CAN_TypeDef* CANx, u32* id, u8* buf, u8* len)
{
    CanRxMsg RxMessage;

    /* Check if message is available */
    if (CAN_MessagePending(CANx, CAN_FIFO0) == 0)
    {
        return 1; /* No message */
    }

    /* Receive message */
    CAN_Receive(CANx, CAN_FIFO0, &RxMessage);

    /* Extract ID */
    if (RxMessage.IDE == CAN_Id_Standard)
    {
        *id = RxMessage.StdId;
    }
    else
    {
        *id = RxMessage.ExtId;
    }

    /* Extract data length */
    *len = RxMessage.DLC;

    /* Copy data */
    for (u8 i = 0; i < RxMessage.DLC && i < 8; i++)
    {
        buf[i] = RxMessage.Data[i];
    }

    return CAN_OK;
}
