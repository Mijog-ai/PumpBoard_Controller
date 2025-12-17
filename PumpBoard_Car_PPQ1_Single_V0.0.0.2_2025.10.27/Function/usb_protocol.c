/**
 * @file usb_protocol.c
 * @brief USB/UART Communication Protocol Handler for PumpBoard Controller
 *
 * @author PumpBoard Project
 * @version 1.0.0
 */

#include "usb_protocol.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include "PID.h"

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

USB_ProtocolHandler_t g_usb_protocol;

/* System tick counter (should be incremented in SysTick handler) */
extern volatile uint32_t g_system_tick_ms;

/* External references from application */
extern _FEEDBACK_VALUE     DetectorFdbVal;
extern _CMD_GEN            InstructionSet;
extern _PPQ_ENA_CMD        PpqEnable;
extern _SYS_PARA           SysParameter;
extern _LOW_PASS_FILTER_PARA FilterTimConstant;
extern u32                 g_u32_PWMOutput_A;
extern u32                 g_u32_PWMOutput_B;
extern u8                  g_u8_SimulationMode;

/* PID controllers from PID.h */
extern pid_location_t Pwm_Output_A;
extern pid_location_t Pwm_Output_B;
extern pid_location_t Angle_Loop;
extern pid_location_t Pressure_Loop;

/* Analog inputs */
extern u16 g_u16_CURRENT_A_CHECK;
extern u16 g_u16_CURRENT_B_CHECK;
extern u16 g_u16_SENSOR_OVER_CURRENT;
extern u16 g_u16_ANALOG_01_OUTPUT_CHECK;
extern u16 g_u16_ANALOG_02_OUTPUT_CHECK;
extern u16 g_u16_AD7689_Ang_Val;
extern u16 g_u16_AD7689_Prs_Val;

/*============================================================================
 * USART CONFIGURATION (USART1 on PA9/PA10 for USB-UART Bridge)
 *============================================================================*/

#define USB_USART               USART1
#define USB_USART_CLK           RCC_APB2Periph_USART1
#define USB_USART_GPIO_CLK      RCC_AHB1Periph_GPIOA
#define USB_USART_TX_PIN        GPIO_Pin_9
#define USB_USART_RX_PIN        GPIO_Pin_10
#define USB_USART_GPIO          GPIOA
#define USB_USART_TX_SOURCE     GPIO_PinSource9
#define USB_USART_RX_SOURCE     GPIO_PinSource10
#define USB_USART_AF            GPIO_AF_USART1
#define USB_USART_IRQn          USART1_IRQn

/* Connection timeout in ms */
#define USB_CONNECTION_TIMEOUT  3000

/*============================================================================
 * INITIALIZATION
 *============================================================================*/

void USB_Protocol_Init(void)
{
    /* Clear protocol handler structure */
    memset(&g_usb_protocol, 0, sizeof(USB_ProtocolHandler_t));

    /* Initialize state machine */
    g_usb_protocol.rx_state = USB_RX_WAIT_START;

    /* Default streaming configuration */
    g_usb_protocol.stream_enabled = 0;
    g_usb_protocol.stream_rate_ms = 50;  /* 20 Hz default */
    g_usb_protocol.stream_data_mask = USB_STREAM_ALL;
}

void USB_UART_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clocks */
    RCC_APB2PeriphClockCmd(USB_USART_CLK, ENABLE);
    RCC_AHB1PeriphClockCmd(USB_USART_GPIO_CLK, ENABLE);

    /* Configure GPIO for USART TX */
    GPIO_InitStruct.GPIO_Pin = USB_USART_TX_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(USB_USART_GPIO, &GPIO_InitStruct);

    /* Configure GPIO for USART RX */
    GPIO_InitStruct.GPIO_Pin = USB_USART_RX_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(USB_USART_GPIO, &GPIO_InitStruct);

    /* Connect pins to USART alternate function */
    GPIO_PinAFConfig(USB_USART_GPIO, USB_USART_TX_SOURCE, USB_USART_AF);
    GPIO_PinAFConfig(USB_USART_GPIO, USB_USART_RX_SOURCE, USB_USART_AF);

    /* Configure USART */
    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USB_USART, &USART_InitStruct);

    /* Enable RX interrupt */
    USART_ITConfig(USB_USART, USART_IT_RXNE, ENABLE);

    /* Configure NVIC for USART interrupt */
    NVIC_InitStruct.NVIC_IRQChannel = USB_USART_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable USART */
    USART_Cmd(USB_USART, ENABLE);

    /* Initialize protocol handler */
    USB_Protocol_Init();
}

/*============================================================================
 * CRC16 CALCULATION
 *============================================================================*/

uint16_t USB_CRC16_CCITT(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t j;

    for (i = 0; i < length; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

/*============================================================================
 * TIMESTAMP
 *============================================================================*/

uint32_t USB_GetTimestamp(void)
{
    return g_system_tick_ms;
}

/*============================================================================
 * BYTE TRANSMISSION
 *============================================================================*/

static void USB_SendByte(uint8_t byte)
{
    while (USART_GetFlagStatus(USB_USART, USART_FLAG_TXE) == RESET);
    USART_SendData(USB_USART, byte);
}

static void USB_SendBuffer(const uint8_t* buffer, uint16_t length)
{
    uint16_t i;
    for (i = 0; i < length; i++) {
        USB_SendByte(buffer[i]);
    }
}

/*============================================================================
 * RECEIVE STATE MACHINE
 *============================================================================*/

void USB_Protocol_ProcessByte(uint8_t byte)
{
    switch (g_usb_protocol.rx_state) {
        case USB_RX_WAIT_START:
            if (byte == USB_PROTOCOL_START_BYTE) {
                g_usb_protocol.rx_state = USB_RX_GET_CMD;
            }
            break;

        case USB_RX_GET_CMD:
            g_usb_protocol.rx_cmd = byte;
            g_usb_protocol.rx_state = USB_RX_GET_LENGTH;
            break;

        case USB_RX_GET_LENGTH:
            g_usb_protocol.rx_length = byte;
            g_usb_protocol.rx_payload_idx = 0;
            if (g_usb_protocol.rx_length > 0 &&
                g_usb_protocol.rx_length <= USB_MAX_PAYLOAD_SIZE) {
                g_usb_protocol.rx_state = USB_RX_GET_PAYLOAD;
            } else if (g_usb_protocol.rx_length == 0) {
                g_usb_protocol.rx_state = USB_RX_GET_CRC_L;
            } else {
                /* Invalid length, reset */
                g_usb_protocol.rx_error_count++;
                g_usb_protocol.rx_state = USB_RX_WAIT_START;
            }
            break;

        case USB_RX_GET_PAYLOAD:
            g_usb_protocol.rx_payload[g_usb_protocol.rx_payload_idx++] = byte;
            if (g_usb_protocol.rx_payload_idx >= g_usb_protocol.rx_length) {
                g_usb_protocol.rx_state = USB_RX_GET_CRC_L;
            }
            break;

        case USB_RX_GET_CRC_L:
            g_usb_protocol.rx_crc = byte;
            g_usb_protocol.rx_state = USB_RX_GET_CRC_H;
            break;

        case USB_RX_GET_CRC_H:
            g_usb_protocol.rx_crc |= ((uint16_t)byte << 8);
            g_usb_protocol.rx_state = USB_RX_GET_END;
            break;

        case USB_RX_GET_END:
            if (byte == USB_PROTOCOL_END_BYTE) {
                /* Verify CRC */
                uint8_t crc_data[USB_MAX_PAYLOAD_SIZE + 2];
                uint16_t crc_len = 0;
                uint16_t calc_crc;

                crc_data[crc_len++] = g_usb_protocol.rx_cmd;
                crc_data[crc_len++] = g_usb_protocol.rx_length;
                memcpy(&crc_data[crc_len], g_usb_protocol.rx_payload,
                       g_usb_protocol.rx_length);
                crc_len += g_usb_protocol.rx_length;

                calc_crc = USB_CRC16_CCITT(crc_data, crc_len);

                if (calc_crc == g_usb_protocol.rx_crc) {
                    /* Valid frame received */
                    g_usb_protocol.rx_frame_count++;
                    g_usb_protocol.last_rx_tick = USB_GetTimestamp();
                    g_usb_protocol.connected = 1;
                    USB_Protocol_ProcessFrame();
                } else {
                    /* CRC mismatch */
                    g_usb_protocol.crc_error_count++;
                    USB_Protocol_SendNack(g_usb_protocol.rx_cmd, USB_ERR_CRC_MISMATCH);
                }
            } else {
                /* Frame error */
                g_usb_protocol.rx_error_count++;
            }
            g_usb_protocol.rx_state = USB_RX_WAIT_START;
            break;

        default:
            g_usb_protocol.rx_state = USB_RX_WAIT_START;
            break;
    }
}

/*============================================================================
 * SEND RESPONSE
 *============================================================================*/

void USB_Protocol_SendResponse(uint8_t cmd, const uint8_t* payload, uint8_t length)
{
    uint8_t frame[USB_MAX_FRAME_SIZE];
    uint16_t idx = 0;
    uint16_t crc;
    uint8_t crc_data[USB_MAX_PAYLOAD_SIZE + 2];
    uint16_t crc_len = 0;

    /* Build CRC data */
    crc_data[crc_len++] = cmd;
    crc_data[crc_len++] = length;
    if (length > 0 && payload != NULL) {
        memcpy(&crc_data[crc_len], payload, length);
        crc_len += length;
    }
    crc = USB_CRC16_CCITT(crc_data, crc_len);

    /* Build frame */
    frame[idx++] = USB_PROTOCOL_START_BYTE;
    frame[idx++] = cmd;
    frame[idx++] = length;
    if (length > 0 && payload != NULL) {
        memcpy(&frame[idx], payload, length);
        idx += length;
    }
    frame[idx++] = (uint8_t)(crc & 0xFF);
    frame[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    frame[idx++] = USB_PROTOCOL_END_BYTE;

    /* Send frame */
    USB_SendBuffer(frame, idx);
    g_usb_protocol.tx_frame_count++;
}

void USB_Protocol_SendAck(uint8_t cmd)
{
    uint8_t payload = cmd;
    USB_Protocol_SendResponse(USB_CMD_ACK, &payload, 1);
}

void USB_Protocol_SendNack(uint8_t cmd, uint8_t error_code)
{
    uint8_t payload[2] = {cmd, error_code};
    USB_Protocol_SendResponse(USB_CMD_NACK, payload, 2);
}

/*============================================================================
 * COMMAND HANDLERS
 *============================================================================*/

static void USB_Handle_GetVersion(void)
{
    USB_SystemStatus_t status;

    status.version_major = 1;
    status.version_minor = 0;
    status.version_patch = 0;
    status.status_flags = 0;
    status.error_code = 0;
    status.enable_flags = (PpqEnable.PumpStrt_Ena ? USB_ENABLE_PUMP_START : 0) |
                          (PpqEnable.AngLeak_Ena ? USB_ENABLE_ANGLE_LEAK : 0) |
                          (PpqEnable.PrsLoop_Ena ? USB_ENABLE_PRESSURE_LOOP : 0) |
                          (PpqEnable.PwrLoop_Ena ? USB_ENABLE_POWER_LOOP : 0) |
                          (PpqEnable.ParaSaveEna ? USB_ENABLE_PARA_SAVE : 0);
    status.simulation_mode = g_u8_SimulationMode;
    status.reserved = 0;

    USB_Protocol_SendResponse(USB_CMD_GET_VERSION,
                             (uint8_t*)&status, sizeof(status));
}

static void USB_Handle_GetAllFeedback(void)
{
    USB_FeedbackData_t data;

    data.timestamp_ms = USB_GetTimestamp();
    data.angle_feedback = DetectorFdbVal.g_u32_AngFdb;
    data.angle_ref = InstructionSet.TiltAngRef;
    data.angle_diff = DetectorFdbVal.g_s32_AngFdb_D;
    data.pressure_feedback = DetectorFdbVal.g_u32_PressureFdb;
    data.pressure_ref = InstructionSet.g_u16_PerUnitPrsVal;
    data.pressure_diff = DetectorFdbVal.g_s32_PressureFdb_D;
    data.current_a_feedback = DetectorFdbVal.g_s32_CurFdb_A;
    data.current_a_ref = InstructionSet.Cur_A_Ref;
    data.current_b_feedback = DetectorFdbVal.g_s32_CurFdb_B;
    data.current_b_ref = InstructionSet.Cur_B_Ref;
    data.pwm_output_a = g_u32_PWMOutput_A;
    data.pwm_output_b = g_u32_PWMOutput_B;
    data.enable_flags = (PpqEnable.PumpStrt_Ena ? USB_ENABLE_PUMP_START : 0) |
                        (PpqEnable.AngLeak_Ena ? USB_ENABLE_ANGLE_LEAK : 0) |
                        (PpqEnable.PrsLoop_Ena ? USB_ENABLE_PRESSURE_LOOP : 0) |
                        (PpqEnable.PwrLoop_Ena ? USB_ENABLE_POWER_LOOP : 0);
    data.error_code = 0;  /* TODO: Add error detection */
    data.reserved = 0;

    USB_Protocol_SendResponse(USB_CMD_GET_ALL_FEEDBACK,
                             (uint8_t*)&data, sizeof(data));
}

static void USB_Handle_GetAnalogInput(void)
{
    USB_AnalogInputs_t inputs;

    inputs.sensor_overcurrent = g_u16_SENSOR_OVER_CURRENT;
    inputs.analog_out1_check = g_u16_ANALOG_01_OUTPUT_CHECK;
    inputs.analog_out2_check = g_u16_ANALOG_02_OUTPUT_CHECK;
    inputs.current_a_adc = g_u16_CURRENT_A_CHECK;
    inputs.current_b_adc = g_u16_CURRENT_B_CHECK;
    inputs.angle_adc = g_u16_AD7689_Ang_Val;
    inputs.pressure_adc = g_u16_AD7689_Prs_Val;
    inputs.reserved = 0;

    USB_Protocol_SendResponse(USB_CMD_GET_ANALOG_INPUT,
                             (uint8_t*)&inputs, sizeof(inputs));
}

static void USB_Handle_SetAngleRef(void)
{
    if (g_usb_protocol.rx_length >= 4) {
        int32_t angle_ref;
        memcpy(&angle_ref, g_usb_protocol.rx_payload, 4);

        /* Clamp to valid range */
        if (angle_ref < 0) angle_ref = 0;
        if (angle_ref > 10000) angle_ref = 10000;

        InstructionSet.TiltAngRef = (u16)angle_ref;
        USB_Protocol_SendAck(USB_CMD_SET_ANGLE_REF);
    } else {
        USB_Protocol_SendNack(USB_CMD_SET_ANGLE_REF, USB_ERR_INVALID_PARAM);
    }
}

static void USB_Handle_SetPressureRef(void)
{
    if (g_usb_protocol.rx_length >= 4) {
        int32_t pressure_ref;
        memcpy(&pressure_ref, g_usb_protocol.rx_payload, 4);

        /* Clamp to valid range */
        if (pressure_ref < 0) pressure_ref = 0;
        if (pressure_ref > 10000) pressure_ref = 10000;

        InstructionSet.g_u16_PerUnitPrsVal = (u16)pressure_ref;
        USB_Protocol_SendAck(USB_CMD_SET_PRESSURE_REF);
    } else {
        USB_Protocol_SendNack(USB_CMD_SET_PRESSURE_REF, USB_ERR_INVALID_PARAM);
    }
}

static void USB_Handle_SetCurrentRef(void)
{
    if (g_usb_protocol.rx_length >= 8) {
        int32_t current_a, current_b;
        memcpy(&current_a, &g_usb_protocol.rx_payload[0], 4);
        memcpy(&current_b, &g_usb_protocol.rx_payload[4], 4);

        /* Clamp to valid range (0-3000 mA) */
        if (current_a < 0) current_a = 0;
        if (current_a > 3000) current_a = 3000;
        if (current_b < 0) current_b = 0;
        if (current_b > 3000) current_b = 3000;

        InstructionSet.Cur_A_Ref = (u32)current_a;
        InstructionSet.Cur_B_Ref = (u32)current_b;
        USB_Protocol_SendAck(USB_CMD_SET_CURRENT_REF);
    } else {
        USB_Protocol_SendNack(USB_CMD_SET_CURRENT_REF, USB_ERR_INVALID_PARAM);
    }
}

static void USB_Handle_SetEnableFlags(void)
{
    if (g_usb_protocol.rx_length >= 1) {
        uint8_t flags = g_usb_protocol.rx_payload[0];

        PpqEnable.PumpStrt_Ena = (flags & USB_ENABLE_PUMP_START) ? 1 : 0;
        PpqEnable.AngLeak_Ena = (flags & USB_ENABLE_ANGLE_LEAK) ? 1 : 0;
        PpqEnable.PrsLoop_Ena = (flags & USB_ENABLE_PRESSURE_LOOP) ? 1 : 0;
        PpqEnable.PwrLoop_Ena = (flags & USB_ENABLE_POWER_LOOP) ? 1 : 0;
        PpqEnable.ParaSaveEna = (flags & USB_ENABLE_PARA_SAVE) ? 1 : 0;

        USB_Protocol_SendAck(USB_CMD_SET_ENABLE_FLAGS);
    } else {
        USB_Protocol_SendNack(USB_CMD_SET_ENABLE_FLAGS, USB_ERR_INVALID_PARAM);
    }
}

static void USB_Handle_GetPIDAngle(void)
{
    USB_PIDParams_t params;

    params.kp = Angle_Loop.kp;
    params.kp_div = Angle_Loop.kp_div;
    params.ki = Angle_Loop.ki;
    params.ki_div = Angle_Loop.ki_div;
    params.kd = Angle_Loop.kd;
    params.kd_div = Angle_Loop.kd_div;
    params.kv = Angle_Loop.kv;
    params.kv_div = Angle_Loop.kv_div;
    params.output_max = Angle_Loop.out_max;
    params.output_min = Angle_Loop.out_min;

    USB_Protocol_SendResponse(USB_CMD_GET_PID_ANGLE,
                             (uint8_t*)&params, sizeof(params));
}

static void USB_Handle_SetPIDAngle(void)
{
    if (g_usb_protocol.rx_length >= sizeof(USB_PIDParams_t)) {
        USB_PIDParams_t params;
        memcpy(&params, g_usb_protocol.rx_payload, sizeof(params));

        Angle_Loop.kp = params.kp;
        Angle_Loop.kp_div = params.kp_div;
        Angle_Loop.ki = params.ki;
        Angle_Loop.ki_div = params.ki_div;
        Angle_Loop.kd = params.kd;
        Angle_Loop.kd_div = params.kd_div;
        Angle_Loop.kv = params.kv;
        Angle_Loop.kv_div = params.kv_div;
        Angle_Loop.out_max = params.output_max;
        Angle_Loop.out_min = params.output_min;

        USB_Protocol_SendAck(USB_CMD_SET_PID_ANGLE);
    } else {
        USB_Protocol_SendNack(USB_CMD_SET_PID_ANGLE, USB_ERR_INVALID_PARAM);
    }
}

static void USB_Handle_GetPIDPressure(void)
{
    USB_PIDParams_t params;

    params.kp = Pressure_Loop.kp;
    params.kp_div = Pressure_Loop.kp_div;
    params.ki = Pressure_Loop.ki;
    params.ki_div = Pressure_Loop.ki_div;
    params.kd = Pressure_Loop.kd;
    params.kd_div = Pressure_Loop.kd_div;
    params.kv = Pressure_Loop.kv;
    params.kv_div = Pressure_Loop.kv_div;
    params.output_max = Pressure_Loop.out_max;
    params.output_min = Pressure_Loop.out_min;

    USB_Protocol_SendResponse(USB_CMD_GET_PID_PRESSURE,
                             (uint8_t*)&params, sizeof(params));
}

static void USB_Handle_SetPIDPressure(void)
{
    if (g_usb_protocol.rx_length >= sizeof(USB_PIDParams_t)) {
        USB_PIDParams_t params;
        memcpy(&params, g_usb_protocol.rx_payload, sizeof(params));

        Pressure_Loop.kp = params.kp;
        Pressure_Loop.kp_div = params.kp_div;
        Pressure_Loop.ki = params.ki;
        Pressure_Loop.ki_div = params.ki_div;
        Pressure_Loop.kd = params.kd;
        Pressure_Loop.kd_div = params.kd_div;
        Pressure_Loop.kv = params.kv;
        Pressure_Loop.kv_div = params.kv_div;
        Pressure_Loop.out_max = params.output_max;
        Pressure_Loop.out_min = params.output_min;

        USB_Protocol_SendAck(USB_CMD_SET_PID_PRESSURE);
    } else {
        USB_Protocol_SendNack(USB_CMD_SET_PID_PRESSURE, USB_ERR_INVALID_PARAM);
    }
}

static void USB_Handle_GetCalibration(void)
{
    USB_CalibrationParams_t params;

    params.angle_min_adc = SysParameter.u16AngAdc_MIN;
    params.angle_mid_adc = SysParameter.u16AngAdc_MID;
    params.angle_max_adc = SysParameter.u16AngAdc_MAX;
    params.angle_range = SysParameter.u16AngPerUnitScope;
    params.pressure_min_adc = SysParameter.u16PrsAdc_MIN;
    params.pressure_mid_adc = SysParameter.u16PrsAdc_MID;
    params.pressure_max_adc = SysParameter.u16PrsAdc_MAX;
    params.pressure_max_bar = SysParameter.u16PrsAdc_MAX_Scope;

    USB_Protocol_SendResponse(USB_CMD_GET_CALIBRATION,
                             (uint8_t*)&params, sizeof(params));
}

static void USB_Handle_SetCalibration(void)
{
    if (g_usb_protocol.rx_length >= sizeof(USB_CalibrationParams_t)) {
        USB_CalibrationParams_t params;
        memcpy(&params, g_usb_protocol.rx_payload, sizeof(params));

        SysParameter.u16AngAdc_MIN = params.angle_min_adc;
        SysParameter.u16AngAdc_MID = params.angle_mid_adc;
        SysParameter.u16AngAdc_MAX = params.angle_max_adc;
        SysParameter.u16AngPerUnitScope = params.angle_range;
        SysParameter.u16PrsAdc_MIN = params.pressure_min_adc;
        SysParameter.u16PrsAdc_MID = params.pressure_mid_adc;
        SysParameter.u16PrsAdc_MAX = params.pressure_max_adc;
        SysParameter.u16PrsAdc_MAX_Scope = params.pressure_max_bar;

        USB_Protocol_SendAck(USB_CMD_SET_CALIBRATION);
    } else {
        USB_Protocol_SendNack(USB_CMD_SET_CALIBRATION, USB_ERR_INVALID_PARAM);
    }
}

static void USB_Handle_GetFilterParams(void)
{
    USB_FilterParams_t params;

    params.angle_ref_filter = FilterTimConstant.g_f32_Ang_Ref_FilterTimPara;
    params.angle_fdb_filter = FilterTimConstant.g_f32_Ang_Fdb_FilterTimPara;
    params.pressure_fdb_filter = FilterTimConstant.g_f32_Prs_Fdb_FilterTimPara;
    params.current_fdb_filter = FilterTimConstant.g_f32_Pwr_Ref_FilterTimPara;

    USB_Protocol_SendResponse(USB_CMD_GET_FILTER_PARAMS,
                             (uint8_t*)&params, sizeof(params));
}

static void USB_Handle_SetFilterParams(void)
{
    if (g_usb_protocol.rx_length >= sizeof(USB_FilterParams_t)) {
        USB_FilterParams_t params;
        memcpy(&params, g_usb_protocol.rx_payload, sizeof(params));

        FilterTimConstant.g_f32_Ang_Ref_FilterTimPara = params.angle_ref_filter;
        FilterTimConstant.g_f32_Ang_Fdb_FilterTimPara = params.angle_fdb_filter;
        FilterTimConstant.g_f32_Prs_Fdb_FilterTimPara = params.pressure_fdb_filter;
        FilterTimConstant.g_f32_Pwr_Ref_FilterTimPara = params.current_fdb_filter;

        USB_Protocol_SendAck(USB_CMD_SET_FILTER_PARAMS);
    } else {
        USB_Protocol_SendNack(USB_CMD_SET_FILTER_PARAMS, USB_ERR_INVALID_PARAM);
    }
}

static void USB_Handle_StartStream(void)
{
    if (g_usb_protocol.rx_length >= sizeof(USB_StreamConfig_t)) {
        USB_StreamConfig_t config;
        memcpy(&config, g_usb_protocol.rx_payload, sizeof(config));

        g_usb_protocol.stream_enabled = config.enable;
        g_usb_protocol.stream_rate_ms = config.rate_ms;
        g_usb_protocol.stream_data_mask = config.data_mask;
        g_usb_protocol.stream_last_tick = USB_GetTimestamp();

        USB_Protocol_SendAck(USB_CMD_START_STREAM);
    } else {
        /* Use default streaming config */
        g_usb_protocol.stream_enabled = 1;
        g_usb_protocol.stream_last_tick = USB_GetTimestamp();
        USB_Protocol_SendAck(USB_CMD_START_STREAM);
    }
}

static void USB_Handle_StopStream(void)
{
    g_usb_protocol.stream_enabled = 0;
    USB_Protocol_SendAck(USB_CMD_STOP_STREAM);
}

/*============================================================================
 * FRAME PROCESSING
 *============================================================================*/

void USB_Protocol_ProcessFrame(void)
{
    switch (g_usb_protocol.rx_cmd) {
        /* System Commands */
        case USB_CMD_HEARTBEAT:
            USB_Protocol_SendAck(USB_CMD_HEARTBEAT);
            break;

        case USB_CMD_GET_VERSION:
        case USB_CMD_GET_STATUS:
            USB_Handle_GetVersion();
            break;

        /* Monitoring Commands */
        case USB_CMD_GET_ALL_FEEDBACK:
            USB_Handle_GetAllFeedback();
            break;

        case USB_CMD_GET_ANALOG_INPUT:
            USB_Handle_GetAnalogInput();
            break;

        /* Control Commands */
        case USB_CMD_SET_ANGLE_REF:
            USB_Handle_SetAngleRef();
            break;

        case USB_CMD_SET_PRESSURE_REF:
            USB_Handle_SetPressureRef();
            break;

        case USB_CMD_SET_CURRENT_REF:
            USB_Handle_SetCurrentRef();
            break;

        case USB_CMD_SET_ENABLE_FLAGS:
            USB_Handle_SetEnableFlags();
            break;

        /* PID Commands */
        case USB_CMD_GET_PID_ANGLE:
            USB_Handle_GetPIDAngle();
            break;

        case USB_CMD_SET_PID_ANGLE:
            USB_Handle_SetPIDAngle();
            break;

        case USB_CMD_GET_PID_PRESSURE:
            USB_Handle_GetPIDPressure();
            break;

        case USB_CMD_SET_PID_PRESSURE:
            USB_Handle_SetPIDPressure();
            break;

        /* Calibration Commands */
        case USB_CMD_GET_CALIBRATION:
            USB_Handle_GetCalibration();
            break;

        case USB_CMD_SET_CALIBRATION:
            USB_Handle_SetCalibration();
            break;

        case USB_CMD_GET_FILTER_PARAMS:
            USB_Handle_GetFilterParams();
            break;

        case USB_CMD_SET_FILTER_PARAMS:
            USB_Handle_SetFilterParams();
            break;

        /* Streaming Commands */
        case USB_CMD_START_STREAM:
            USB_Handle_StartStream();
            break;

        case USB_CMD_STOP_STREAM:
            USB_Handle_StopStream();
            break;

        default:
            USB_Protocol_SendNack(g_usb_protocol.rx_cmd, USB_ERR_INVALID_CMD);
            break;
    }
}

/*============================================================================
 * STREAMING
 *============================================================================*/

void USB_Protocol_SendAllFeedback(void)
{
    USB_Handle_GetAllFeedback();
}

void USB_Protocol_SendStreamData(void)
{
    /* Use the same format as GET_ALL_FEEDBACK but with STREAM_DATA command */
    USB_FeedbackData_t data;

    data.timestamp_ms = USB_GetTimestamp();
    data.angle_feedback = DetectorFdbVal.g_u32_AngFdb;
    data.angle_ref = InstructionSet.TiltAngRef;
    data.angle_diff = DetectorFdbVal.g_s32_AngFdb_D;
    data.pressure_feedback = DetectorFdbVal.g_u32_PressureFdb;
    data.pressure_ref = InstructionSet.g_u16_PerUnitPrsVal;
    data.pressure_diff = DetectorFdbVal.g_s32_PressureFdb_D;
    data.current_a_feedback = DetectorFdbVal.g_s32_CurFdb_A;
    data.current_a_ref = InstructionSet.Cur_A_Ref;
    data.current_b_feedback = DetectorFdbVal.g_s32_CurFdb_B;
    data.current_b_ref = InstructionSet.Cur_B_Ref;
    data.pwm_output_a = g_u32_PWMOutput_A;
    data.pwm_output_b = g_u32_PWMOutput_B;
    data.enable_flags = (PpqEnable.PumpStrt_Ena ? USB_ENABLE_PUMP_START : 0) |
                        (PpqEnable.AngLeak_Ena ? USB_ENABLE_ANGLE_LEAK : 0) |
                        (PpqEnable.PrsLoop_Ena ? USB_ENABLE_PRESSURE_LOOP : 0) |
                        (PpqEnable.PwrLoop_Ena ? USB_ENABLE_POWER_LOOP : 0);
    data.error_code = 0;
    data.reserved = 0;

    USB_Protocol_SendResponse(USB_CMD_STREAM_DATA,
                             (uint8_t*)&data, sizeof(data));
}

/*============================================================================
 * PERIODIC TASK
 *============================================================================*/

void USB_Protocol_Task(void)
{
    uint32_t current_tick = USB_GetTimestamp();

    /* Check connection timeout */
    if (g_usb_protocol.connected) {
        if ((current_tick - g_usb_protocol.last_rx_tick) > USB_CONNECTION_TIMEOUT) {
            g_usb_protocol.connected = 0;
            g_usb_protocol.stream_enabled = 0;
        }
    }

    /* Handle streaming */
    if (g_usb_protocol.stream_enabled && g_usb_protocol.connected) {
        if ((current_tick - g_usb_protocol.stream_last_tick) >=
            g_usb_protocol.stream_rate_ms) {
            g_usb_protocol.stream_last_tick = current_tick;
            USB_Protocol_SendStreamData();
        }
    }
}

uint8_t USB_Protocol_IsConnected(void)
{
    return g_usb_protocol.connected;
}

/*============================================================================
 * USART INTERRUPT HANDLER
 *============================================================================*/

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USB_USART, USART_IT_RXNE) != RESET) {
        uint8_t byte = (uint8_t)USART_ReceiveData(USB_USART);
        USB_Protocol_ProcessByte(byte);
        USART_ClearITPendingBit(USB_USART, USART_IT_RXNE);
    }
}
