/**
 * @file usb_protocol.h
 * @brief USB/UART Communication Protocol Handler for PumpBoard Controller
 *
 * Implements the communication protocol for PC GUI connectivity via
 * USB Type-B (Virtual COM Port / USART).
 *
 * @author PumpBoard Project
 * @version 1.0.0
 */

#ifndef USB_PROTOCOL_H
#define USB_PROTOCOL_H

#include "stm32f4xx.h"
#include "application.h"

/*============================================================================
 * PROTOCOL CONSTANTS
 *============================================================================*/

#define USB_PROTOCOL_START_BYTE     0xAA
#define USB_PROTOCOL_END_BYTE       0x55
#define USB_PROTOCOL_VERSION        0x01

#define USB_MAX_PAYLOAD_SIZE        128
#define USB_MAX_FRAME_SIZE          (USB_MAX_PAYLOAD_SIZE + 6)

#define USB_RX_BUFFER_SIZE          256
#define USB_TX_BUFFER_SIZE          256

/*============================================================================
 * COMMAND DEFINITIONS
 *============================================================================*/

/* System Commands (0x00 - 0x0F) */
#define USB_CMD_HEARTBEAT           0x01
#define USB_CMD_GET_VERSION         0x02
#define USB_CMD_RESET               0x03
#define USB_CMD_GET_STATUS          0x04

/* Monitoring Commands (0x10 - 0x2F) */
#define USB_CMD_GET_ALL_FEEDBACK    0x10
#define USB_CMD_GET_ANGLE_FDB       0x11
#define USB_CMD_GET_PRESSURE_FDB    0x12
#define USB_CMD_GET_CURRENT_FDB     0x13
#define USB_CMD_GET_PWM_OUTPUT      0x14
#define USB_CMD_GET_ERROR_CODE      0x15
#define USB_CMD_GET_ANALOG_INPUT    0x16
#define USB_CMD_GET_PID_STATE       0x17

/* Control Commands (0x30 - 0x4F) */
#define USB_CMD_SET_ANGLE_REF       0x30
#define USB_CMD_SET_PRESSURE_REF    0x31
#define USB_CMD_SET_CURRENT_REF     0x32
#define USB_CMD_SET_ENABLE_FLAGS    0x33
#define USB_CMD_SET_PWM_DIRECT      0x34

/* PID Tuning Commands (0x50 - 0x6F) */
#define USB_CMD_GET_PID_ANGLE       0x50
#define USB_CMD_SET_PID_ANGLE       0x51
#define USB_CMD_GET_PID_PRESSURE    0x52
#define USB_CMD_SET_PID_PRESSURE    0x53
#define USB_CMD_GET_PID_CURRENT     0x54
#define USB_CMD_SET_PID_CURRENT     0x55

/* Calibration Commands (0x70 - 0x8F) */
#define USB_CMD_GET_CALIBRATION     0x70
#define USB_CMD_SET_CALIBRATION     0x71
#define USB_CMD_GET_FILTER_PARAMS   0x72
#define USB_CMD_SET_FILTER_PARAMS   0x73

/* Data Streaming Commands (0x90 - 0x9F) */
#define USB_CMD_START_STREAM        0x90
#define USB_CMD_STOP_STREAM         0x91
#define USB_CMD_SET_STREAM_RATE     0x92
#define USB_CMD_STREAM_DATA         0x93

/* Parameter Save/Load (0xA0 - 0xAF) */
#define USB_CMD_SAVE_PARAMS         0xA0
#define USB_CMD_LOAD_PARAMS         0xA1
#define USB_CMD_RESET_DEFAULTS      0xA2

/* Response/Acknowledgment */
#define USB_CMD_ACK                 0xF0
#define USB_CMD_NACK                0xF1
#define USB_CMD_ERROR               0xFF

/*============================================================================
 * ENABLE FLAGS BITMASK
 *============================================================================*/

#define USB_ENABLE_PUMP_START       (1 << 0)
#define USB_ENABLE_ANGLE_LEAK       (1 << 1)
#define USB_ENABLE_PRESSURE_LOOP    (1 << 2)
#define USB_ENABLE_POWER_LOOP       (1 << 3)
#define USB_ENABLE_PARA_SAVE        (1 << 4)

/*============================================================================
 * ERROR CODES
 *============================================================================*/

#define USB_ERR_NONE                0x00
#define USB_ERR_OVERCURRENT         0x01
#define USB_ERR_OVERPRESSURE        0x02
#define USB_ERR_SENSOR_FAULT        0x04
#define USB_ERR_COMM_TIMEOUT        0x08
#define USB_ERR_INVALID_CMD         0x10
#define USB_ERR_INVALID_PARAM       0x20
#define USB_ERR_CRC_MISMATCH        0x40
#define USB_ERR_FRAME_ERROR         0x80

/*============================================================================
 * DATA STRUCTURES
 *============================================================================*/

#pragma pack(push, 1)

/* System Status Response */
typedef struct {
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t version_patch;
    uint8_t status_flags;
    uint8_t error_code;
    uint8_t enable_flags;
    uint8_t simulation_mode;
    uint8_t reserved;
} USB_SystemStatus_t;

/* All Feedback Values */
typedef struct {
    uint32_t timestamp_ms;
    int32_t  angle_feedback;
    int32_t  angle_ref;
    int32_t  angle_diff;
    int32_t  pressure_feedback;
    int32_t  pressure_ref;
    int32_t  pressure_diff;
    int32_t  current_a_feedback;
    int32_t  current_a_ref;
    int32_t  current_b_feedback;
    int32_t  current_b_ref;
    uint32_t pwm_output_a;
    uint32_t pwm_output_b;
    uint8_t  enable_flags;
    uint8_t  error_code;
    uint16_t reserved;
} USB_FeedbackData_t;

/* PID Parameters */
typedef struct {
    int32_t kp;
    int32_t kp_div;
    int32_t ki;
    int32_t ki_div;
    int32_t kd;
    int32_t kd_div;
    int32_t kv;
    int32_t kv_div;
    int32_t output_max;
    int32_t output_min;
} USB_PIDParams_t;

/* Calibration Parameters */
typedef struct {
    uint16_t angle_min_adc;
    uint16_t angle_mid_adc;
    uint16_t angle_max_adc;
    uint16_t angle_range;
    uint16_t pressure_min_adc;
    uint16_t pressure_mid_adc;
    uint16_t pressure_max_adc;
    uint16_t pressure_max_bar;
} USB_CalibrationParams_t;

/* Filter Parameters */
typedef struct {
    float angle_ref_filter;
    float angle_fdb_filter;
    float pressure_fdb_filter;
    float current_fdb_filter;
} USB_FilterParams_t;

/* Control Command */
typedef struct {
    int32_t angle_ref;
    int32_t pressure_ref;
    int32_t current_a_ref;
    int32_t current_b_ref;
} USB_ControlCommand_t;

/* Stream Configuration */
typedef struct {
    uint8_t enable;
    uint8_t rate_ms;
    uint8_t data_mask;
    uint8_t reserved;
} USB_StreamConfig_t;

/* Analog Input Values */
typedef struct {
    uint16_t sensor_overcurrent;
    uint16_t analog_out1_check;
    uint16_t analog_out2_check;
    uint16_t current_a_adc;
    uint16_t current_b_adc;
    uint16_t angle_adc;
    uint16_t pressure_adc;
    uint16_t reserved;
} USB_AnalogInputs_t;

#pragma pack(pop)

/* Stream data mask bits */
#define USB_STREAM_ANGLE        (1 << 0)
#define USB_STREAM_PRESSURE     (1 << 1)
#define USB_STREAM_CURRENT      (1 << 2)
#define USB_STREAM_PWM          (1 << 3)
#define USB_STREAM_PID_STATE    (1 << 4)
#define USB_STREAM_ANALOG       (1 << 5)
#define USB_STREAM_ALL          0xFF

/*============================================================================
 * RECEIVE STATE MACHINE
 *============================================================================*/

typedef enum {
    USB_RX_WAIT_START,
    USB_RX_GET_CMD,
    USB_RX_GET_LENGTH,
    USB_RX_GET_PAYLOAD,
    USB_RX_GET_CRC_L,
    USB_RX_GET_CRC_H,
    USB_RX_GET_END
} USB_RxState_t;

/*============================================================================
 * PROTOCOL HANDLER STRUCTURE
 *============================================================================*/

typedef struct {
    /* RX State Machine */
    USB_RxState_t rx_state;
    uint8_t rx_cmd;
    uint8_t rx_length;
    uint8_t rx_payload[USB_MAX_PAYLOAD_SIZE];
    uint8_t rx_payload_idx;
    uint16_t rx_crc;

    /* TX Buffer */
    uint8_t tx_buffer[USB_TX_BUFFER_SIZE];
    uint16_t tx_length;

    /* Streaming */
    uint8_t stream_enabled;
    uint8_t stream_rate_ms;
    uint8_t stream_data_mask;
    uint32_t stream_last_tick;

    /* Statistics */
    uint32_t rx_frame_count;
    uint32_t tx_frame_count;
    uint32_t rx_error_count;
    uint32_t crc_error_count;

    /* Timeout */
    uint32_t last_rx_tick;
    uint8_t connected;
} USB_ProtocolHandler_t;

/*============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/

/**
 * @brief Initialize USB protocol handler
 */
void USB_Protocol_Init(void);

/**
 * @brief Initialize USART for USB communication
 * @param baudrate Communication baud rate (default: 115200)
 */
void USB_UART_Init(uint32_t baudrate);

/**
 * @brief Process received byte (call from USART RX interrupt)
 * @param byte Received byte
 */
void USB_Protocol_ProcessByte(uint8_t byte);

/**
 * @brief Process complete frame (called when frame is ready)
 */
void USB_Protocol_ProcessFrame(void);

/**
 * @brief Periodic task for streaming and timeout detection
 * @note Call this from the 10ms or 100ms task
 */
void USB_Protocol_Task(void);

/**
 * @brief Send response frame
 * @param cmd Command ID
 * @param payload Payload data
 * @param length Payload length
 */
void USB_Protocol_SendResponse(uint8_t cmd, const uint8_t* payload, uint8_t length);

/**
 * @brief Send ACK response
 * @param cmd Original command being acknowledged
 */
void USB_Protocol_SendAck(uint8_t cmd);

/**
 * @brief Send NACK response
 * @param cmd Original command
 * @param error_code Error code
 */
void USB_Protocol_SendNack(uint8_t cmd, uint8_t error_code);

/**
 * @brief Send all feedback data
 */
void USB_Protocol_SendAllFeedback(void);

/**
 * @brief Send stream data packet
 */
void USB_Protocol_SendStreamData(void);

/**
 * @brief Check if GUI is connected
 * @return 1 if connected, 0 if timeout
 */
uint8_t USB_Protocol_IsConnected(void);

/**
 * @brief Calculate CRC16-CCITT
 * @param data Data buffer
 * @param length Data length
 * @return CRC16 value
 */
uint16_t USB_CRC16_CCITT(const uint8_t* data, uint16_t length);

/**
 * @brief Get current system timestamp (milliseconds)
 * @return Timestamp in ms
 */
uint32_t USB_GetTimestamp(void);

/*============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

extern USB_ProtocolHandler_t g_usb_protocol;

#endif /* USB_PROTOCOL_H */
