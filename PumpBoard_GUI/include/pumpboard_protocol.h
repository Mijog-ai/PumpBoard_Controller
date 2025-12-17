/**
 * @file pumpboard_protocol.h
 * @brief PumpBoard Controller Communication Protocol
 *
 * This header defines the communication protocol between the STM32F407xx
 * PumpBoard Controller and the PC GUI application via USB CDC (Virtual COM Port).
 *
 * Frame Format:
 * [START_BYTE][CMD][LENGTH][PAYLOAD...][CRC16_L][CRC16_H][END_BYTE]
 *
 * @author PumpBoard GUI Project
 * @version 1.0.0
 */

#ifndef PUMPBOARD_PROTOCOL_H
#define PUMPBOARD_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * PROTOCOL CONSTANTS
 *============================================================================*/

#define PROTOCOL_START_BYTE     0xAA
#define PROTOCOL_END_BYTE       0x55
#define PROTOCOL_VERSION        0x01

#define MAX_PAYLOAD_SIZE        128
#define MAX_FRAME_SIZE          (MAX_PAYLOAD_SIZE + 6)  // Start + Cmd + Len + CRC16 + End

/*============================================================================
 * COMMAND DEFINITIONS
 *============================================================================*/

/* System Commands (0x00 - 0x0F) */
#define CMD_HEARTBEAT           0x01    // Heartbeat/keepalive
#define CMD_GET_VERSION         0x02    // Get firmware version
#define CMD_RESET               0x03    // Reset device
#define CMD_GET_STATUS          0x04    // Get system status

/* Monitoring Commands (0x10 - 0x2F) */
#define CMD_GET_ALL_FEEDBACK    0x10    // Get all sensor feedback values
#define CMD_GET_ANGLE_FDB       0x11    // Get angle feedback
#define CMD_GET_PRESSURE_FDB    0x12    // Get pressure feedback
#define CMD_GET_CURRENT_FDB     0x13    // Get current A/B feedback
#define CMD_GET_PWM_OUTPUT      0x14    // Get PWM output values
#define CMD_GET_ERROR_CODE      0x15    // Get error codes
#define CMD_GET_ANALOG_INPUT    0x16    // Get raw analog inputs
#define CMD_GET_PID_STATE       0x17    // Get PID internal states

/* Control Commands (0x30 - 0x4F) */
#define CMD_SET_ANGLE_REF       0x30    // Set angle reference (0-10000)
#define CMD_SET_PRESSURE_REF    0x31    // Set pressure reference (0-10000)
#define CMD_SET_CURRENT_REF     0x32    // Set current A/B references
#define CMD_SET_ENABLE_FLAGS    0x33    // Set enable/disable flags
#define CMD_SET_PWM_DIRECT      0x34    // Direct PWM control (test mode)

/* PID Tuning Commands (0x50 - 0x6F) */
#define CMD_GET_PID_ANGLE       0x50    // Get angle loop PID parameters
#define CMD_SET_PID_ANGLE       0x51    // Set angle loop PID parameters
#define CMD_GET_PID_PRESSURE    0x52    // Get pressure loop PID parameters
#define CMD_SET_PID_PRESSURE    0x53    // Set pressure loop PID parameters
#define CMD_GET_PID_CURRENT     0x54    // Get current loop PID parameters
#define CMD_SET_PID_CURRENT     0x55    // Set current loop PID parameters

/* Calibration Commands (0x70 - 0x8F) */
#define CMD_GET_CALIBRATION     0x70    // Get sensor calibration values
#define CMD_SET_CALIBRATION     0x71    // Set sensor calibration values
#define CMD_GET_FILTER_PARAMS   0x72    // Get filter parameters
#define CMD_SET_FILTER_PARAMS   0x73    // Set filter parameters

/* Data Streaming Commands (0x90 - 0x9F) */
#define CMD_START_STREAM        0x90    // Start continuous data streaming
#define CMD_STOP_STREAM         0x91    // Stop data streaming
#define CMD_SET_STREAM_RATE     0x92    // Set streaming rate (ms)
#define CMD_STREAM_DATA         0x93    // Streamed data packet

/* Parameter Save/Load (0xA0 - 0xAF) */
#define CMD_SAVE_PARAMS         0xA0    // Save parameters to flash
#define CMD_LOAD_PARAMS         0xA1    // Load parameters from flash
#define CMD_RESET_DEFAULTS      0xA2    // Reset to factory defaults

/* Response/Acknowledgment */
#define CMD_ACK                 0xF0    // Acknowledgment
#define CMD_NACK                0xF1    // Negative acknowledgment
#define CMD_ERROR               0xFF    // Error response

/*============================================================================
 * ENABLE FLAGS BITMASK
 *============================================================================*/

#define ENABLE_PUMP_START       (1 << 0)    // Bit 0: Pump start
#define ENABLE_ANGLE_LEAK       (1 << 1)    // Bit 1: Angle leakage compensation
#define ENABLE_PRESSURE_LOOP    (1 << 2)    // Bit 2: Pressure loop
#define ENABLE_POWER_LOOP       (1 << 3)    // Bit 3: Power loop
#define ENABLE_PARA_SAVE        (1 << 4)    // Bit 4: Parameter save enabled

/*============================================================================
 * ERROR CODES
 *============================================================================*/

#define ERR_NONE                0x00
#define ERR_OVERCURRENT         0x01
#define ERR_OVERPRESSURE        0x02
#define ERR_SENSOR_FAULT        0x04
#define ERR_COMM_TIMEOUT        0x08
#define ERR_INVALID_CMD         0x10
#define ERR_INVALID_PARAM       0x20
#define ERR_CRC_MISMATCH        0x40
#define ERR_FRAME_ERROR         0x80

/*============================================================================
 * DATA STRUCTURES
 *============================================================================*/

#pragma pack(push, 1)

/* Protocol Frame Header */
typedef struct {
    uint8_t start_byte;     // PROTOCOL_START_BYTE
    uint8_t command;        // Command ID
    uint8_t length;         // Payload length
} ProtocolHeader_t;

/* Protocol Frame Footer */
typedef struct {
    uint16_t crc16;         // CRC16-CCITT
    uint8_t end_byte;       // PROTOCOL_END_BYTE
} ProtocolFooter_t;

/* System Status */
typedef struct {
    uint8_t version_major;
    uint8_t version_minor;
    uint8_t version_patch;
    uint8_t status_flags;       // Bit flags for system status
    uint8_t error_code;         // Current error code
    uint8_t enable_flags;       // Current enable states
    uint8_t simulation_mode;    // 1 = simulation, 0 = normal
    uint8_t reserved;
} SystemStatus_t;

/* All Feedback Values - Sent with CMD_GET_ALL_FEEDBACK or CMD_STREAM_DATA */
typedef struct {
    uint32_t timestamp_ms;      // System timestamp in ms
    int32_t  angle_feedback;    // Angle feedback (0-10000)
    int32_t  angle_ref;         // Current angle reference
    int32_t  angle_diff;        // Angle differential
    int32_t  pressure_feedback; // Pressure feedback (0-10000, or bar*100)
    int32_t  pressure_ref;      // Current pressure reference
    int32_t  pressure_diff;     // Pressure differential
    int32_t  current_a_feedback;// Current A feedback (mA)
    int32_t  current_a_ref;     // Current A reference
    int32_t  current_b_feedback;// Current B feedback (mA)
    int32_t  current_b_ref;     // Current B reference
    uint32_t pwm_output_a;      // PWM duty cycle A (0-105000)
    uint32_t pwm_output_b;      // PWM duty cycle B (0-105000)
    uint8_t  enable_flags;      // Current enable states
    uint8_t  error_code;        // Error status
    uint16_t reserved;          // Alignment padding
} FeedbackData_t;

/* PID Parameters */
typedef struct {
    int32_t kp;             // Proportional gain
    int32_t kp_div;         // Proportional divisor
    int32_t ki;             // Integral gain
    int32_t ki_div;         // Integral divisor
    int32_t kd;             // Derivative gain
    int32_t kd_div;         // Derivative divisor
    int32_t kv;             // Feed-forward gain (optional)
    int32_t kv_div;         // Feed-forward divisor
    int32_t output_max;     // Maximum output
    int32_t output_min;     // Minimum output
} PIDParams_t;

/* PID State (for monitoring) */
typedef struct {
    int32_t error;          // Current error
    int32_t sum_error;      // Integral sum
    int32_t diff_error;     // Derivative term
    int32_t output;         // Current output
    int32_t p_term;         // P contribution
    int32_t i_term;         // I contribution
    int32_t d_term;         // D contribution
} PIDState_t;

/* Calibration Parameters */
typedef struct {
    uint16_t angle_min_adc;     // Minimum angle ADC value
    uint16_t angle_mid_adc;     // Middle angle ADC value
    uint16_t angle_max_adc;     // Maximum angle ADC value
    uint16_t angle_range;       // Angle per-unit range (default 5000)
    uint16_t pressure_min_adc;  // Minimum pressure ADC value
    uint16_t pressure_mid_adc;  // Middle pressure ADC value
    uint16_t pressure_max_adc;  // Maximum pressure ADC value
    uint16_t pressure_max_bar;  // Maximum pressure in bar (default 600)
} CalibrationParams_t;

/* Filter Parameters */
typedef struct {
    float angle_ref_filter;     // Angle reference filter time constant
    float angle_fdb_filter;     // Angle feedback filter time constant
    float pressure_fdb_filter;  // Pressure feedback filter time constant
    float current_fdb_filter;   // Current feedback filter time constant
} FilterParams_t;

/* Control Command - Set References */
typedef struct {
    int32_t angle_ref;      // Angle reference (0-10000)
    int32_t pressure_ref;   // Pressure reference (0-10000)
    int32_t current_a_ref;  // Current A reference (mA)
    int32_t current_b_ref;  // Current B reference (mA)
} ControlCommand_t;

/* Stream Configuration */
typedef struct {
    uint8_t enable;         // 1 = enable streaming
    uint8_t rate_ms;        // Streaming rate in ms (10-1000)
    uint8_t data_mask;      // Bitmask of data to include
    uint8_t reserved;
} StreamConfig_t;

/* Data mask bits for streaming */
#define STREAM_ANGLE        (1 << 0)
#define STREAM_PRESSURE     (1 << 1)
#define STREAM_CURRENT      (1 << 2)
#define STREAM_PWM          (1 << 3)
#define STREAM_PID_STATE    (1 << 4)
#define STREAM_ANALOG       (1 << 5)
#define STREAM_ALL          0xFF

/* Raw Analog Input Values */
typedef struct {
    uint16_t sensor_overcurrent;
    uint16_t analog_out1_check;
    uint16_t analog_out2_check;
    uint16_t current_a_adc;
    uint16_t current_b_adc;
    uint16_t angle_adc;         // From AD7689
    uint16_t pressure_adc;      // From AD7689
    uint16_t reserved;
} AnalogInputs_t;

#pragma pack(pop)

/*============================================================================
 * CRC16 CALCULATION (CRC16-CCITT)
 *============================================================================*/

/**
 * @brief Calculate CRC16-CCITT
 * @param data Pointer to data buffer
 * @param length Length of data
 * @return CRC16 value
 */
static inline uint16_t crc16_ccitt(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    while (length--) {
        crc ^= (*data++) << 8;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

/*============================================================================
 * HELPER MACROS
 *============================================================================*/

/* Convert per-unit (0-10000) to percentage (0-100.0) */
#define PERUNIT_TO_PERCENT(x)   ((float)(x) / 100.0f)

/* Convert percentage (0-100.0) to per-unit (0-10000) */
#define PERCENT_TO_PERUNIT(x)   ((int32_t)((x) * 100.0f))

/* Convert PWM value to percentage (based on ARR=105000) */
#define PWM_TO_PERCENT(x)       ((float)(x) / 105000.0f * 100.0f)

/* Convert pressure per-unit to bar (assuming max 600 bar) */
#define PERUNIT_TO_BAR(x)       ((float)(x) / 10000.0f * 600.0f)

#ifdef __cplusplus
}
#endif

#endif /* PUMPBOARD_PROTOCOL_H */
