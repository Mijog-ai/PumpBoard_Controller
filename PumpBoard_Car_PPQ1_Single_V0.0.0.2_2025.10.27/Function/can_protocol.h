/********************************************************************************
 * File name    : can_protocol.h
 * Description  : CAN protocol layer for PumpBoard controller
 * Author       : PumpBoard Team
 * Date         : 2025-12-01
 * Protocol     : Custom protocol for pump control
 ********************************************************************************/

#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include "stm32f4xx.h"

/* CAN Message IDs (Standard 11-bit IDs) */
#define CAN_ID_HEARTBEAT        0x100   /* Heartbeat message (periodic) */
#define CAN_ID_CMD_ANGLE        0x200   /* Angle command */
#define CAN_ID_CMD_PRESSURE     0x201   /* Pressure command */
#define CAN_ID_CMD_CURRENT      0x202   /* Current command */
#define CAN_ID_CMD_ENABLE       0x203   /* Enable/disable control loops */
#define CAN_ID_FDB_STATUS       0x300   /* Status feedback */
#define CAN_ID_FDB_ANGLE        0x301   /* Angle feedback */
#define CAN_ID_FDB_PRESSURE     0x302   /* Pressure feedback */
#define CAN_ID_FDB_CURRENT      0x303   /* Current feedback */
#define CAN_ID_FDB_ERROR        0x3FF   /* Error codes */

/* CAN Protocol Configuration */
#define CAN_NODE_ID             0x01    /* This node's ID (configurable) */
#define CAN_HEARTBEAT_PERIOD    100     /* Heartbeat period in ms */
#define CAN_FEEDBACK_PERIOD     50      /* Feedback period in ms */

/* Error Codes */
#define CAN_ERR_NONE            0x00
#define CAN_ERR_OVERCURRENT     0x01
#define CAN_ERR_OVERPRESSURE    0x02
#define CAN_ERR_SENSOR_FAULT    0x04
#define CAN_ERR_COMM_TIMEOUT    0x08
#define CAN_ERR_INVALID_CMD     0x10

/* CAN Message Structures */
typedef struct
{
    u32 id;
    u8 data[8];
    u8 len;
} CAN_Message_t;

/* Function Prototypes */
void CAN_Protocol_Init(void);
void CAN_Protocol_Task(void);
void CAN_Send_Heartbeat(void);
void CAN_Send_Status(void);
void CAN_Send_Feedback_Angle(u16 angle_fdb);
void CAN_Send_Feedback_Pressure(float pressure_fdb);
void CAN_Send_Feedback_Current(s16 cur_a, s16 cur_b);
void CAN_Send_Error(u8 error_code);
void CAN_Process_Received_Msg(u32 id, u8* data, u8 len);
void CAN_Handle_Angle_Command(u8* data, u8 len);
void CAN_Handle_Pressure_Command(u8* data, u8 len);
void CAN_Handle_Current_Command(u8* data, u8 len);
void CAN_Handle_Enable_Command(u8* data, u8 len);

/* Global Variables */
extern u8 g_can_enabled;
extern u16 g_can_heartbeat_counter;
extern u16 g_can_feedback_counter;
extern u8 g_can_error_code;

#endif /* __CAN_PROTOCOL_H */
