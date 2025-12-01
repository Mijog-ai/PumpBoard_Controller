/********************************************************************************
 * File name    : can_protocol.c
 * Description  : CAN protocol layer implementation
 * Author       : PumpBoard Team
 * Date         : 2025-12-01
 ********************************************************************************/

#include "can_protocol.h"
#include "Hw_CAN.h"
#include "application.h"

/* Global Variables */
u8 g_can_enabled = 0;
u16 g_can_heartbeat_counter = 0;
u16 g_can_feedback_counter = 0;
u8 g_can_error_code = CAN_ERR_NONE;

/*******************************************************************************
* Function Name : CAN_Protocol_Init
* Description   : Initialize CAN protocol layer
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Protocol_Init(void)
{
    g_can_enabled = 1;
    g_can_heartbeat_counter = 0;
    g_can_feedback_counter = 0;
    g_can_error_code = CAN_ERR_NONE;
}

/*******************************************************************************
* Function Name : CAN_Protocol_Task
* Description   : Periodic CAN protocol task (call every 1ms)
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Protocol_Task(void)
{
    if (!g_can_enabled)
        return;

    /* Heartbeat transmission */
    g_can_heartbeat_counter++;
    if (g_can_heartbeat_counter >= CAN_HEARTBEAT_PERIOD)
    {
        g_can_heartbeat_counter = 0;
        CAN_Send_Heartbeat();
    }

    /* Feedback transmission */
    g_can_feedback_counter++;
    if (g_can_feedback_counter >= CAN_FEEDBACK_PERIOD)
    {
        g_can_feedback_counter = 0;
        CAN_Send_Status();
    }
}

/*******************************************************************************
* Function Name : CAN_Send_Heartbeat
* Description   : Send heartbeat message
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Send_Heartbeat(void)
{
    u8 data[2];

    data[0] = CAN_NODE_ID;
    data[1] = g_can_error_code;

    CAN_Transmit_Msg(CAN1, CAN_ID_HEARTBEAT, data, 2, 0);
}

/*******************************************************************************
* Function Name : CAN_Send_Status
* Description   : Send system status message
* Input         : None
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Send_Status(void)
{
    u8 data[8];

    /* Byte 0: Enable flags */
    data[0] = 0;
    if (User_Parameter.PpqEna->PumpStrt_Ena) data[0] |= 0x01;
    if (User_Parameter.PpqEna->PrsLoop_Ena)  data[0] |= 0x02;
    if (User_Parameter.PpqEna->AngLeak_Ena)  data[0] |= 0x04;
    if (User_Parameter.PpqEna->PwrLoop_Ena)  data[0] |= 0x08;

    /* Byte 1: Error code */
    data[1] = g_can_error_code;

    /* Bytes 2-3: Angle feedback (0-10000) */
    data[2] = (DetectorFdbVal.g_u32_AngFdb >> 8) & 0xFF;
    data[3] = DetectorFdbVal.g_u32_AngFdb & 0xFF;

    /* Bytes 4-5: Pressure feedback (scaled) */
    u16 pressure_scaled = (u16)(DetectorFdbVal.g_f32_PressureFdb * 10.0f);
    data[4] = (pressure_scaled >> 8) & 0xFF;
    data[5] = pressure_scaled & 0xFF;

    /* Bytes 6-7: Reserved */
    data[6] = 0;
    data[7] = 0;

    CAN_Transmit_Msg(CAN1, CAN_ID_FDB_STATUS, data, 8, 0);
}

/*******************************************************************************
* Function Name : CAN_Send_Feedback_Angle
* Description   : Send angle feedback message
* Input         : angle_fdb - Angle feedback value (0-10000)
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Send_Feedback_Angle(u16 angle_fdb)
{
    u8 data[4];

    data[0] = (angle_fdb >> 8) & 0xFF;
    data[1] = angle_fdb & 0xFF;
    data[2] = (InstructionSet.TiltAngRef >> 8) & 0xFF;
    data[3] = InstructionSet.TiltAngRef & 0xFF;

    CAN_Transmit_Msg(CAN1, CAN_ID_FDB_ANGLE, data, 4, 0);
}

/*******************************************************************************
* Function Name : CAN_Send_Feedback_Pressure
* Description   : Send pressure feedback message
* Input         : pressure_fdb - Pressure feedback in bar
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Send_Feedback_Pressure(float pressure_fdb)
{
    u8 data[4];
    u16 pressure_scaled = (u16)(pressure_fdb * 10.0f);  /* 0.1 bar resolution */

    data[0] = (pressure_scaled >> 8) & 0xFF;
    data[1] = pressure_scaled & 0xFF;
    data[2] = (InstructionSet.g_u16_PerUnitPrsVal >> 8) & 0xFF;
    data[3] = InstructionSet.g_u16_PerUnitPrsVal & 0xFF;

    CAN_Transmit_Msg(CAN1, CAN_ID_FDB_PRESSURE, data, 4, 0);
}

/*******************************************************************************
* Function Name : CAN_Send_Feedback_Current
* Description   : Send current feedback message
* Input         : cur_a - Current A in mA
*                 cur_b - Current B in mA
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Send_Feedback_Current(s16 cur_a, s16 cur_b)
{
    u8 data[8];

    /* Current A feedback and reference */
    data[0] = (cur_a >> 8) & 0xFF;
    data[1] = cur_a & 0xFF;
    data[2] = (InstructionSet.Cur_A_Ref >> 8) & 0xFF;
    data[3] = InstructionSet.Cur_A_Ref & 0xFF;

    /* Current B feedback and reference */
    data[4] = (cur_b >> 8) & 0xFF;
    data[5] = cur_b & 0xFF;
    data[6] = (InstructionSet.Cur_B_Ref >> 8) & 0xFF;
    data[7] = InstructionSet.Cur_B_Ref & 0xFF;

    CAN_Transmit_Msg(CAN1, CAN_ID_FDB_CURRENT, data, 8, 0);
}

/*******************************************************************************
* Function Name : CAN_Send_Error
* Description   : Send error code message
* Input         : error_code - Error code bitmap
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Send_Error(u8 error_code)
{
    u8 data[2];

    g_can_error_code = error_code;

    data[0] = CAN_NODE_ID;
    data[1] = error_code;

    CAN_Transmit_Msg(CAN1, CAN_ID_FDB_ERROR, data, 2, 0);
}

/*******************************************************************************
* Function Name : CAN_Process_Received_Msg
* Description   : Process received CAN message
* Input         : id   - CAN message ID
*                 data - Pointer to received data
*                 len  - Data length
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Process_Received_Msg(u32 id, u8* data, u8 len)
{
    switch (id)
    {
        case CAN_ID_CMD_ANGLE:
            CAN_Handle_Angle_Command(data, len);
            break;

        case CAN_ID_CMD_PRESSURE:
            CAN_Handle_Pressure_Command(data, len);
            break;

        case CAN_ID_CMD_CURRENT:
            CAN_Handle_Current_Command(data, len);
            break;

        case CAN_ID_CMD_ENABLE:
            CAN_Handle_Enable_Command(data, len);
            break;

        default:
            /* Unknown message ID */
            break;
    }
}

/*******************************************************************************
* Function Name : CAN_Handle_Angle_Command
* Description   : Handle angle reference command
* Input         : data - Command data (2 bytes: angle reference 0-10000)
*                 len  - Data length
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Handle_Angle_Command(u8* data, u8 len)
{
    if (len >= 2)
    {
        u16 angle_ref = (data[0] << 8) | data[1];

        /* Limit to valid range */
        if (angle_ref <= 10000)
        {
            InstructionSet.TiltAngRef = angle_ref;
        }
    }
}

/*******************************************************************************
* Function Name : CAN_Handle_Pressure_Command
* Description   : Handle pressure reference command
* Input         : data - Command data (2 bytes: pressure * 10)
*                 len  - Data length
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Handle_Pressure_Command(u8* data, u8 len)
{
    if (len >= 2)
    {
        u16 pressure_ref = (data[0] << 8) | data[1];
        InstructionSet.g_u16_PerUnitPrsVal = pressure_ref;
    }
}

/*******************************************************************************
* Function Name : CAN_Handle_Current_Command
* Description   : Handle current reference command
* Input         : data - Command data (4 bytes: cur_a, cur_b in mA)
*                 len  - Data length
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Handle_Current_Command(u8* data, u8 len)
{
    if (len >= 4)
    {
        u16 cur_a_ref = (data[0] << 8) | data[1];
        u16 cur_b_ref = (data[2] << 8) | data[3];

        InstructionSet.Cur_A_Ref = cur_a_ref;
        InstructionSet.Cur_B_Ref = cur_b_ref;
    }
}

/*******************************************************************************
* Function Name : CAN_Handle_Enable_Command
* Description   : Handle enable/disable command
* Input         : data - Command data (1 byte: enable flags)
*                 len  - Data length
* Output        : None
* Return        : None
*******************************************************************************/
void CAN_Handle_Enable_Command(u8* data, u8 len)
{
    if (len >= 1)
    {
        u8 enable_flags = data[0];

        User_Parameter.PpqEna->PumpStrt_Ena = (enable_flags & 0x01) ? 1 : 0;
        User_Parameter.PpqEna->PrsLoop_Ena  = (enable_flags & 0x02) ? 1 : 0;
        User_Parameter.PpqEna->AngLeak_Ena  = (enable_flags & 0x04) ? 1 : 0;
        User_Parameter.PpqEna->PwrLoop_Ena  = (enable_flags & 0x08) ? 1 : 0;
    }
}
