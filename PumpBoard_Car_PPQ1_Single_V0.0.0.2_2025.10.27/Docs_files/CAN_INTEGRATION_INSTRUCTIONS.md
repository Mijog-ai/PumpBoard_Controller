# CAN Bus Integration Instructions

## Overview
This document describes how to integrate the CAN bus implementation into the existing PumpBoard project.

---

## Files Created
1. **HardWare_Init/Inc/Hw_CAN.h** - CAN hardware header
2. **HardWare_Init/Src/Hw_CAN.c** - CAN hardware implementation
3. **Function/can_protocol.h** - CAN protocol header
4. **Function/can_protocol.c** - CAN protocol implementation

---

## Files to Modify

### 1. USER/main.c

**Location:** After line 100 (after `AD7689InitFunc();`)

**Add these lines:**

```c
// Initialize CAN bus (add after AD7689InitFunc())
#ifdef ENABLE_CAN1
    Can1_Config();              // Initialize CAN1 with configured baud rate
    CAN_Protocol_Init();        // Initialize CAN protocol layer
#endif

#ifdef ENABLE_CAN2
    Can2_Config();              // Initialize CAN2 (optional)
#endif
```

**Add includes at top of file (after existing includes):**

```c
#include "Hw_CAN.h"
#include "can_protocol.h"
```

**Add configuration defines (before main function):**

```c
/* CAN Configuration */
#define ENABLE_CAN1             // Comment out to disable CAN1
// #define ENABLE_CAN2          // Uncomment to enable CAN2

/* CAN Baud Rate Configuration */
typedef struct {
    u16 baud_rate_canopen;      // CAN1 baud rate (125, 250, or 500 kbps)
    u16 baud_rate_j1939;        // CAN2 baud rate (for J1939 protocol)
} user_parameter_t;

user_parameter_t user_parameter = {
    .baud_rate_canopen = 250,   // Default 250 kbps
    .baud_rate_j1939 = 250      // Default 250 kbps
};
```

---

### 2. Function/stm32f4xx_it.c

**Location:** Replace existing empty CAN interrupt handlers (lines 201-208)

**Replace this code:**

```c
void CAN1_RX0_IRQHandler(void)//18fe3082
{
//    CAN_Receive_j1939(CAN1, 0, &canopen_msg);
}
void CAN2_RX0_IRQHandler(void)
{
//    CAN_Receive(CAN2, 0, &can_j1939_msg);
}
```

**With this code:**

```c
/**
  * @brief  CAN1 RX0 interrupt handler
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;
    u32 id;
    u8 data[8];
    u8 len;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        /* Receive message */
        CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

        /* Extract ID and data */
        if (RxMessage.IDE == CAN_Id_Standard)
        {
            id = RxMessage.StdId;
        }
        else
        {
            id = RxMessage.ExtId;
        }

        len = RxMessage.DLC;
        for (u8 i = 0; i < len && i < 8; i++)
        {
            data[i] = RxMessage.Data[i];
        }

        /* Process message */
        CAN_Process_Received_Msg(id, data, len);

        /* Clear interrupt flag */
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
    }
}

/**
  * @brief  CAN2 RX0 interrupt handler (optional)
  * @param  None
  * @retval None
  */
void CAN2_RX0_IRQHandler(void)
{
    CanRxMsg RxMessage;

    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);

        /* Add CAN2 message processing here if needed */

        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
    }
}
```

**Add includes at top of file:**

```c
#include "Hw_CAN.h"
#include "can_protocol.h"
```

---

### 3. HardWare_Init/Src/driver.c

**Modify Can1_Config() and Can2_Config() functions to use the new implementation:**

**Replace lines 20-47 (Can1_Config function):**

```c
void Can1_Config(void)
{
    // 500k =6  250k = 12  125k = 24
    if(user_parameter.baud_rate_canopen==125)
    {
        CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,24,CAN_Mode_Normal);
    }
    else if(user_parameter.baud_rate_canopen==250)
    {
        CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,12,CAN_Mode_Normal);
    }
    else if(user_parameter.baud_rate_canopen==500)
    {
        CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);
    }
    else  // Default 250k
    {
        CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,12,CAN_Mode_Normal);
    }
}
```

**Add include at top of driver.c:**

```c
#include "Hw_CAN.h"
```

**Note:** Keep the same code structure, it will now call the implemented CAN1_Mode_Init() function.

---

### 4. Function/Task.c

**Add CAN protocol task to the periodic task list**

**Find the Task.c file and locate the 1ms or 10ms periodic task function.**

**Add this code:**

```c
/* CAN Protocol Task - Call every 1ms */
void Task_CAN_Protocol(void)
{
    #ifdef ENABLE_CAN1
    CAN_Protocol_Task();
    #endif
}
```

**Add includes at top:**

```c
#include "can_protocol.h"
```

**Add the task call in the appropriate timer interrupt or task scheduler.**

---

## Hardware Configuration

### CAN1 Pins (STM32F407ZE):
- **CAN1_RX:** PA11 (already defined in gpio.h)
- **CAN1_TX:** PA12 (already defined in gpio.h)

### CAN2 Pins (Optional):
- **CAN2_RX:** PB12
- **CAN2_TX:** PB13

### External Hardware Required:
- **CAN Transceiver:** MCP2551, TJA1050, SN65HVD230, or similar
- **Termination Resistor:** 120Ω (if node is at bus end)

---

## CAN Protocol Messages

### Command Messages (Received):

| CAN ID | Name | Data Format | Description |
|--------|------|-------------|-------------|
| 0x200 | Angle Command | [MSB, LSB] | Angle reference (0-10000) |
| 0x201 | Pressure Command | [MSB, LSB] | Pressure reference * 10 |
| 0x202 | Current Command | [A_MSB, A_LSB, B_MSB, B_LSB] | Current A & B in mA |
| 0x203 | Enable Command | [Flags] | Bit 0: Pump, Bit 1: Pressure, Bit 2: Angle, Bit 3: Power |

### Feedback Messages (Transmitted):

| CAN ID | Name | Period | Data Format | Description |
|--------|------|--------|-------------|-------------|
| 0x100 | Heartbeat | 100ms | [Node_ID, Error] | System alive message |
| 0x300 | Status | 50ms | [Flags, Error, Ang_H, Ang_L, Prs_H, Prs_L, 0, 0] | Full status |
| 0x301 | Angle Feedback | On demand | [Fdb_H, Fdb_L, Ref_H, Ref_L] | Angle feedback & ref |
| 0x302 | Pressure Feedback | On demand | [Fdb_H, Fdb_L, Ref_H, Ref_L] | Pressure feedback & ref |
| 0x303 | Current Feedback | On demand | [A_Fdb_H, A_Fdb_L, A_Ref_H, A_Ref_L, B_Fdb_H, B_Fdb_L, B_Ref_H, B_Ref_L] | Current A & B |
| 0x3FF | Error | On event | [Node_ID, Error_Code] | Error notification |

---

## Baud Rate Configuration

**APB1 Clock = 42MHz (SYS_CLK / 4)**

| Baud Rate | BRP | BS1 | BS2 | SJW | Formula |
|-----------|-----|-----|-----|-----|---------|
| 125 kbps | 24 | 7tq | 6tq | 1tq | 42MHz / (24 * (1+7+6)) = 125k |
| 250 kbps | 12 | 7tq | 6tq | 1tq | 42MHz / (12 * (1+7+6)) = 250k |
| 500 kbps | 6 | 7tq | 6tq | 1tq | 42MHz / (6 * (1+7+6)) = 500k |
| 1 Mbps | 3 | 7tq | 6tq | 1tq | 42MHz / (3 * (1+7+6)) = 1000k |

---

## Testing Procedure

### 1. Basic Communication Test:
```c
// Send test message
u8 test_data[2] = {0x12, 0x34};
CAN_Transmit_Msg(CAN1, 0x123, test_data, 2, 0);
```

### 2. Enable CAN Feedback:
```c
// Enable in main loop or task
CAN_Send_Status();
CAN_Send_Feedback_Angle(DetectorFdbVal.g_u32_AngFdb);
```

### 3. Receive Commands:
Messages are automatically processed in CAN1_RX0_IRQHandler when received.

---

## Compilation

**Add new files to Keil project:**

1. Right-click on project → Add New Group → Name it "CAN"
2. Right-click "CAN" group → Add Existing Files
3. Add:
   - `HardWare_Init/Src/Hw_CAN.c`
   - `Function/can_protocol.c`

**Or manually edit PumpCtrl.uvprojx to include the new files.**

---

## Troubleshooting

### CAN Bus Not Working:
1. Check CAN transceiver power and connections
2. Verify 120Ω termination resistors at both bus ends
3. Use oscilloscope to check CAN_H and CAN_L signals
4. Verify baud rate matches other nodes on the bus

### No Messages Received:
1. Check CAN filter configuration in Hw_CAN.c
2. Verify interrupt is enabled and handler is called
3. Check CAN bus termination
4. Use CAN analyzer to verify bus traffic

### Messages Not Transmitted:
1. Check if CAN mailboxes are full
2. Verify CAN bus is not in bus-off state
3. Check return value of CAN_Transmit_Msg()
4. Verify other nodes are acknowledging messages

---

## Example Usage

```c
// In main.c initialization section:
Can1_Config();
CAN_Protocol_Init();

// In 1ms periodic task:
CAN_Protocol_Task();

// To send angle command from another node:
u8 cmd[2];
cmd[0] = 0x13;  // 5000 >> 8
cmd[1] = 0x88;  // 5000 & 0xFF
// This sets angle to 5000 (50%)
CAN_Transmit_Msg(CAN1, CAN_ID_CMD_ANGLE, cmd, 2, 0);
```

---

## Notes

- CAN protocol uses standard 11-bit IDs by default
- Extended 29-bit IDs can be used by setting id_type=1
- Maximum message length is 8 bytes (CAN 2.0B specification)
- CAN bus requires at least 2 nodes to function properly
- Always use proper CAN transceiver IC (3.3V logic compatible)
- CAN_H and CAN_L should be twisted pair cable for EMI immunity

