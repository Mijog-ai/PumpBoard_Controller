# CAN Protocol Quick Reference Guide

## Overview
This document provides a quick reference for the PumpBoard CAN bus protocol implementation.

---

## Hardware Configuration

### CAN1 (Primary Interface)
- **RX Pin:** PA11
- **TX Pin:** PA12
- **Baud Rate:** 125/250/500 kbps (configurable)
- **Protocol:** Custom PumpBoard control protocol

### CAN2 (Optional - J1939)
- **RX Pin:** PB12
- **TX Pin:** PB13
- **Baud Rate:** 125/250/500 kbps (configurable)
- **Protocol:** J1939 (implementation to be added)

---

## CAN Message Protocol

### Message ID Ranges
- **0x100-0x1FF:** System messages (heartbeat, etc.)
- **0x200-0x2FF:** Command messages (incoming)
- **0x300-0x3FF:** Feedback messages (outgoing)

---

## Command Messages (Received)

### 0x200 - Angle Command
**Description:** Set swivel angle reference
**Direction:** Master → PumpBoard
**Data Length:** 2 bytes
**Format:**
```
Byte 0: Angle reference MSB
Byte 1: Angle reference LSB
```
**Value Range:** 0-10000 (0% to 100%)
**Example:**
```c
// Set angle to 50% (5000)
u8 cmd[2] = {0x13, 0x88};  // 5000 = 0x1388
CAN_Transmit_Msg(CAN1, 0x200, cmd, 2, 0);
```

---

### 0x201 - Pressure Command
**Description:** Set pressure reference
**Direction:** Master → PumpBoard
**Data Length:** 2 bytes
**Format:**
```
Byte 0: Pressure reference MSB (pressure * 10)
Byte 1: Pressure reference LSB
```
**Value Range:** 0-5000 (0-500.0 bar, 0.1 bar resolution)
**Example:**
```c
// Set pressure to 150.5 bar
u16 pressure = 1505;  // 150.5 * 10
u8 cmd[2] = {(pressure >> 8) & 0xFF, pressure & 0xFF};
CAN_Transmit_Msg(CAN1, 0x201, cmd, 2, 0);
```

---

### 0x202 - Current Command
**Description:** Set solenoid current references
**Direction:** Master → PumpBoard
**Data Length:** 4 bytes
**Format:**
```
Byte 0: Current A reference MSB
Byte 1: Current A reference LSB
Byte 2: Current B reference MSB
Byte 3: Current B reference LSB
```
**Value Range:** 0-3000 mA per channel
**Example:**
```c
// Set current A to 1500mA, current B to 1200mA
u8 cmd[4] = {0x05, 0xDC, 0x04, 0xB0};
CAN_Transmit_Msg(CAN1, 0x202, cmd, 4, 0);
```

---

### 0x203 - Enable Command
**Description:** Enable/disable control loops
**Direction:** Master → PumpBoard
**Data Length:** 1 byte
**Format:**
```
Byte 0: Enable flags
  Bit 0: Pump start enable
  Bit 1: Pressure loop enable
  Bit 2: Angle leak enable
  Bit 3: Power loop enable
  Bits 4-7: Reserved (set to 0)
```
**Example:**
```c
// Enable all loops
u8 cmd[1] = {0x0F};  // Binary: 00001111
CAN_Transmit_Msg(CAN1, 0x203, cmd, 1, 0);

// Enable only pump and angle loop
u8 cmd[1] = {0x05};  // Binary: 00000101
CAN_Transmit_Msg(CAN1, 0x203, cmd, 1, 0);
```

---

## Feedback Messages (Transmitted)

### 0x100 - Heartbeat
**Description:** System alive message
**Direction:** PumpBoard → Master
**Period:** 100ms
**Data Length:** 2 bytes
**Format:**
```
Byte 0: Node ID (0x01)
Byte 1: Error code (bitmap)
```
**Error Code Bits:**
- Bit 0: Overcurrent fault
- Bit 1: Overpressure fault
- Bit 2: Sensor fault
- Bit 3: Communication timeout
- Bit 4: Invalid command

---

### 0x300 - Status Feedback
**Description:** Complete system status
**Direction:** PumpBoard → Master
**Period:** 50ms
**Data Length:** 8 bytes
**Format:**
```
Byte 0: Enable flags
  Bit 0: Pump started
  Bit 1: Pressure loop enabled
  Bit 2: Angle leak enabled
  Bit 3: Power loop enabled
Byte 1: Error code
Byte 2: Angle feedback MSB
Byte 3: Angle feedback LSB
Byte 4: Pressure feedback MSB (pressure * 10)
Byte 5: Pressure feedback LSB
Byte 6: Reserved
Byte 7: Reserved
```

---

### 0x301 - Angle Feedback
**Description:** Detailed angle feedback
**Direction:** PumpBoard → Master
**Period:** On demand
**Data Length:** 4 bytes
**Format:**
```
Byte 0: Angle feedback MSB
Byte 1: Angle feedback LSB
Byte 2: Angle reference MSB
Byte 3: Angle reference LSB
```

---

### 0x302 - Pressure Feedback
**Description:** Detailed pressure feedback
**Direction:** PumpBoard → Master
**Period:** On demand
**Data Length:** 4 bytes
**Format:**
```
Byte 0: Pressure feedback MSB (pressure * 10)
Byte 1: Pressure feedback LSB
Byte 2: Pressure reference MSB
Byte 3: Pressure reference LSB
```

---

### 0x303 - Current Feedback
**Description:** Solenoid current feedback
**Direction:** PumpBoard → Master
**Period:** On demand
**Data Length:** 8 bytes
**Format:**
```
Byte 0: Current A feedback MSB
Byte 1: Current A feedback LSB
Byte 2: Current A reference MSB
Byte 3: Current A reference LSB
Byte 4: Current B feedback MSB
Byte 5: Current B feedback LSB
Byte 6: Current B reference MSB
Byte 7: Current B reference LSB
```

---

### 0x3FF - Error Message
**Description:** Error event notification
**Direction:** PumpBoard → Master
**Period:** On event
**Data Length:** 2 bytes
**Format:**
```
Byte 0: Node ID
Byte 1: Error code (same as heartbeat)
```

---

## Error Codes

| Code | Bit | Description | Action |
|------|-----|-------------|--------|
| 0x01 | 0 | Overcurrent detected | Reduce current reference |
| 0x02 | 1 | Overpressure detected | Reduce pressure reference |
| 0x04 | 2 | Sensor fault | Check sensor connections |
| 0x08 | 3 | Communication timeout | Check CAN bus |
| 0x10 | 4 | Invalid command | Verify command format |

---

## Example Communication Sequences

### Startup Sequence
```
1. Master → 0x203: Enable all loops [0x0F]
2. PumpBoard → 0x100: Heartbeat [0x01, 0x00]  (alive, no errors)
3. Master → 0x200: Set angle 30% [0x0B, 0xB8]  (3000 = 0x0BB8)
4. Master → 0x201: Set pressure 100 bar [0x03, 0xE8]  (1000 = 0x03E8)
5. PumpBoard → 0x300: Status feedback [0x0F, 0x00, ...]
```

### Normal Operation
```
Every 50ms:  PumpBoard → 0x300: Status feedback
Every 100ms: PumpBoard → 0x100: Heartbeat
On command:  Master → 0x200/0x201/0x202: Update references
```

### Fault Handling
```
1. Overcurrent detected
2. PumpBoard → 0x3FF: Error [0x01, 0x01]  (Overcurrent error)
3. PumpBoard → 0x100: Heartbeat [0x01, 0x01]  (Error in heartbeat too)
4. Master → 0x202: Reduce current [lower values]
5. Error clears when current is within limits
6. PumpBoard → 0x100: Heartbeat [0x01, 0x00]  (Error cleared)
```

---

## Timing Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| Heartbeat period | 100ms | Fixed, cannot be changed |
| Status feedback period | 50ms | Fixed, cannot be changed |
| Command processing | <1ms | Immediate via interrupt |
| CAN bus latency | <5ms | Typical at 250kbps |
| Timeout detection | 500ms | If no commands received |

---

## Testing with PCAN-View or CANoe

### Send Angle Command (50%):
```
ID: 0x200
DLC: 2
Data: 13 88
```

### Send Enable All Loops:
```
ID: 0x203
DLC: 1
Data: 0F
```

### Expected Response (Status):
```
ID: 0x300
DLC: 8
Data: 0F 00 13 88 00 00 00 00
      ↑  ↑  ↑---↑  ↑---↑
      │  │    │     └─ Pressure feedback
      │  │    └─ Angle feedback (5000)
      │  └─ No errors
      └─ All loops enabled
```

---

## Python Example (using python-can)

```python
import can

# Initialize CAN interface
bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=250000)

# Send angle command (50%)
msg = can.Message(
    arbitration_id=0x200,
    data=[0x13, 0x88],  # 5000 = 50%
    is_extended_id=False
)
bus.send(msg)

# Receive messages
while True:
    msg = bus.recv(timeout=1.0)
    if msg:
        print(f"ID: 0x{msg.arbitration_id:03X}, Data: {msg.data.hex()}")
```

---

## Troubleshooting

### No CAN Messages Received:
1. Check termination resistors (120Ω at both ends)
2. Verify baud rate matches (default 250kbps)
3. Check CAN_H and CAN_L are not swapped
4. Verify transceiver power (usually 5V)

### Messages Sent But Not Acknowledged:
1. At least 2 nodes required on bus
2. Check that both nodes have correct termination
3. Use CAN analyzer to verify ACK bit

### Invalid Data Received:
1. Check byte order (MSB first)
2. Verify data length (DLC)
3. Check for bit errors on bus (use CAN analyzer)

---

## CAN Transceiver Connections

### MCP2551 / TJA1050 Typical Circuit:
```
STM32F407         MCP2551         CAN Bus
---------         -------         -------
PA12 (TX) ──────> TXD
PA11 (RX) <────── RXD
                  CANH    ──────> CAN_H ──┐
                  CANL    ──────> CAN_L ──┤
                  VDD     <────── 5V      │
                  VSS     <────── GND     │
                  Rs      <────── GND     │
                                          │
                              120Ω        │
                          (termination)   │
                                          │
                              Other Node ─┘
```

---

## Notes

- All multi-byte values are **BIG-ENDIAN** (MSB first)
- Standard 11-bit CAN IDs are used (not extended 29-bit)
- Maximum message length is 8 bytes
- Angle values: 0-10000 represents 0-100%
- Pressure values: Multiply bar by 10 for integer transmission
- Current values: Direct milliampere values (0-3000 mA)

