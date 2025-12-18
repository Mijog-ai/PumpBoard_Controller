"""
Serial Communication Protocol Handler for STM32F407 PumpBoard Controller
Implements frame protocol matching ParseFrame.c/h in STM32 firmware
"""

import serial
import serial.tools.list_ports
import struct
import threading
import time
from queue import Queue, Empty
from PyQt5.QtCore import QObject, pyqtSignal

# Protocol Constants
FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55

# Command Codes
CMD_READ = 0x01
CMD_WRITE = 0x02
CMD_DRAW_WAVE = 0x03
PARA_WRITE = 0x05
PARA_READ = 0x06

# Response codes
RESP_OK = 0x00
RESP_ERROR = 0xFF


class CRC16:
    """CRC16 calculation matching STM32 firmware"""

    # CRC16 lookup table (polynomial 0x8005)
    CRC_TABLE = [
        0x0000, 0x8005, 0x800f, 0x000a, 0x801b, 0x001e, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003c, 0x8039, 0x0028, 0x802d, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006c, 0x8069, 0x0078, 0x807d, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805f, 0x005a, 0x804b, 0x004e, 0x0044, 0x8041,
        0x80c3, 0x00c6, 0x00cc, 0x80c9, 0x00d8, 0x80dd, 0x80d7, 0x00d2,
        0x00f0, 0x80f5, 0x80ff, 0x00fa, 0x80eb, 0x00ee, 0x00e4, 0x80e1,
        0x00a0, 0x80a5, 0x80af, 0x00aa, 0x80bb, 0x00be, 0x00b4, 0x80b1,
        0x8093, 0x0096, 0x009c, 0x8099, 0x0088, 0x808d, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018c, 0x8189, 0x0198, 0x819d, 0x8197, 0x0192,
        0x01b0, 0x81b5, 0x81bf, 0x01ba, 0x81ab, 0x01ae, 0x01a4, 0x81a1,
        0x01e0, 0x81e5, 0x81ef, 0x01ea, 0x81fb, 0x01fe, 0x01f4, 0x81f1,
        0x81d3, 0x01d6, 0x01dc, 0x81d9, 0x01c8, 0x81cd, 0x81c7, 0x01c2,
        0x0140, 0x8145, 0x814f, 0x014a, 0x815b, 0x015e, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017c, 0x8179, 0x0168, 0x816d, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012c, 0x8129, 0x0138, 0x813d, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811f, 0x011a, 0x810b, 0x010e, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030c, 0x8309, 0x0318, 0x831d, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833f, 0x033a, 0x832b, 0x032e, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836f, 0x036a, 0x837b, 0x037e, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035c, 0x8359, 0x0348, 0x834d, 0x8347, 0x0342,
        0x03c0, 0x83c5, 0x83cf, 0x03ca, 0x83db, 0x03de, 0x03d4, 0x83d1,
        0x83f3, 0x03f6, 0x03fc, 0x83f9, 0x03e8, 0x83ed, 0x83e7, 0x03e2,
        0x83a3, 0x03a6, 0x03ac, 0x83a9, 0x03b8, 0x83bd, 0x83b7, 0x03b2,
        0x0390, 0x8395, 0x839f, 0x039a, 0x838b, 0x038e, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828f, 0x028a, 0x829b, 0x029e, 0x0294, 0x8291,
        0x82b3, 0x02b6, 0x02bc, 0x82b9, 0x02a8, 0x82ad, 0x82a7, 0x02a2,
        0x82e3, 0x02e6, 0x02ec, 0x82e9, 0x02f8, 0x82fd, 0x82f7, 0x02f2,
        0x02d0, 0x82d5, 0x82df, 0x02da, 0x82cb, 0x02ce, 0x02c4, 0x82c1,
        0x8243, 0x0246, 0x024c, 0x8249, 0x0258, 0x825d, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827f, 0x027a, 0x826b, 0x026e, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822f, 0x022a, 0x823b, 0x023e, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021c, 0x8219, 0x0208, 0x820d, 0x8207, 0x0202
    ]

    @staticmethod
    def calculate(data):
        """Calculate CRC16 for byte array"""
        crc = 0xFFFF
        for byte in data:
            crc = ((crc << 8) & 0xFF00) ^ CRC16.CRC_TABLE[((crc >> 8) ^ byte) & 0xFF]
        return crc & 0xFFFF


class SerialProtocol(QObject):
    """
    Serial protocol handler with frame parsing
    Frame format: [0xAA][CMD][INDEX][LEN][PAYLOAD...][CRC16_L][CRC16_H][0x55]
    """

    # Signals for Qt integration
    connected = pyqtSignal()
    disconnected = pyqtSignal()
    data_received = pyqtSignal(int, int)  # (index, value)
    error_occurred = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_connected = False
        self.rx_thread = None
        self.tx_queue = Queue()
        self.stop_event = threading.Event()

        self.response_queue = Queue()
        self.response_timeout = 1.0  # seconds

    def list_ports(self):
        """List available serial ports"""
        ports = serial.tools.list_ports.comports()
        return [(port.device, port.description) for port in ports]

    def connect(self, port, baudrate=115200):
        """Connect to serial port"""
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )

            self.is_connected = True
            self.stop_event.clear()

            # Start receiver thread
            self.rx_thread = threading.Thread(target=self._receive_thread, daemon=True)
            self.rx_thread.start()

            self.connected.emit()
            return True

        except Exception as e:
            self.error_occurred.emit(f"Connection failed: {str(e)}")
            return False

    def disconnect(self):
        """Disconnect from serial port"""
        if self.is_connected:
            self.stop_event.set()
            if self.rx_thread:
                self.rx_thread.join(timeout=2.0)

            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

            self.is_connected = False
            self.disconnected.emit()

    def _build_frame(self, cmd, index, data=None):
        """
        Build protocol frame
        Frame: [0xAA][CMD][INDEX][LEN][PAYLOAD...][CRC16_L][CRC16_H][0x55]
        """
        frame = bytearray()
        frame.append(FRAME_HEADER)
        frame.append(cmd)
        frame.append(index)

        if data is None:
            data = []
        elif isinstance(data, int):
            # Convert int to 4 bytes (little-endian)
            data = list(struct.pack('<I', data))

        frame.append(len(data))
        frame.extend(data)

        # Calculate CRC on CMD, INDEX, LEN, and PAYLOAD
        crc_data = frame[1:]  # Exclude header
        crc = CRC16.calculate(crc_data)

        frame.append(crc & 0xFF)  # CRC low byte
        frame.append((crc >> 8) & 0xFF)  # CRC high byte
        frame.append(FRAME_TAIL)

        return bytes(frame)

    def _parse_frame(self, data):
        """
        Parse received frame
        Returns: (cmd, index, payload) or None if invalid
        """
        if len(data) < 7:  # Minimum frame size
            return None

        if data[0] != FRAME_HEADER or data[-1] != FRAME_TAIL:
            return None

        cmd = data[1]
        index = data[2]
        length = data[3]

        if len(data) != (7 + length):
            return None

        payload = data[4:4+length]
        crc_received = data[4+length] | (data[5+length] << 8)

        # Verify CRC
        crc_data = data[1:4+length]
        crc_calculated = CRC16.calculate(crc_data)

        if crc_received != crc_calculated:
            self.error_occurred.emit(f"CRC error: expected {crc_calculated:04X}, got {crc_received:04X}")
            return None

        return (cmd, index, payload)

    def _receive_thread(self):
        """Background thread for receiving data"""
        buffer = bytearray()

        while not self.stop_event.is_set():
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(data)

                    # Look for complete frames
                    while FRAME_HEADER in buffer:
                        header_idx = buffer.index(FRAME_HEADER)

                        # Remove data before header
                        if header_idx > 0:
                            buffer = buffer[header_idx:]

                        # Check if we have enough data for length field
                        if len(buffer) < 4:
                            break

                        length = buffer[3]
                        frame_size = 7 + length

                        # Wait for complete frame
                        if len(buffer) < frame_size:
                            break

                        # Extract and parse frame
                        frame = buffer[:frame_size]
                        buffer = buffer[frame_size:]

                        result = self._parse_frame(frame)
                        if result:
                            cmd, index, payload = result
                            self._handle_received_frame(cmd, index, payload)

                else:
                    time.sleep(0.001)

            except Exception as e:
                if self.is_connected:
                    self.error_occurred.emit(f"Receive error: {str(e)}")
                break

    def _handle_received_frame(self, cmd, index, payload):
        """Handle received frame"""
        if cmd == CMD_READ and len(payload) == 4:
            # Read response with 32-bit value
            value = struct.unpack('<i', bytes(payload))[0]  # Signed 32-bit
            self.data_received.emit(index, value)
            self.response_queue.put((index, value))

        elif cmd == CMD_WRITE:
            # Write acknowledgment
            self.response_queue.put((index, RESP_OK))

    def read_parameter(self, index):
        """
        Read single parameter
        Returns: value or None if timeout/error
        """
        if not self.is_connected:
            return None

        try:
            # Clear response queue
            while not self.response_queue.empty():
                try:
                    self.response_queue.get_nowait()
                except Empty:
                    break

            # Send read command
            frame = self._build_frame(CMD_READ, index)
            self.serial_port.write(frame)

            # Wait for response
            try:
                idx, value = self.response_queue.get(timeout=self.response_timeout)
                if idx == index:
                    return value
            except Empty:
                self.error_occurred.emit(f"Timeout reading index {index}")
                return None

        except Exception as e:
            self.error_occurred.emit(f"Read error: {str(e)}")
            return None

    def write_parameter(self, index, value):
        """
        Write single parameter
        Returns: True if successful
        """
        if not self.is_connected:
            return False

        try:
            # Clear response queue
            while not self.response_queue.empty():
                try:
                    self.response_queue.get_nowait()
                except Empty:
                    break

            # Send write command with 32-bit value
            frame = self._build_frame(CMD_WRITE, index, value)
            self.serial_port.write(frame)

            # Wait for acknowledgment
            try:
                idx, resp = self.response_queue.get(timeout=self.response_timeout)
                return resp == RESP_OK
            except Empty:
                self.error_occurred.emit(f"Timeout writing index {index}")
                return False

        except Exception as e:
            self.error_occurred.emit(f"Write error: {str(e)}")
            return False

    def read_multiple(self, indices):
        """
        Read multiple parameters
        Returns: dict {index: value}
        """
        results = {}
        for index in indices:
            value = self.read_parameter(index)
            if value is not None:
                results[index] = value
            time.sleep(0.01)  # Small delay between reads
        return results

    def write_multiple(self, params):
        """
        Write multiple parameters
        params: dict {index: value}
        Returns: dict {index: success}
        """
        results = {}
        for index, value in params.items():
            success = self.write_parameter(index, value)
            results[index] = success
            time.sleep(0.01)
        return results
