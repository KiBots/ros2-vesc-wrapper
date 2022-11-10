import serial
from datatypes import COMM_PACKET_ID
from buffer import append_int8, append_int16, append_int32, append_float16, append_float32, append_buffer
from crc import crc16


class VescInterface:
    def __init__(self, serial_port: str, baudrate: int, timeout: float) -> None:
        self._serial_port = serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout)

    def set_rpm(self, rpm: int) -> None:
        payload = bytearray(5)
        payload[0] = COMM_PACKET_ID.COMM_SET_RPM
        append_int32(payload, rpm, 1)
        self._send_message(payload)

    def set_handbrake(self, handbrake: float) -> None:
        payload = bytearray(5)
        payload[0] = COMM_PACKET_ID.COMM_SET_HANDBRAKE
        append_float32(payload, handbrake, 1e3, 1)
        self._send_message(payload)

    def set_servo_pos(self, servo_pos: float) -> None:
        payload = bytearray(5)
        payload[0] = COMM_PACKET_ID.COMM_SET_SERVO_POS
        append_float16(payload, servo_pos, 1e3, 1)
        self._send_message(payload)

    def _send_message(self, payload: bytearray) -> None:
        message = bytearray(len(payload) + 5)  # 1 byte start, 1 byte payload-length, n bytes payload, 2 bytes checksum, 1 byte end
        idx = append_int8(message, 2, 0)
        idx = append_int8(message, len(payload), idx)
        idx = append_buffer(message, payload, idx)
        idx = append_int16(message, crc16(payload), idx)
        append_int8(message, 3, idx)
        self._serial_port.write(message)
