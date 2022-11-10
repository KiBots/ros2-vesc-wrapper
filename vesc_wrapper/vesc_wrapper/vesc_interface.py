import serial
from datatypes import COMM_PACKET_ID, Values
import buffer
from crc import crc16


class VescInterface:
    def __init__(self, serial_port: str, baudrate: int, timeout: float) -> None:
        self._serial_port = serial.Serial(port=serial_port, baudrate=baudrate, timeout=timeout)

    def set_rpm(self, rpm: int) -> None:
        payload = bytearray(5)
        payload[0] = COMM_PACKET_ID.COMM_SET_RPM
        buffer.append_int32(payload, rpm, 1)
        self._send_message(payload)

    def set_handbrake(self, handbrake: float) -> None:
        payload = bytearray(5)
        payload[0] = COMM_PACKET_ID.COMM_SET_HANDBRAKE
        buffer.append_float32(payload, handbrake, 1e3, 1)
        self._send_message(payload)

    def set_servo_pos(self, servo_pos: float) -> None:
        payload = bytearray(5)
        payload[0] = COMM_PACKET_ID.COMM_SET_SERVO_POS
        buffer.append_float16(payload, servo_pos, 1e3, 1)
        self._send_message(payload)

    def _send_message(self, payload: bytearray) -> None:
        message = bytearray(len(payload) + 5)  # 1 byte start, 1 byte payload-length, n bytes payload, 2 bytes checksum, 1 byte end
        idx = buffer.append_int8(message, 2, 0)
        idx = buffer.append_int8(message, len(payload), idx)
        idx = buffer.append_buffer(message, payload, idx)
        idx = buffer.append_int16(message, crc16(payload), idx)
        buffer.append_int8(message, 3, idx)
        self._serial_port.write(message)

    def _get_values(self, message: bytearray) -> Values:
        result: Values = Values()
        
        idx, result.temp_fet_filtered = buffer.get_float16(message, 1e1, 0)
        idx, result.temp_motor_filtered = buffer.get_float16(message, 1e1, idx)
        idx, result.avg_motor_current = buffer.get_float32(message, 1e2, idx)
        idx, result.avg_input_current = buffer.get_float32(message, 1e2, idx)
        idx, result.avg_id = buffer.get_float32(message, 1e2, idx)
        idx, result.avg_iq = buffer.get_float32(message, 1e2, idx)
        idx, result.duty_cycle_now = buffer.get_float16(message, 1e3, idx)
        idx, result.rpm = buffer.get_float32(message, 1e0, idx)
        idx, result.input_voltage_filtered = buffer.get_float16(message, 1e1, idx)
        idx, result.amp_hours = buffer.get_float32(message, 1e4, idx)
        idx, result.amp_hours_charged = buffer.get_float32(message, 1e4, idx)
        idx, result.watt_hours = buffer.get_float32(message, 1e4, idx)
        idx, result.watt_hours_charged = buffer.get_float32(message, 1e4, idx)
        idx, result.tachometer_value = buffer.get_int32(message, idx)
        idx, result.tachometer_abs_value = buffer.get_int32(message, idx)
        idx, result.fault = buffer.get_int8(message, idx)
        idx, result.pid_pos_now = buffer.get_float32(message, 1e6, idx)
        idx, result.controller_id = buffer.get_int8(message, idx)
        idx, result.ntc_temp_mos1 = buffer.get_float16(message, 1e1, idx)
        idx, result.ntc_temp_mos2 = buffer.get_float16(message, 1e1, idx)
        idx, result.ntc_temp_mos3 = buffer.get_float16(message, 1e1, idx)
        idx, result.avg_vd = buffer.get_float32(message, 1e3, idx)
        idx, result.avg_vq = buffer.get_float32(message, 1e3, idx)
        result.status = buffer.get_int8(message, idx)

        return result
