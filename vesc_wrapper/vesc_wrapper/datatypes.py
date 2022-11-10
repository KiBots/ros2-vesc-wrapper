from enum import IntEnum
from dataclasses import dataclass


class COMM_PACKET_ID(IntEnum):
    COMM_FW_VERSION = 0
    COMM_GET_VALUES = 4
    COMM_SET_CURRENT_BRAKE = 7
    COMM_SET_RPM = 8
    COMM_SET_HANDBRAKE = 10
    COMM_SET_SERVO_POS = 12


@dataclass
class Values:
    temp_fet_filtered: float#16
    temp_motor_filtered: float#16
    avg_motor_current: float#32
    avg_input_current: float#32
    avg_id: float#32
    avg_iq: float#32
    duty_cycle_now: float#16
    rpm: float#32
    input_voltage_filtered: float#16
    amp_hours: float#32
    amp_hours_charged: float#32
    watt_hours: float#32
    watt_hours_charged: float#32
    tachometer_value: int#32
    tachometer_abs_value: int#32
    fault: int#8
    pid_pos_now: float#32
    controller_id: int#8
    ntc_temp_mos1: float#16
    ntc_temp_mos2: float#16
    ntc_temp_mos3: float#16
    avg_vd: float#32
    avg_vq: float#32
    status: int#8

