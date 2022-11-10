import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from vesc_msgs.msg import VescCtrl, VescState
from vesc_interface import VescInterface
from threading import Lock
import numpy as np


class VescWrapperNode(Node):
    def __init__(self) -> None:
        super().__init__('vesc_interface_node')

        self.declare_parameter('serial_port', None)
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.05)

        self.declare_parameter('heartbeat_enabled', True)
        self.declare_parameter('heartbeat_rate', 50)

        self.declare_parameter('min_rpm', 0)
        self.declare_parameter('max_rpm', 1000)
        self.declare_parameter('min_handbrake', 0.0)
        self.declare_parameter('max_handbrake', 1.0)
        self.declare_parameter('min_servo_pos', -1.0)
        self.declare_parameter('max_servo_pos', 1.0)

        serial_port: str = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate: int = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout: float = self.get_parameter('timeout').get_parameter_value().double_value
        self._vesc_interface: VescInterface = VescInterface(serial_port, baudrate, timeout)

        self._min_rpm: int = self.get_parameter('min_rpm').get_parameter_value().integer_value
        self._max_rpm: int = self.get_parameter('max_rpm').get_parameter_value().integer_value
        self._min_handbrake: float = self.get_parameter('min_handbrake').get_parameter_value().double_value
        self._max_handbrake: float = self.get_parameter('max_handbrake').get_parameter_value().double_value
        self._min_servo_pos: float = self.get_parameter('min_servo_pos').get_parameter_value().double_value
        self._max_servo_pos: float = self.get_parameter('max_servo_pos').get_parameter_value().double_value

        heartbeat_enabled: bool = self.get_parameter('heartbeat_enabled').get_parameter_value().bool_value
        if heartbeat_enabled:
            self._last_servo_pos: int = 0
            self._last_rpm: float = 0.0
            self._last_handbrake: float = 0.0

            heartbeat_rate: int = self.get_parameter('heartbeat_rate').get_parameter_value().integer_value
            self._update_timer = self.create_timer(1.0 / heartbeat_rate, self._do_heartbeat)

        self._lock: Lock = Lock()

        self._vesc_state_pub = self.create_publisher(VescState, '~/vesc_state', 10)
        self._vesc_ctrl_sub = self.create_subscription(VescCtrl, 'vesc_ctrl', self._on_vesc_ctrl_received, 10)
        self._rpm_sub = self.create_subscription(Int32, 'rpm', self._on_rpm_received, 10)
        self._handbrake_sub = self.create_subscription(Float32, 'handbrake', self._on_handbrake_received, 10)
        self._servo_pos_sub = self.create_subscription(Float32, 'servo_pos', self._on_servo_pos_received, 10)

    def _do_heartbeat(self) -> None:
        self._lock.acquire()
        self._vesc_interface.set_rpm(self._last_rpm)
        self._vesc_interface.set_handbrake(self._last_handbrake)
        self._vesc_interface.set_servo_pos(self._last_servo_pos)
        self._lock.release()

    def _on_vesc_ctrl_received(self, msg: VescCtrl) -> None:
        self._lock.acquire()
        self._last_rpm = np.clip(msg.rpm, self._min_rpm, self._max_rpm)
        self._last_handbrake = np.clip(msg.handbrake, self._min_handbrake, self._max_handbrake)
        self._last_servo_pos = np.clip(msg.servo_pos, self._min_servo_pos, self._max_servo_pos)
        self._lock.release()

    def _on_rpm_received(self, msg: Int32) -> None:
        self._last_rpm = np.clip(msg.data, self._min_rpm, self._max_rpm)

    def _on_handbrake_received(self, msg: Float32) -> None:
        self._last_handbrake = np.clip(msg.data, self._min_handbrake, self._max_handbrake)

    def _on_servo_pos_received(self, msg: Float32) -> None:
        self._last_servo_pos = np.clip(msg.data, self._min_servo_pos, self._max_servo_pos)


def main(args=None) -> None:
    rclpy.init(args=args)
    vesc_wrapper_node = VescWrapperNode()
    rclpy.spin(vesc_wrapper_node)
    vesc_wrapper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
