import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
from vesc_msgs.msg import VescCtrl, VescState
import serial

class VescInterfaceNode(Node):
    def __init__(self):
        super().__init__('vesc_interface_node')

        self.declare_parameter('serial_port', None)
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.05)
        self.declare_parameter('update_rate', 50)
        
        self._serial_port = self.get_parameter('serial_port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._timeout = self.get_parameter('timeout').value
        self._update_rate = self.get_parameter('update_rate').value

        self.serial_port = serial.Serial(port=self._serial_port, baudrate=self._baudrate, timeout=self._timeout)

        self._last_servo_pos = 0.0
        self._last_rpm = 0
        self._last_handbrake = 0

        self._update_timer = self.create_timer(1.0 / self._update_rate, self._update)

        self._vesc_state_pub = self.create_publisher(VescState, '~/vesc_state', 10)
        
        self._vesc_ctrl_sub = self.create_subscription(VescCtrl, 'vesc_ctrl', self._on_vesc_ctrl_received, 10)
        self._rpm_sub = self.create_subscription(Float64, 'rpm', self._on_rpm_received, 10)
        self._handbrake_sub = self.create_subscription(Float64, 'handbrake', self._on_handbrake_received, 10)
        self._servo_pos_sub = self.create_subscription(Float64, 'servo_pos', self._on_servo_pos_received, 10)

    def _update(self):
        pass

    def _on_vesc_ctrl_received(self, msg):
        self._last_rpm = msg.rpm
        self._last_handbrake = msg.handbrake
        self._last_servo_pos = msg.servo_pos

    def _on_rpm_received(self, msg):
        self._last_rpm = msg.data

    def _on_handbrake_received(self, msg):
        self._last_handbrake = msg.data

    def _on_servo_pos_received(self, msg):
        self._last_servo_pos = msg.data

def main(args=None):
    rclpy.init(args=args)
    vesc_interface_node = VescInterfaceNode()
    rclpy.spin(vesc_interface_node)
    vesc_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
