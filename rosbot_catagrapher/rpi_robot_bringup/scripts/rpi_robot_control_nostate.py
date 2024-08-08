#!/usr/bin/env python3

from dataclasses import dataclass
import math
import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from tf2_ros import TransformBroadcaster

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry


def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q


@dataclass
class SerialStatus:
    """Class for different data given by the embedded system"""
    left_ref_speed: float
    right_ref_speed: float
    left_speed: float
    right_speed: float
    left_effort: float
    right_effort: float
    x_pos: float
    y_pos: float
    theta: float
    v: float
    w: float


class RobotControlNode(Node):
    """Simple node for controlling a differential drive robot"""
    def __init__(self):
        super().__init__('rpi_robot_node')
        
        self.declare_parameter('pico_port', '/dev/ttyUSB0')

        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        self.twist_subscription

        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        time.sleep(0.2)
        port = self.get_parameter('pico_port').get_parameter_value().string_value
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        self.get_logger().info(f'UART initialized on port {port}')
        self.twist = Twist()
        # set timer
        self.pub_period = 0.04  # 0.04 seconds = 25 hz = pid rate for robot
        self.pub_timer = self.create_timer(self.pub_period, self.timer_callback)
    
    def timer_callback(self):
        robot_state = self.send_command(self.twist.linear.x, self.twist.angular.z)
        if robot_state is None:
            return

    def send_command(self, linear: float, angular: float) -> SerialStatus:
        
        command = f'{linear:.3f},{angular:.3f}\n'.encode('UTF-8')
        self.get_logger().debug(f'Sending command: "{command}"')
        self.ser.write(command)
    
        timeout = 0.1  # 100 ms timeout
        start_time = time.time()
    
        while self.ser.in_waiting == 0:
            if time.time() - start_time > timeout:
                self.get_logger().info("No response, resending command")
                self.ser.write(command)
                start_time = time.time()
        
        try:
            res = self.ser.readline().decode('UTF-8')
            values = res.split(',')
            values = list(map(float, values))
        except (UnicodeDecodeError, ValueError) as e:
            self.get_logger().info(f'Bad data: "{res}" - {e}')
            return None

        return SerialStatus(*values)

    def twist_callback(self, twist: Twist):
        self.twist = twist
        self.get_logger().info(f'Received cmd_vel: linear=({twist.linear.x}), angular=({twist.angular.z})')

def main(args=None):
    rclpy.init(args=args)
    robot_control_node = RobotControlNode()
    rclpy.spin(robot_control_node)

    robot_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
