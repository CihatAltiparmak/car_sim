#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty


class CarSimTeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__("car_sim_teleop_keyboard_node")

        self.declare_parameter('velocity_increase_rate', 0.1)
        self.velocity_increase_rate = self.get_parameter('velocity_increase_rate').get_parameter_value().double_value

        self.declare_parameter('steering_increase_rate', 0.1)
        self.steering_increase_rate = self.get_parameter('steering_increase_rate').get_parameter_value().double_value

        self.declare_parameter('max_velocity', 10.0)
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value

        self.declare_parameter('max_steering_angle', 0.5)
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value

        self.declare_parameter('drive_topic', '/itusct/command_cmd')
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.current_velocity = 0.0
        self.current_steering_angle = 0.0

        self.settings = termios.tcgetattr(sys.stdin)
        self.last_key = None
        
        self.drive_msg = AckermannDriveStamped()
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 10)



    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
	    
    def publish_drive_msg(self):
        self.last_key = self.get_key()
        print("[publish_drive_msg]", self.last_key)

        if self.last_key == "s":
            self.publish_stop_msg()
            self.print_information()
            return

        if self.last_key == "w":
            self.current_velocity += self.velocity_increase_rate

        if self.last_key == "x":
            self.current_velocity -= self.velocity_increase_rate

        if self.last_key == "a":
            self.current_steering_angle += self.steering_increase_rate

        if self.last_key == "d":
            self.current_steering_angle -= self.steering_increase_rate

        self.current_velocity = min(self.current_velocity, self.max_velocity)
        self.current_velocity = max(self.current_velocity, -self.max_velocity)

        self.current_steering_angle = min(self.current_steering_angle, self.max_steering_angle)
        self.current_steering_angle = max(self.current_steering_angle, -self.max_steering_angle)

        self.drive_msg.drive.speed = self.current_velocity
        self.drive_msg.drive.steering_angle = self.current_steering_angle
        self.drive_pub.publish(self.drive_msg)

        self.print_information()


    def publish_stop_msg(self):
        self.current_steering_angle = 0.0
        self.current_velocity = 0.0
        self.drive_msg.drive.speed = self.current_velocity
        self.drive_msg.drive.steering_angle = self.current_steering_angle
        self.drive_pub.publish(self.drive_msg)

    def is_break(self):
        if self.last_key == '\x03':
            return True
        else:
            return False

    def on_exit(self):
        self.current_steering_angle = 0
        self.current_velocity = 0
        self.publish_drive_msg()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def print_information(self):
        print("-------------------")
        print("last_key:", self.last_key)
        print("current_velocity:", self.current_velocity)
        print("current_steering:", self.current_steering_angle)

def main(args):
    rclpy.init(args = args)
    teleop_node = CarSimTeleopKeyboardNode()

    while (1):
        teleop_node.publish_drive_msg()
        if teleop_node.is_break():
            break

    teleop_node.on_exit()

if __name__ == "__main__":
    main(sys.argv)