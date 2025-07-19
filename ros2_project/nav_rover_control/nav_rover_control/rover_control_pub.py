#!/usr/bin/python3
import math
import rclpy
import serial
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        timer_period = 0.01  # 50Hz

        self.sub = self.create_subscription(Twist, '/cmd_vel_nav', self.cmd_vel_callback, 10)
        self.pub = self.create_publisher(Int16MultiArray, 'uart_command', 10)
        self.rover_control = [0.0, 0.0] # Angle, Speed

        self.opposite_degree_R_id = 0x310
        self.opposite_degree_L_id = 0x311
        self.opposite_spped_id = 0x312

        self.angle_can_id = 0x300
        self.speed_can_id = 0x300

        self.latest_twist = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist):
        self.latest_twist = msg

    def calculate_angle_and_velocity(self, twist: Twist):
        # 並進速度と角速度を取得
        linear_x = twist.linear.x
        angular_z = twist.angular.z

        velocity = linear_x
        direction_angle = -angular_z # 右回りを正とするために符号を反転

        return direction_angle, velocity

    def timer_callback(self):
        # 角度と速度を計算
        direction_angle, velocity = self.calculate_angle_and_velocity(self.latest_twist)
        rover_cmd = Int16MultiArray()

        if abs(direction_angle) < 0.3:
            self.rover_control[0] = 180
            self.rover_control[1] = velocity * 100 * 2.0 + 120 # (-1 ~ 1) * 10 * 3 + 120

        elif direction_angle >= 0.2:
            self.rover_control[0] = math.degrees(-direction_angle) * 0.4 + 180
            self.rover_control[1] = (velocity * 100 * 2.0 + abs(direction_angle) * 30) + 120
            self.angle_can_id = self.opposite_degree_R_id

        else:
            self.rover_control[0] = math.degrees(-direction_angle) * 0.4 + 180
            self.rover_control[1] = (velocity * 100 * 2.0+ abs(direction_angle) * 30) + 120
            self.angle_can_id = self.opposite_degree_L_id
        self.speed_can_id = self.opposite_spped_id

        # IDと制御値を順番にデータ配列に追加
        rover_cmd.data = [self.angle_can_id, int(self.rover_control[0]), self.speed_can_id, int(self.rover_control[1])]

        # Publish the rover command
        self.pub.publish(rover_cmd)

        self.rover_control = [0, 0]

def main(args=None):
    rclpy.init(args=args)
    commander = Commander()
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        pass
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
