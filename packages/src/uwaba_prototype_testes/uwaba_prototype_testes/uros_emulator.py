#!/usr/bin/env python3
import rclpy
import time
import random
from math import pi
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState, Imu, LaserScan, Temperature
from geometry_msgs.msg import TwistStamped
from uwaba_prototype_interfaces.msg import EncoderMsg


class UrosEmulator(Node):
    def __init__(self):
        super().__init__("uros_emulator_node")
        self.get_logger().info("microROS Emulator has started.")

        self.qos_profile_micro_ = QoSProfile(
            depth=10, reliability=2, durability=2, liveliness=1
        )

        self.encoder_publish_freq_ = 30
        self.lidar_publish_freq_ = 6
        self.temperature_publish_freq_ = 70
        self.imu_publish_freq_ = 50

        self.encoder_msgs_ = EncoderMsg()
        self.lidar_msgs_ = LaserScan()
        self.temperature_msgs_ = Temperature()
        self.imu_msgs_ = Imu()

        self.linear_x_ = None
        self.angular_z_ = None
        self.left_wheel_vel_ = 0.0
        self.right_wheel_vel_ = 0.0

        self.wheel_distance_ = 0.13607

        self.encoder_publisher_ = self.create_publisher(
            EncoderMsg, "micro_encoders", self.qos_profile_micro_
        )
        self.lidar_publisher_ = self.create_publisher(
            LaserScan, "micro_laserscan", self.qos_profile_micro_
        )
        self.temperature_publisher_ = self.create_publisher(
            Temperature, "micro_temperature", self.qos_profile_micro_
        )
        self.imu_publisher_ = self.create_publisher(
            Imu, "micro_imu", self.qos_profile_micro_
        )
        self.cmd_vel_subscription_ = self.create_subscription(
            TwistStamped,
            "uwaba_controller_server_node/cmd_vel",
            self.cmd_vel_subscription,
            self.qos_profile_micro_,
            callback_group=ReentrantCallbackGroup(),
        )

        self.encoder_publish_rate_ = self.create_timer(
            1.0 / self.encoder_publish_freq_,
            self.encoder_publish,
            callback_group=ReentrantCallbackGroup(),
        )
        self.lidar_publish_rate_ = self.create_timer(
            1.0 / self.lidar_publish_freq_,
            self.lidar_publish,
            callback_group=ReentrantCallbackGroup(),
        )
        self.temperature_publish_rate_ = self.create_timer(
            1.0 / self.temperature_publish_freq_,
            self.temperature_publish,
            callback_group=ReentrantCallbackGroup(),
        )
        self.imu_publish_rate_ = self.create_timer(
            1.0 / self.imu_publish_freq_,
            self.imu_publish,
            callback_group=ReentrantCallbackGroup(),
        )

    def encoder_publish(self):
        if self.linear_x_ is not None and self.angular_z_ is not None:
            self.encoder_msgs_.header.stamp = self.get_clock().now().to_msg()
            self.encoder_msgs_.encoders = [
                self.left_wheel_vel_,
                self.right_wheel_vel_,
            ]
            self.encoder_publisher_.publish(self.encoder_msgs_)
        elif self.linear_x_ is None and self.angular_z_ is None:
            self.encoder_msgs_.header.stamp = self.get_clock().now().to_msg()
            self.encoder_msgs_.encoders = [
                0.0,
                0.0,
            ]
            self.encoder_publisher_.publish(self.encoder_msgs_)
        else:
            self.get_logger().warn("Linear and angular velocities not yet ready.")

    def lidar_publish(self):
        self.lidar_publisher_.publish(self.lidar_msgs_)

    def temperature_publish(self):
        self.temperature_publisher_.publish(self.temperature_msgs_)

    def imu_publish(self):
        self.imu_publisher_.publish(self.imu_msgs_)

    def cmd_vel_subscription(self, cmd_vel: TwistStamped):
        self.linear_x_ = cmd_vel.twist.linear.x
        self.angular_z_ = cmd_vel.twist.angular.z
        # self.left_wheel_vel_ = self.linear_x_ - 0.5 * self.wheel_distance_ * self.angular_z_
        # self.right_wheel_vel_ = self.linear_x_ + 0.5 * self.wheel_distance_ * self.angular_z_
        # Simulate some delay and noise in the motor response
        if self.linear_x_ != 0.0 or self.angular_z_ != 0.0:
            self.left_wheel_vel_ = float(
                self.linear_x_
                - 0.5 * self.wheel_distance_ * self.angular_z_
                # + random.uniform(-0.01, 0.01)
            )
            self.right_wheel_vel_ = float(
                self.linear_x_
                + 0.5 * self.wheel_distance_ * self.angular_z_
                # + random.uniform(-0.01, 0.01)
            )
        else:
            self.left_wheel_vel_ = (
                self.linear_x_ - 0.5 * self.wheel_distance_ * self.angular_z_
            )
            self.right_wheel_vel_ = (
                self.linear_x_ + 0.5 * self.wheel_distance_ * self.angular_z_
            )

        time.sleep(0.05)  # Simulate processing delay

        # Simulate encoder feedback
        self.update_from_encoders(self.left_wheel_vel_, self.right_wheel_vel_)

    def update_from_encoders(self, left_wheel_velocity, right_wheel_velocity):
        linear_x = (left_wheel_velocity + right_wheel_velocity) / 2.0
        angular_z = (right_wheel_velocity - left_wheel_velocity) / self.wheel_distance_

        # Update internal state or publish these values
        self.get_logger().info(
            f"Linear velocity: {linear_x}, Angular velocity: {angular_z}"
        )
        # self.linear_x_ = linear_x
        # self.angular_z_ = angular_z


def main(args=None):
    rclpy.init(args=args)
    node = UrosEmulator()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
