#!/usr/bin/env python3
import rclpy
from math import pi
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import JointState, Imu, LaserScan, Temperature
from geometry_msgs.msg import TwistStamped


class UrosEmulator(Node):
    def __init__(self):
        super().__init__("uros_emulator_node")
        self.get_logger().info("microROS Emulator has started.")

        self.encoder_publish_freq_ = 30
        self.lidar_publish_freq_ = 50
        self.temperature_publish_freq_ = 70
        self.imu_publish_freq_ = 50

        self.encoder_msgs_: JointState = JointState()
        self.lidar_msgs_: LaserScan = LaserScan()
        self.temperature_msgs_: Temperature = Temperature()
        self.imu_msgs_: Imu = Imu()

        self.linear_x_ = None
        self.angular_z_ = None
        self.left_encoder_vel_ = None
        self.right_encoder_vel_ = None

        self.encoder_publisher_ = self.create_publisher(
            JointState, "micro_encoders", 10
        )
        self.lidar_publisher_ = self.create_publisher(LaserScan, "micro_laser", 10)
        self.temperature_publisher_ = self.create_publisher(
            Temperature, "micro_temp", 10
        )
        self.imu_publisher_ = self.create_publisher(Imu, "micro_imu", 10)
        self.cmd_vel_subscription_ = self.create_subscription(
            TwistStamped,
            "uwaba_controller_server_node/cmd_vel",
            self.cmd_vel_subscription,
            10,
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
            self.encoder_msgs_.header.frame_id = "motor_vels"
            self.encoder_msgs_.name = ["Left_wheel_velocity", "Right_wheel_velocity"]
            self.encoder_msgs_.velocity = [
                self.left_encoder_vel_,
                self.right_encoder_vel_,
            ]
            self.encoder_publisher_.publish(self.encoder_msgs_)
        elif self.linear_x_ is None and self.angular_z_ is None:
            self.encoder_msgs_.header.stamp = self.get_clock().now().to_msg()
            self.encoder_msgs_.header.frame_id = "motor_vels"
            self.encoder_msgs_.name = ["Left_wheel_velocity", "Right_wheel_velocity"]
            self.encoder_msgs_.velocity = [
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
        self.left_encoder_vel_ = self.linear_x_ - 0.5 * 0.13607 * self.angular_z_
        self.right_encoder_vel_ = self.linear_x_ + 0.5 * 0.13607 * self.angular_z_


def main(args=None):
    rclpy.init(args=args)
    node = UrosEmulator()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
