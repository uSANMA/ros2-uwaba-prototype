#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import TwistStamped


class UrosCmdVelTeste(Node):
    def __init__(self):
        super().__init__("uros_cmd_vel_test_node")
        self.declare_parameter("set_x", 0.1)
        self.declare_parameter("set_az", 0.2)
        self.declare_parameter("set_frame_id", "uwaba_prototype")
        self.declare_parameter("set_freq", 30)
        self.custom_qos_ = QoSProfile(depth=100, reliability=2, durability=2, liveliness=1, history=2)
        self.x_value_ = self.get_parameter("set_x").value
        self.az_value_ = self.get_parameter("set_az").value
        self.frame_id_value_ = self.get_parameter("set_frame_id").value
        self.timer_freq_ = self.get_parameter("set_freq").value
        self.data_publisher_ = self.create_publisher(TwistStamped, "cmd_vel", self.custom_qos_)
        self.timer_ = self.create_timer(1.0 / self.timer_freq_, self.publish_cmd)

    def publish_cmd(self):
        cmd_msg = self.set_cmd_vel(
            self.frame_id_value_, x=self.x_value_, az=self.az_value_
        )
        self.data_publisher_.publish(cmd_msg)
        self.get_logger().warn(
            "\n- Stamp sec:\t\t"
            + str(cmd_msg.header.stamp.sec)
            + "\n- Stamp nanosec:\t"
            + str(cmd_msg.header.stamp.nanosec)
            + "\n- Sending to linear x:\t"
            + str(cmd_msg.twist.linear.x)
            + "\n- Sending to angular z:\t"
            + str(cmd_msg.twist.angular.z)
            + "\n- Frame ID:\t\t"
            + str(cmd_msg.header.frame_id)
        )

    def set_cmd_vel(
        self,
        frame_id="",
        x=0.0,
        y=0.0,
        z=0.0,
        ax=0.0,
        ay=0.0,
        az=0.0,
    ) -> TwistStamped:
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = frame_id
        cmd_msg.twist.linear.x = x
        cmd_msg.twist.linear.y = y
        cmd_msg.twist.linear.z = z
        cmd_msg.twist.angular.x = ax
        cmd_msg.twist.angular.y = ay
        cmd_msg.twist.angular.z = az
        return cmd_msg


def main(args=None):
    rclpy.init(args=args)
    node = UrosCmdVelTeste()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
