#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState


class UrosTestNode(Node):
    def __init__(self):
        super().__init__("Uros_Test_Node")
        # Set publish frequency [Hz]
        self.timer_freq_ = 1
        # Configure publishers and subscribers
        self.data_publisher_ = self.create_publisher(TwistStamped, "cmd_vel", 10)
        # self.data_publisher2_ = self.create_publisher(JointState, "Encoder_Msgs", 10)
        # self.data_subscriber_ = self.create_subscription(
        #     JointState, "/motor/encoder", self.subscription_encoder, 10
        # )
        self.timer_ = self.create_timer(1.0 / self.timer_freq_, self.publish_cmd)

    def publish_cmd(self):
        cmd_msg = self.set_cmd_vel("right_wheel", x=5.66, az=1.57)
        self.data_publisher_.publish(cmd_msg)
        self.get_logger().warn(
            "\nSending to linear x:\t"
            + str(cmd_msg.twist.linear.x)
            + "\nand to angular z:\t"
            + str(cmd_msg.twist.angular.z)
            + "\nat:\t\t\t"
            + str(cmd_msg.header.frame_id)
            + "\nstamp sec:\t\t"
            + str(cmd_msg.header.stamp.sec)
            + "\nstamp nanosec:\t\t"
            + str(cmd_msg.header.stamp.nanosec)
        )

    # def subscription_encoder(self, msg: JointState):
    #     self.data_publisher2_.publish(msg)
    #     self.get_logger().warn("Dado recebido: " + str(msg))

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
    node = UrosTestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
