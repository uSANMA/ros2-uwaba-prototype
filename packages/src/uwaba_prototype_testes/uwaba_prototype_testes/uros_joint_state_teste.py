#!/usr/bin/env python3
import rclpy
from math import pi
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped


class UrosJointStateTeste(Node):
    def __init__(self):
        super().__init__("uros_joint_state_test_node")
        self.declare_parameter("set_frame_id", "encoders")
        self.frame_id_value_ = self.get_parameter("set_frame_id").value
        self.joint_state_publisher_ = self.create_publisher(JointState, "encoder", 10)
        self.cmd_vel_subscriber_ = self.create_subscription(
            TwistStamped, "cmd_vel", self.cmd_vel_sub, 10
        )
        self.degree_ = pi / 180
        self.left_wheel_pos_ = 0.0
        self.right_wheel_pos_ = 0.0
        self.left_wheel_pos_inc_ = 0.001
        self.right_wheel_pos_inc_ = 0.001

    def cmd_vel_sub(self, twist_msgs: TwistStamped):
        self.publish_joint(twist_msgs)

    def publish_joint(self, cmd_vel_msgs: TwistStamped):
        # Insert some mathmagic here
        # Here I'll have to implement some logic to convert linear and angular values
        # to positions and maybe also velocity.
        # First idea is to increase position as ticks over time to simulate encoder data
        # and use Twist values to augment those ticks
        # Remember to take into consideration the different values for each wheel depending
        # on the twist angular values as in the differential drive robot
        self.left_wheel_pos_ = cmd_vel_msgs.twist.linear.x
        self.right_wheel_pos_ = cmd_vel_msgs.twist.linear.x

        joint_state = self.set_joint_state(
            self.frame_id_value_,
            name=["left_wheel", "right_wheel"],
            position=[self.left_wheel_pos_, self.right_wheel_pos_],
        )
        self.joint_state_publisher_.publish(joint_state)
        self.get_logger().warn(
            "\n- Stamp sec:\t\t"
            + str(joint_state.header.stamp.sec)
            + "\n- Stamp nanosec:\t"
            + str(joint_state.header.stamp.nanosec)
            + "\n- Frame ID:\t\t"
            + str(joint_state.header.frame_id)
            + "\n- Names:\t"
            + str(joint_state.name)
            + f"\n- Position:{joint_state.name[0]}\t"
            + str(joint_state.position[0])
            + f"\n- Position:{joint_state.name[1]}\t"
            + str(joint_state.position[1])
        )

    def set_joint_state(
        self, frame_id="", name=[], position=[], velocity=[], effort=[]
    ) -> JointState:
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = frame_id
        joint_state.name = name
        joint_state.position = position
        joint_state.velocity = velocity
        joint_state.effort = effort
        return joint_state


def main(args=None):
    rclpy.init(args=args)
    node = UrosJointStateTeste()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
