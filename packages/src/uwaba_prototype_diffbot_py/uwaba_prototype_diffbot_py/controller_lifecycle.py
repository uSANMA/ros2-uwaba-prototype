#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn


class ControllerInterfaceNode(LifecycleNode):
    def __init__(self):
        super().__init__("controller_lifecycle_node")
        self.qos_profile_ = QoSProfile(depth=10)

    # Usually this part would be the HW communications with ROS2
    def on_configure(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        # Create cmd_vel and joint_state subscribers
        self.get_logger().info("State: on_activate")
        return TransitionCallbackReturn.SUCCESS

    # Activate/Enable Hardware, for an example
    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        # Enable and check communications with microROS
        self.get_logger().info("State: on_activate")
        return super().on_activate(previous_state)

    # Deactivate/Disable Hardware, for an example
    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        # Disable communications
        self.get_logger().info("State: on_deactivate")
        return super().on_deactivate(previous_state)

    # Destroy ROS2 communications, disconnect HardWare
    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        # Clean all process and terminate them
        self.get_logger().info("State: on_cleanup")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("State: on_shutdown")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("State: on_error")
        return TransitionCallbackReturn.SUCCESS

    def destroy_it_all(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ControllerInterfaceNode()
    rclpy.spin_once(node)
    rclpy.shutdown()
