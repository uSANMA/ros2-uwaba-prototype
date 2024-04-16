#!/usr/bin/env python3
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn



class ControllerInterfaceNode(LifecycleNode):
    def __init__(self):
        super().__init__("controller_interface_node")

    # Usually this part would be the HW communications with ROS2
    def on_configure(self, pŕevious_state: LifecycleState) -> TransitionCallbackReturn:
        # Add code here to check microROS connection
        return TransitionCallbackReturn.SUCCESS  # or FAILURE

    # Activate/Enable Hardware, for an example
    def on_activate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        # Enable communications
        return super().on_activate(previous_state)

    # Deactivate/Disable Hardware, for an example
    def on_deactivate(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        # Disable communications
        return super().on_deactivate(previous_state)

    # Destroy ROS2 communications, disconnect HardWare
    def on_cleanup(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS
    
    def on_error(self, previous_state: LifecycleState) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS
    
    def destroy_it_all(self):
        self.destroy_timer(self.number_timer_)
        self.number_ = 1
        self.destroy_lifecycle_publisher(self.number_publisher_)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerInterfaceNode()
    rclpy.spin(node)
    rclpy.shutdown()
