#!/usr/bin/env python3
import rclpy
import time

from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition


class LifecycleControllerNodeManager(Node):
    def __init__(self):
        super().__init__("lifecycle_controller_manager")
        self.declare_parameter("managed_node_name", rclpy.Parameter.Type.STRING)

    def get_state(self):
        request_state = GetState.Request()
        future = self.client_get_state.call_async(request_state)
        rclpy.spin_until_future_complete(self, future)
        return (future.result().current_state.id, future.result().current_state.label)

    def change_state(self, transition: Transition):
        self.client_change_state.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = self.client_change_state.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def initialization_sequence(self):
        self.get_logger().info("Initializing first transition...")

        self.get_logger().info("Changing state to 'configure'...")

        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"

        id, label = self.get_state()
        self.get_logger().warn("id: " + str(id) + " label: " + label)
        
        time.sleep(1)
        self.change_state(transition)
        time.sleep(1)
        
        id, label = self.get_state()
        self.get_logger().warn("id: " + str(id) + " label: " + label)

        self.get_logger().info("State changed to configured!")


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleControllerNodeManager()
    node.initialization_sequence()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
