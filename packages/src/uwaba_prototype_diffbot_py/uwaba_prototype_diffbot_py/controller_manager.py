#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from uwaba_prototype_interfaces.action import ControlActions


class ControllerManager(Node):
    def __init__(self):
        super().__init__("controller_manager")
        self.declare_parameter("goal_request", "empty")
        self.goal_request_ = self.get_parameter("goal_request").value
        self.declare_parameter("child_frame_id", "empty")
        self.goal_child_frame_id_ = self.get_parameter("child_frame_id").value
        self.declare_parameter("managed_node_names", "controller_server_node")
        node_name = self.get_parameter("managed_node_names").value
        self.get_logger().info(f"Nodes: {node_name}")
        service_change_state_name = f"/{node_name}/change_state"
        self.client_ = self.create_client(ChangeState, service_change_state_name)
        self.action_client_ = ActionClient(
            self, ControlActions, "uWABA_prototype/Control_Server"
        )

    def change_state(self, transition: Transition):
        self.client_.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def initialization_sequence(self):
        # Not configured to Inactive
        self.get_logger().info("Trying to switch to configuring")
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = "configure"
        self.change_state(transition)
        self.get_logger().info("Configuring OK, now inactive")

        # Inactive to Active
        self.get_logger().info("Trying to switch to activating")
        transition = Transition()
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = "activate"
        self.change_state(transition)
        self.get_logger().info("Activating OK, now active")
        self.send_goal(self.goal_child_frame_id_, self.goal_request_)

    def send_goal(self, child_frame_id, request):
        self.action_client_.wait_for_server()
        goal = ControlActions.Goal()
        goal.child_frame_id = child_frame_id
        goal.request = request
        self.action_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_feedback_callback(self, feedback_msg):
        process = feedback_msg.feedback.process
        self.get_logger().info(f"Got feedback: {process}")

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(
                self.goal_result_callback
            )
        else:
            self.get_logger().warn("Goal got rejected")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("Canceled")
        self.get_logger().info(f"Result: {result.result_msg}")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerManager()
    node.initialization_sequence()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
