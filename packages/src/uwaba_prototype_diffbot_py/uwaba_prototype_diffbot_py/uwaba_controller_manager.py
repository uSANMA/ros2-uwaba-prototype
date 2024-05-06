#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node, Executor
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from uwaba_prototype_interfaces.action import ControlActions


class ControllerManager(Node):
    def __init__(self):
        super().__init__("uwaba_controller_manager_node")
        self.declare_parameter("goal_request", "empty")
        self.goal_request_ = self.get_parameter("goal_request").value
        self.declare_parameter("child_frame_id", "empty")
        self.goal_child_frame_id_ = self.get_parameter("child_frame_id").value
        self.declare_parameter("managed_node_name", "uwaba_controller_server_node")
        node_name = self.get_parameter("managed_node_name").value
        self.get_logger().info(f"Server Node: {node_name}")
        service_change_state_name = f"/{node_name}/change_state"
        service_get_state = f"/{node_name}/get_state"
        self.client_ = self.create_client(ChangeState, service_change_state_name)
        self.get_state_client_ = self.create_client(GetState, service_get_state)
        self.action_client_ = ActionClient(
            self, ControlActions, "uWABA_prototype/Control_Server"
        )
        self.goal_flag = False
        self.trans_cleaned = False

    def change_state(self, transition: Transition):
        self.client_.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def initialization_sequence(self):

        self.transition_ = Transition()

        if self.get_state_service() == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info("Trying to switch to configuring")
            self.transition_.id = Transition.TRANSITION_CONFIGURE
            self.transition_.label = "configure"
            self.change_state(self.transition_)
            self.get_logger().info("Configuring OK, now inactive")

        # Inactive to Active
        if self.get_state_service() == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info("Trying to switch to activating")
            self.transition_.id = Transition.TRANSITION_ACTIVATE
            self.transition_.label = "activate"
            self.change_state(self.transition_)
            self.get_logger().info("Activating OK, now active")
            self.send_goal(self.goal_child_frame_id_, self.goal_request_)
        else:
            self.get_logger().warn(
                "Server not in a state to be initialized, now trying to deactivate it:"
            )
            self.deactivate_and_cleanup()

    def send_goal(self, child_frame_id, request):
        self.action_client_.wait_for_server()
        goal = ControlActions.Goal()
        goal.child_frame_id = child_frame_id
        goal.request = request
        self.action_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)
        self.goal_flag = True

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
        self.get_logger().info(f"Result: {result.result_msg} with Status: {status}")

    def deactivate_and_cleanup(self):
        if self.get_state_service() == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info("Trying to switch to deactivating")
            self.transition_.id = Transition.TRANSITION_DEACTIVATE
            self.transition_.label = "deactivate"
            self.change_state(self.transition_)
            self.get_logger().info("Deactivating OK, now inactive")
        if self.get_state_service() == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info("Trying to switch to cleaning up")
            self.transition_.id = Transition.TRANSITION_CLEANUP
            self.transition_.label = "cleanup"
            self.change_state(self.transition_)
            self.get_logger().info("Cleanup OK, now unconfigured")
            self.trans_cleaned = True
        else:
            self.get_logger().warn("Server not in a state to be deactivated")

    def get_state_service(self):
        self.get_state_client_.wait_for_service()
        get_current_state = GetState.Request()
        future = self.get_state_client_.call_async(get_current_state)
        rclpy.spin_until_future_complete(self, future)
        current_state_id = future.result().current_state.id
        return current_state_id


def main(args=None):
    rclpy.init(args=args)
    node = ControllerManager()
    node.initialization_sequence()
    try:
        if not node.trans_cleaned:
            rclpy.spin(node)
    except KeyboardInterrupt:
        if node.goal_flag:
            node.get_logger().warn("Sending a cancel request")
            node.goal_handle_.cancel_goal_async()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
