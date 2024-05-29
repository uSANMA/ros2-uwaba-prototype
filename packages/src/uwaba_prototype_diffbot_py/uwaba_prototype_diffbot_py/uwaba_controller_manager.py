#!/usr/bin/env python3
import rclpy
import re
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State

from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from uwaba_prototype_interfaces.action import ControlActions
from uwaba_prototype_interfaces.srv import ManagerServices
from uwaba_prototype_interfaces.msg import ServerLog


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
        self.trans_cleaned = False
        self.action_request_service_ = []
        self.logging_msgs_ = ServerLog()
        self.server_log_publisher_ = self.create_publisher(ServerLog, "server_logs", 10)

        self.client_ = self.create_client(
            ChangeState,
            service_change_state_name,
            callback_group=ReentrantCallbackGroup(),
        )

        self.get_state_client_ = self.create_client(
            GetState, service_get_state, callback_group=ReentrantCallbackGroup()
        )

        self.action_client_ = ActionClient(
            self,
            ControlActions,
            "uwaba_prototype/control_server",
            callback_group=ReentrantCallbackGroup(),
        )

        self.goal_service_ = self.create_service(
            ManagerServices,
            f"/{node_name}/goal_service",
            callback=self.goal_service_request_handler,
        )


    def goal_service_request_handler(
        self, request: ManagerServices.Request, response: ManagerServices.Response
    ):
        # Define the regex pattern
        goal_pattern = r"(set_goal)_(\w) = (-?\d+\.\d+)"
        goal_match_simple = re.match(goal_pattern, request.goal_request)
        goal_pattern_mult = r"(set_goal)_(\w)_?(\w)? = (-?\d+\.\d+)\,? ?(-?\d+\.\d+)?"
        goal_match_mult = re.match(goal_pattern_mult, request.goal_request)
        cancel_pattern = r"(cancel)"
        goal_match_cancel = re.match(cancel_pattern, request.goal_request)

        # Match the pattern with the request string
        if goal_match_simple:
            action, axis_x, value_x = goal_match_simple.groups()
            axis_y = ""
            self.get_logger().info(
                f"Received goal request action {action} to axis {axis_x} at {float(value_x)}"
            )
        elif goal_match_mult:
            action, axis_x, axis_y, value_x, value_y = goal_match_mult.groups()
            self.get_logger().info(
                f"Received goal request action {action} to axis {axis_x} at {float(value_x)} and to axis {axis_y} at {float(value_y)}"
            )
        elif goal_match_cancel:
            action = goal_match_cancel.group(1)
            self.get_logger().info(f"Received a {action} request.")

        try:
            if action == "set_goal":
                if axis_x == "x" and axis_y == "":
                    self.action_request_service_ = [float(value_x), 0.0]
                    response.request_info = f"Request to new goal at {axis_x}: {self.action_request_service_} was successful. Sending action to the server..."
                    self.send_goal_from_service(action, self.action_request_service_)
                    return response
                elif axis_x == "x" and axis_y == "y":
                    self.action_request_service_ = [float(value_x), float(value_y)]
                    response.request_info = f"Request to new goal at {axis_x}: {float(value_x)} and at {axis_y}: {float(value_y)} was successful. Sending action to the server..."
                    self.send_goal_from_service(action, self.action_request_service_)
                    return response
                else:
                    self.get_logger().warn(
                        "The goal to this axis is not yet implemented."
                    )
            elif action == "cancel":
                response.request_info = (
                    f"Request to {action} successful. Sending action to the server..."
                )
                self.goal_handle_.cancel_goal_async()
                return response
            else:
                self.get_logger().warn("This goal is not yet implemented.")
        except ValueError as e:
            self.get_logger().error(e)

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

    def send_goal(self, frame_id, request):
        self.action_client_.wait_for_server()
        goal = ControlActions.Goal()
        goal.frame_id = frame_id
        goal.request = request
        self.action_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def send_goal_from_service(self, request, goal_requested):
        self.action_client_.wait_for_server()
        goal = ControlActions.Goal()
        goal.goal_request = goal_requested
        goal.request = request
        goal.frame_id = "new_goal"
        self.action_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_feedback_callback(self, feedback_msg):
        process = feedback_msg.feedback.process
        self.logging_msgs_.logging_msg = process
        self.server_log_publisher_.publish(self.logging_msgs_)

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
            self.get_logger().warn("Aborted")
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
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
