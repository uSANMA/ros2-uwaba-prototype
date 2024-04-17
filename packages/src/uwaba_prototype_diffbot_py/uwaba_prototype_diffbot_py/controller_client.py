#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from uwaba_prototype_interfaces.action import ControlActions

# # Goal
# uint8 request_id
# string request_label
# ---
# # Result
# uint8 status_id
# string action_msg
# ---
# # Feedback
# uint8 process_id


class ControllerClient(Node):
    def __init__(self):
        super().__init__("controller_client_node")
        self.declare_parameter("request_id", 0)
        self.request_id_ = self.get_parameter("request_id").value
        self.declare_parameter("request_label", "empty")
        self.request_label_ = self.get_parameter("request_label").value
        self.declare_parameter("cancel_request", False)
        self.request_cancel_ = self.get_parameter("cancel_request").value

        self.count_until_client_ = ActionClient(
            self, ControlActions, "uWABA/Control_Server"
        )
        self.get_logger().info("Controller Client has started.")

    def send_goal(self, request_id, request_label):
        # Wait for the server
        self.count_until_client_.wait_for_server()

        # Create goal
        goal = ControlActions.Goal()
        goal.request_id = request_id
        goal.request_label = request_label

        # Send the goal
        self.get_logger().info("Sending goal.")
        self.count_until_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal was accepted.")
            self.goal_handle_.get_result_async().add_done_callback(
                self.goal_result_callback
            )
            if self.request_cancel_:
                self.timer_ = self.create_timer(5.0, self.cancel_goal)
                self.get_logger().warn(
                    "The timer to cancel the goal has been set to 5.0 seconds."
                )
        else:
            self.get_logger().warn("Goal was rejected.")

    def cancel_goal(self):
        self.get_logger().info("Sending a cancel request")
        self.goal_handle_.cancel_goal_async()
        self.timer_.cancel()

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal Succeeded with status {status}")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error(f"Goal Aborted with status {status}")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f"Goal Canceled with status {status}")
        self.get_logger().info(
            f"Result id: {result.status_id} and Result msg: {result.action_msg}"
        )

    def goal_feedback_callback(self, feedback_msg):
        number = feedback_msg.feedback.process_id
        self.get_logger().info(
            "Controller Server feedback: " + "process_id: " + str(number)
        )


def main(args=None):
    rclpy.init(args=args)
    node = ControllerClient()
    node.send_goal(node.request_id_, node.request_label_)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
