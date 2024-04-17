#!/usr/bin/env python3
import rclpy
import time
import threading

from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

from tf2_ros import TransformBroadcaster, TransformStamped

from uwaba_prototype_interfaces.action import ControlActions

from geometry_msgs.msg import TwistStamped


class ControllerServer(Node):
    def __init__(self):
        super().__init__("controller_server_node")
        self.qos_profile_ = QoSProfile(depth=10)
        # Action Server Params and init
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.goal_queue_ = []
        self.count_until_server_ = ActionServer(
            self,
            ControlActions,
            "uWABA/Control_Server",
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Controller lifecycle manager has started.")

        # Lifecycle manager client params and init
        self.declare_parameter("managed_node_name", rclpy.Parameter.Type.STRING)
        node_name = self.get_parameter("managed_node_name").value
        service_change_state_name = "/" + node_name + "/change_state"
        service_get_state_name = "/" + node_name + "/get_state"
        self.client_change_state = self.create_client(
            ChangeState, service_change_state_name
        )
        self.client_get_state = self.create_client(GetState, service_get_state_name)
        self.get_logger().info("Controller server has started.")

    def goal_callback(self, goal_request: ControlActions.Goal):
        self.get_logger().info("Received a goal.")
        request = (goal_request.request_id, goal_request.request_label)
        # Policy: Refuse new goal if there's an ongoing goal
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active :
        #         self.get_logger().error(
        #             "A goal is already active, rejecting new goal..."
        #         )
        #         return GoalResponse.REJECT

        # Validate goal request
        self.get_logger().info(
            "Accepting goal: " + f"id:{request[0]} and label:{request[1]}"
        )
        return GoalResponse.ACCEPT

        # Policy: Goal preemption (must be after some goal are already valid thus preempting)
        # with self.goal_lock_:
        #     if self.goal_handle_ is not None and self.goal_handle_.is_active:
        #         self.get_logger().warn(
        #             "Aborting current goal and accepting a ne w one..."
        #         )
        #         self.goal_handle_.abort()

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().warn(
            f"Received a cancel request, canceling with status {goal_handle.status}"
        )
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        # Locking the variable so it doesn't bring issues whilst several threads are being
        # used and prevent the variable to be used simultaneously by different threads
        with self.goal_lock_:
            # Set goal_handle as class attribute so it can be used outside of this callback
            self.goal_handle_ = goal_handle

        # Got request from goal
        request_id = goal_handle.request.request_id
        request_label = goal_handle.request.request_label

        result = ControlActions.Result()  # instance the result object
        feedback = ControlActions.Feedback()  # instance the feedback object

        timeout_counter = 0.0

        # Execute the action
        self.get_logger().info("Executing goal...")
        counter = 0
        while timeout_counter <= 10:
            if not goal_handle.is_active:
                result.status_id = request_id
                result.action_msg = request_label
                self.next_in_queue()
                return result
            if goal_handle.is_cancel_requested:
                self.get_logger().info("Goal is being canceled.")
                goal_handle.canceled()
                self.get_logger().info("Goal canceled.")
                result.status_id = -1
                result.action_msg = "cancelled"
                self.next_in_queue()
                return result
            self.get_logger().info("Request id: " + str(request_id))
            feedback.process_id = request_id
            goal_handle.publish_feedback(feedback)
            timeout_counter += 1
            time.sleep(1.25)

        # Once done counting, set goal final state...
        goal_handle.succeed()

        # ...and send the result
        result.status_id = request_id
        result.action_msg = "finished goal"
        self.next_in_queue()
        return result

    def next_in_queue(self):
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None

    def get_state(self):
        self.client_get_state.wait_for_service()
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
        self.change_state(transition)
        id, label = self.get_state()
        self.get_logger().warn("id: " + str(id) + " label: " + label)

        self.get_logger().info("State changed to configured!")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
