#!/usr/bin/env python3
### SIMPLE LINE SIMULATOR ###
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
from uwaba_prototype_interfaces.msg import EncoderMsg, ServerLog


class ControllerManager(Node):
    def __init__(self):
        self.node_name = "uwaba_controller_manager_node"
        super().__init__(f"{self.node_name}")

        self.declare_parameter("managed_node_name", self.node_name)
        node_name = self.get_parameter("managed_node_name").value
        self.declare_parameter("micro_ros_node_name", "uWABA")
        self.micro_ros_node_name__ = self.get_parameter("micro_ros_node_name").value
        self.declare_parameter("microros_checker_rate", 10.0)
        self.microros_checker_rate__ = self.get_parameter("microros_checker_rate").value

        self.get_logger().info(f"Server Node: {node_name}")
        service_change_state_name = f"/{node_name}/change_state"
        service_get_state = f"/{node_name}/get_state"
        self.action_request_service_ = ""
        # self.logging_msgs_ = ServerLog()
        # self.server_log_publisher_ = self.create_publisher(ServerLog, "server_logs", 10)
        self.activated_uros_checker_ = False
        self.datalog_counter_ = 0

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
            callback_group=ReentrantCallbackGroup(),
        )

    def goal_service_request_handler(
        self, request: ManagerServices.Request, response: ManagerServices.Response
    ):
        # Define the regex pattern
        cancel_pattern = r"(stop)"
        goal_match_cancel = re.match(
            cancel_pattern, request.goal_request, re.IGNORECASE
        )
        continue_pattern = r"(continue)"
        goal_match_continue = re.match(
            continue_pattern, request.goal_request, re.IGNORECASE
        )

        # Match the pattern with the request string
        if goal_match_cancel:
            action = goal_match_cancel.group(1).lower()
            self.get_logger().info(f"Received a {action} request.")

        elif goal_match_continue:
            action = goal_match_continue.group(1).lower()
            self.get_logger().info(f"Received a {action} request.")

        try:
            if action == "stop":
                response.request_info = (
                    f"Request to {action} successful. Sending action to the server..."
                )
                self.send_goal_from_service(action)
                return response

            elif action == "continue":
                response.request_info = (
                    f"Request to {action} successful. Sending action to the server..."
                )
                self.send_goal_from_service(action)
                return response

            else:
                self.get_logger().warn("This goal is not yet implemented.")
        except ValueError as e:
            self.get_logger().error(str(e))

    def change_state(self, transition: Transition):
        self.client_.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = self.client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def initialization_sequence(self):

        self.transition_ = Transition()

        if self.get_state_service() == State.PRIMARY_STATE_UNCONFIGURED:
            self.get_logger().info("Trying to switch to 'configure' state...")
            self.transition_.id = Transition.TRANSITION_CONFIGURE
            self.transition_.label = "configure"
            self.change_state(self.transition_)
            self.get_logger().info("Configuring OK, now state set as inactive.")

        # Inactive to Active
        if self.get_state_service() == State.PRIMARY_STATE_INACTIVE:
            self.get_logger().info("Trying to switch to 'activating' state...")
            self.transition_.id = Transition.TRANSITION_ACTIVATE
            self.transition_.label = "activate"
            self.change_state(self.transition_)
            self.get_logger().info("Activating OK, now state set as active.")
            self.get_logger().info("\033[90;1m Node Test Timer Has Started\033[0m")
            self.microros_checker_timer = self.create_timer(
                (self.microros_checker_rate__),
                self.micro_ros_checker,
                callback_group=ReentrantCallbackGroup(),
            )
            self.activated_uros_checker_ = True
        else:
            self.get_logger().warn(
                "Server not in a state to be set as ACTIVE, now trying to deactivate it..."
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

    def send_goal_from_service(self, request):
        self.action_client_.wait_for_server()
        goal = ControlActions.Goal()
        goal.request = request
        goal.frame_id = "new_goal"
        self.action_client_.send_goal_async(
            goal, feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def goal_feedback_callback(self, feedback_msg):
        process = feedback_msg.feedback.process
        self.logging_msgs_.logging_msg = process
        self.get_logger().info(process)

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("Goal got accepted.")
            self.goal_handle_.get_result_async().add_done_callback(
                self.goal_result_callback
            )
        else:
            self.get_logger().warn("Goal got rejected.")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Success")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().info("Aborted")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Canceled")
        self.get_logger().info(f"Result: {result.result_msg} with Status: {status}")

    def deactivate_and_cleanup(self):
        self.get_logger().info(
            "Sending goal cancel request to server to be sure that no goal is being executed..."
        )
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
            self.get_logger().info(
                "\033[91;1m Node Test Timer Has Been Destroyed\033[0m"
            )
            if self.activated_uros_checker_:
                self.microros_checker_timer.destroy()
        else:
            if self.activated_uros_checker_:
                self.microros_checker_timer.destroy()
            self.get_logger().warn("Server not in a state to be deactivated")

    def get_state_service(self):
        self.get_state_client_.wait_for_service()
        get_current_state = GetState.Request()
        future = self.get_state_client_.call_async(get_current_state)
        rclpy.spin_until_future_complete(self, future)
        current_state_id = future.result().current_state.id
        return current_state_id

    def micro_ros_checker(self):
        # if f"/{self.micro_ros_node_name__}" not in active_nodes:
        #     restart_microcontroller()
        # '_history',
        # '_depth',
        # '_reliability',
        # '_durability',
        # '_lifespan',
        # '_deadline',
        # '_liveliness',
        # '_liveliness_lease_duration',
        # '_avoid_ros_namespace_conventions',
        active_imu_nodes = self.get_publishers_info_by_topic("/micro_imu")
        active_scan_nodes = self.get_publishers_info_by_topic("/micro_laserscan")
        active_encoder_nodes = self.get_publishers_info_by_topic("/micro_encoders")
        if active_imu_nodes and active_encoder_nodes and active_scan_nodes:
            self.datalog_counter_ += 1
            if (self.datalog_counter_ >= 15):
                for info in active_imu_nodes:
                    self.get_logger().info(
                        f"\n\t- \033[95;1mNodes found:\033[0m \033[94m{info.node_name}\033[0m\
                        \n\t- \033[95;1mTopic Type:\033[0m \033[94;4m{info.topic_type}\033[0m\
                        \n\t- \033[95;1mQoS:\033[0m\
                        \n\t\t-- \033[95mHistory:\t\t\t\t\033[0m \033[94m{info.qos_profile.history}\033[0m\
                        \n\t\t-- \033[95mDepth:\t\t\t\t\033[0m \033[94m{info.qos_profile.depth}\033[0m\
                        \n\t\t-- \033[95mReliability:\t\t\t\t\033[0m \033[94m{info.qos_profile.reliability}\033[0m\
                        \n\t\t-- \033[95mDurability:\t\t\t\t\033[0m \033[94m{info.qos_profile.durability}\033[0m\
                        \n\t\t-- \033[95mLifespan:\t\t\t\t\033[0m \033[94m{info.qos_profile.lifespan}\033[0m\
                        \n\t\t-- \033[95mDeadline:\t\t\t\t\033[0m \033[94m{info.qos_profile.deadline}\033[0m\
                        \n\t\t-- \033[95mLiveliness:\t\t\t\t\033[0m \033[94m{info.qos_profile.liveliness}\033[0m\
                        \n\t\t-- \033[95mLiveliness Lase Duration:\t\t\033[0m \033[94m{info.qos_profile.liveliness_lease_duration}\033[0m\
                        \n\t\t-- \033[95mAvoid ROS Namespace Conventions:\t\033[0m \033[94m{info.qos_profile.avoid_ros_namespace_conventions}\033[0m\
                        \n\t- \033[95;1mEndpoint GID:\033[0m \033[94m{info.endpoint_gid}\033[0m\
                        \n\t- \033[95;1mEndpoint Type:\033[0m \033[94m{info.endpoint_type}\033[0m\
                        \n\t- \033[95;1mNode Namespace:\033[0m \033[94m{info.node_namespace}\033[0m\n---"
                    )
                active_imu_nodes = 0

                for info in active_scan_nodes:
                    self.get_logger().info(
                        f"\n---\n\t- \033[95;1mNodes found:\033[0m \033[94m{info.node_name}\033[0m\
                        \n\t- \033[95;1mTopic Type:\033[0m \033[94;4m{info.topic_type}\033[0m\
                        \n\t- \033[95;1mQoS:\033[0m\
                        \n\t\t-- \033[95mHistory:\t\t\t\t\033[0m \033[94m{info.qos_profile.history}\033[0m\
                        \n\t\t-- \033[95mDepth:\t\t\t\t\033[0m \033[94m{info.qos_profile.depth}\033[0m\
                        \n\t\t-- \033[95mReliability:\t\t\t\t\033[0m \033[94m{info.qos_profile.reliability}\033[0m\
                        \n\t\t-- \033[95mDurability:\t\t\t\t\033[0m \033[94m{info.qos_profile.durability}\033[0m\
                        \n\t\t-- \033[95mLifespan:\t\t\t\t\033[0m \033[94m{info.qos_profile.lifespan}\033[0m\
                        \n\t\t-- \033[95mDeadline:\t\t\t\t\033[0m \033[94m{info.qos_profile.deadline}\033[0m\
                        \n\t\t-- \033[95mLiveliness:\t\t\t\t\033[0m \033[94m{info.qos_profile.liveliness}\033[0m\
                        \n\t\t-- \033[95mLiveliness Lase Duration:\t\t\033[0m \033[94m{info.qos_profile.liveliness_lease_duration}\033[0m\
                        \n\t\t-- \033[95mAvoid ROS Namespace Conventions:\t\033[0m \033[94m{info.qos_profile.avoid_ros_namespace_conventions}\033[0m\
                        \n\t- \033[95;1mEndpoint GID:\033[0m \033[94m{info.endpoint_gid}\033[0m\
                        \n\t- \033[95;1mEndpoint Type:\033[0m \033[94m{info.endpoint_type}\033[0m\
                        \n\t- \033[95;1mNode Namespace:\033[0m \033[94m{info.node_namespace}\033[0m\n---"
                    )
                active_scan_nodes = 0

                for info in active_encoder_nodes:
                    self.get_logger().info(
                        f"\n---\n\t- \033[95;1mNodes found:\033[0m \033[94m{info.node_name}\033[0m\
                        \n\t- \033[95;1mTopic Type:\033[0m \033[94;4m{info.topic_type}\033[0m\
                        \n\t- \033[95;1mQoS:\033[0m\
                        \n\t\t-- \033[95mHistory:\t\t\t\t\033[0m \033[94m{info.qos_profile.history}\033[0m\
                        \n\t\t-- \033[95mDepth:\t\t\t\t\033[0m \033[94m{info.qos_profile.depth}\033[0m\
                        \n\t\t-- \033[95mReliability:\t\t\t\t\033[0m \033[94m{info.qos_profile.reliability}\033[0m\
                        \n\t\t-- \033[95mDurability:\t\t\t\t\033[0m \033[94m{info.qos_profile.durability}\033[0m\
                        \n\t\t-- \033[95mLifespan:\t\t\t\t\033[0m \033[94m{info.qos_profile.lifespan}\033[0m\
                        \n\t\t-- \033[95mDeadline:\t\t\t\t\033[0m \033[94m{info.qos_profile.deadline}\033[0m\
                        \n\t\t-- \033[95mLiveliness:\t\t\t\t\033[0m \033[94m{info.qos_profile.liveliness}\033[0m\
                        \n\t\t-- \033[95mLiveliness Lase Duration:\t\t\033[0m \033[94m{info.qos_profile.liveliness_lease_duration}\033[0m\
                        \n\t\t-- \033[95mAvoid ROS Namespace Conventions:\t\033[0m \033[94m{info.qos_profile.avoid_ros_namespace_conventions}\033[0m\
                        \n\t- \033[95;1mEndpoint GID:\033[0m \033[94m{info.endpoint_gid}\033[0m\
                        \n\t- \033[95;1mEndpoint Type:\033[0m \033[94m{info.endpoint_type}\033[0m\
                        \n\t- \033[95;1mNode Namespace:\033[0m \033[94m{info.node_namespace}\033[0m\n---"
                    )
                active_encoder_nodes = 0
                self.datalog_counter_ = 0

        elif not active_scan_nodes or not active_imu_nodes or not active_encoder_nodes:
            self.get_logger().info("\n\033[93;1m---\n\tTopics are idle!!!\n---\033[0m")
        else:
            active_imu_nodes = 0
            active_scan_nodes = 0
            active_encoder_nodes = 0


def main(args=None):
    rclpy.init(args=args)
    node = ControllerManager()
    node.initialization_sequence()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
