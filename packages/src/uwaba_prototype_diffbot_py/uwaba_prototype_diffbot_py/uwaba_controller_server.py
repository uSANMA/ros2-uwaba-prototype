#!/usr/bin/env python3
import rclpy
import time
import threading
import tf_transformations
import numpy as np
from math import sin, cos, pi

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from tf2_ros import TransformBroadcaster, TransformStamped

from uwaba_prototype_interfaces.action import ControlActions

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from uwaba_prototype_diffbot_py.uwaba_controller_manager import ControllerManager


###################################################################################################
#                                                                                                 #
# REMINDER FOR FINAL IMPLEMENTATION: Please remove the get_logger() to avoid delaying the process #
#                                                                                                 #
###################################################################################################


class ControllerServer(LifecycleNode):
    def __init__(self):
        self.lf_node_name_ = "controller_server_node"
        super().__init__(f"{self.lf_node_name_}")
        self.qos_profile_ = QoSProfile(depth=10)
        self.server_activated_ = False
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.cmd_flag_lock_ = threading.Lock()
        self.got_twist_package_ = False
        self.joint_flag_lock_ = threading.Lock()
        self.got_joint_package_ = False
        self.joint_state_broadcaster_ = TransformBroadcaster(self, self.qos_profile_)
        self.covariance_fill_ = np.empty((36,))
        self.cmd_vel_header_stamp_ = None
        self.cmd_vel_header_frame_id_ = None
        self.cmd_vel_linear_ = None
        self.cmd_vel_angular_ = None
        self.joint_state_header_stamp_ = None
        self.joint_state_header_frame_id_ = None
        self.joint_state_names_ = None
        self.joint_state_position_ = None
        self.joint_state_velocity_ = None
        self.joint_state_effort_ = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on configure")  ## REMOVE WHEN DONE
        self.control_server_ = ActionServer(
            self,
            ControlActions,
            "uWABA_prototype/Control_Server",
            goal_callback=self.goal_callback,
            # handle_accepted_callback=self.handle_accepted_callback,          # Won't implement goal queue for now
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        # BEGIN: This is a test publisher
        self.send_cmd_vel_back_ = self.create_publisher(
            TwistStamped, f"{self.lf_node_name_}/cmd_vel", self.qos_profile_
        )
        # END
        self.odom_publisher_ = self.create_publisher(
            Odometry, f"{self.lf_node_name_}/odom", self.qos_profile_
        )
        self.joint_state_publisher_ = self.create_publisher(
            JointState, f"{self.lf_node_name_}/joint_states", self.qos_profile_
        )
        self.get_logger().info("Controller action server has started.")
        # Check whether the frame_id from the wheels of the diffbot are connected and ready to send msgs
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on activate")  ## REMOVE WHEN DONE
        # Here should be created the subscribers
        self.cmd_vel_subscriber = self.create_subscription(
            TwistStamped, "cmd_vel", self.cmd_vel_subscription, self.qos_profile_
        )
        self.uros_joint_state_subscriber = self.create_subscription(
            JointState, "encoder", self.uros_encoder_subscription, self.qos_profile_
        )
        self.server_activated_ = True
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on deactivate")  ## REMOVE WHEN DONE
        self.server_activated_ = False
        # To make sure that the variable is not being accessed at the same time in two different threads
        # is a best practice to lock the thread
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.goal_handle_.abort()
            # self.goal_queue_ = []
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on cleanup ")  ## REMOVE WHEN DONE
        self.control_server_.destroy()
        self.destroy_publisher(self.cmd_vel_subscriber)
        self.destroy_publisher(self.send_cmd_vel_back_)
        self.destroy_publisher(self.odom_publisher_)
        self.destroy_publisher(self.joint_state_publisher_)
        self.destroy_subscription(self.cmd_vel_subscriber)
        self.destroy_subscription(self.uros_joint_state_subscriber)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on shutdown")  ## REMOVE WHEN DONE
        self.control_server_.destroy()
        self.destroy_publisher(self.cmd_vel_subscriber)
        self.destroy_publisher(self.send_cmd_vel_back_)
        self.destroy_publisher(self.odom_publisher_)
        self.destroy_publisher(self.joint_state_publisher_)
        self.destroy_subscription(self.cmd_vel_subscriber)
        self.destroy_subscription(self.uros_joint_state_subscriber)
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().error(
            f"There was an error with state {state}"
        )  ## REMOVE WHEN DONE
        self.control_server_.destroy()
        self.destroy_publisher(self.cmd_vel_subscriber)
        self.destroy_publisher(self.send_cmd_vel_back_)
        self.destroy_publisher(self.odom_publisher_)
        self.destroy_publisher(self.joint_state_publisher_)
        self.destroy_subscription(self.cmd_vel_subscriber)
        self.destroy_subscription(self.uros_joint_state_subscriber)
        return super().on_error(state)

    def goal_callback(self, goal_request: ControlActions.Goal) -> GoalResponse:
        self.get_logger().info("Received a goal.")  ## REMOVE WHEN DONE
        if not self.server_activated_:
            self.get_logger().warn("Server not yet activated.")
            return GoalResponse.REJECT

        # Validate goal request, again with locking the thread so the same
        # variable is not being accessed at the same time
        with self.goal_lock_:
            # Policy: Goal preemption (must be after some goal are already valid thus preempting)
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().warn(
                    "A goal is already active, aborting new goal."
                )  ## REMOVE WHEN DONE
                self.goal_handle_.abort()
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().warn(
            f"Received a cancel request, canceling with status {goal_handle.status}"
        )  ## REMOVE WHEN DONE
        self.goal_handle_.abort()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> ControlActions:
        self.get_logger().info("Executing goal")
        # Locking the variable so it doesn't bring issues whilst several threads are being
        # used and prevent the variable to be used simultaneously by different threads
        with self.goal_lock_:
            # Set goal_handle as class attribute so it can be used outside of this callback
            self.goal_handle_ = goal_handle

        # Implement new interfaces for dealing with goals, response and feedbacks
        # I need to aim more in a goal toward a movement rather than labeling the actions itself
        # Also feedback should return the robot's position (last implementation since it will
        # use odometry, joint states and cmd_vel)

        request = goal_handle.request.request
        self.get_logger().info(f"Request: {request}")
        child_frame_id = goal_handle.request.child_frame_id
        self.get_logger().info(f"Child_Frame_Id: {child_frame_id}")

        result = ControlActions.Result()  # instance the result object
        feedback = ControlActions.Feedback()  # instance the feedback object

        twist_msg = TwistStamped()
        odom_msg = Odometry()

        twist_msg.header.frame_id = f"{self.lf_node_name_}/cmd_vel"
        odom_msg.child_frame_id = child_frame_id
        odom_msg.header.frame_id = "odom"

        while rclpy.ok():
            if not goal_handle.is_active:
                result.result_msg = "Preempted by another goal, or node deactivated."
                return result

            if goal_handle.is_cancel_requested:
                result.result_msg = "Goal was cancel requested"
                return result

            with self.cmd_flag_lock_:
                acquired = self.joint_flag_lock_.acquire(timeout=1/15)
                if acquired:
                    try:
                        if request == "transform" and self.got_twist_package_ and self.got_joint_package_:
                            self.got_twist_package_ = False
                            self.got_joint_package_ = False
                            twist_msg.header.stamp = self.cmd_vel_header_stamp_
                            twist_msg.twist.linear = self.cmd_vel_linear_
                            twist_msg.twist.angular = self.cmd_vel_angular_

                            # BEGIN:
                            # Change these values so that the pose is estimated by the joint_states
                            # alongside with cmd_vel
                            odom_msg.header.stamp = self.get_clock().now().to_msg()
                            # BEGIN: The values below must be given from the joint_state subscriber function callback
                            (
                                odom_msg.pose.pose.position.x,
                                odom_msg.pose.pose.position.y,
                                odom_msg.pose.pose.position.z,
                            ) = (0.0, 3.0, 0.0)
                            (
                                odom_msg.pose.pose.orientation.x,
                                odom_msg.pose.pose.orientation.y,
                                odom_msg.pose.pose.orientation.z,
                                odom_msg.pose.pose.orientation.w,
                            ) = tf_transformations.quaternion_from_euler(0.0, 0.0, pi / 2)
                            # END
                            odom_msg.twist.twist = twist_msg.twist
                            odom_msg.pose.covariance = self.covariance_fill_
                            odom_msg.twist.covariance = self.covariance_fill_
                            # END

                            self.send_cmd_vel_back_.publish(twist_msg)
                            self.odom_publisher_.publish(odom_msg)
                    finally:
                        self.joint_flag_lock_.release()
                else:
                    self.get_logger().warn("Joint State flag wasn't acquired properly.")
                
            if request == "empty" or request is None or request == "":
                time.sleep(1.0)
                feedback.process = "Goal request variable is empty."
                goal_handle.publish_feedback(feedback)

        # Implement code for executing the robot movement (joint state publisher with tfs)
        # Don't forget about odometry and so on and so forth (actually can't since the robot
        # only moves in relation to the odom frame)
        result.result_msg = "Conversions stopped and goal handled"
        goal_handle.succeed()
        # Won't implement a queue for now since, at first, the robot may only execute one goal at time
        # self.next_in_queue()
        return result

    def cmd_vel_subscription(self, twist_msgs: TwistStamped):
        self.cmd_vel_header_stamp_ = twist_msgs.header.stamp
        self.cmd_vel_header_frame_id_ = twist_msgs.header.frame_id
        self.cmd_vel_linear_ = twist_msgs.twist.linear
        self.cmd_vel_angular_ = twist_msgs.twist.angular
        with self.cmd_flag_lock_:
            self.got_twist_package_ = True

    def uros_encoder_subscription(self, joint_msg: JointState):
        self.joint_state_header_stamp_ = joint_msg.header.stamp
        self.joint_state_header_frame_id_ = joint_msg.header.frame_id
        self.joint_state_names_ = [joint_msg.name]
        self.joint_state_position_ = [joint_msg.position]
        self.joint_state_velocity_ = [joint_msg.velocity]
        self.joint_state_effort_ = [joint_msg.effort]
        with self.joint_flag_lock_:
            self.got_joint_package_ = True

    ############################### YET TO BE IMPLEMENTED ###############################

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        # Queue will be implemented when we use Nav2 for sending the goals to the simple api fw
        with self.goal_lock_:
            if self.goal_handle_ is not None:
                self.goal_queue_.append(goal_handle)
            else:
                goal_handle.execute()

    def next_in_queue(self):
        # Queue will be implemented when we use Nav2 for sending the goals to the simple api fw
        with self.goal_lock_:
            if len(self.goal_queue_) > 0:
                self.goal_queue_.pop(0).execute()
            else:
                self.goal_handle_ = None

    #####################################################################################


def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
