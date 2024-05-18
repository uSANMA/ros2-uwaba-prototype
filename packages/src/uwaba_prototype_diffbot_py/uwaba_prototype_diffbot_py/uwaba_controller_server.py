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

# from uwaba_prototype_interfaces.msg import MotorVels

from geometry_msgs.msg import TwistStamped, Quaternion
from sensor_msgs.msg import JointState, Imu, LaserScan, Temperature
from nav_msgs.msg import Odometry

# from uwaba_prototype_diffbot_py.uwaba_controller_manager import ControllerManager


###################################################################################################
#                                                                                                 #
# REMINDER FOR FINAL IMPLEMENTATION: Please remove all get_logger() to avoid delaying the process #
#                                                                                                 #
###################################################################################################


class ControllerServer(LifecycleNode):
    def __init__(self):
        self.lf_node_name_ = "uwaba_controller_server_node"
        super().__init__(f"{self.lf_node_name_}")
        # Constructor parameters
        self.qos_profile_ = QoSProfile(depth=10)
        self.server_activated_ = False
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.cmd_flag_lock_ = threading.Lock()
        self.got_twist_package_ = None
        self.encoder_flag_lock_ = threading.Lock()
        self.got_encoder_package_ = None
        self.imu_flag_lock_ = threading.Lock()
        self.got_imu_package_ = None
        self.lidar_flag_lock_ = threading.Lock()
        self.got_lidar_package_ = None
        self.temp_flag_lock_ = threading.Lock()
        self.got_temp_package_ = None
        self.timing_lock_ = threading.Lock()
        self.got_timing_ = None
        self.msgs_began_ = False
        self.goal_arrived_ = False
        self.joint_state_broadcaster_ = TransformBroadcaster(self, self.qos_profile_)
        self.joint_state_left_wheel_ = 0.0
        self.joint_state_right_wheel_ = 0.0
        self.changed_velocity_flag_ = False
        self.slow_down_inc_ = 1

        # Subscription parameters
        self.covariance_fill_ = np.zeros((36,))
        self.cmd_vel_header_stamp_ = None
        self.cmd_vel_header_frame_id_ = None
        self.cmd_vel_linear_ = None
        self.cmd_vel_angular_ = None
        self.motor_velocity_header_stamp_ = None
        self.motor_velocity_header_frame_id_ = None
        self.motor_velocity_name_ = None
        self.motor_velocity_encoders_ = None

        # ROS 2 parameters
        self.declare_parameter("wheels_separation", 0.0)
        self.wheels_separation__ = self.get_parameter("wheels_separation").value
        self.declare_parameter("wheel_radius", 0.0)
        self.wheel_radius__ = self.get_parameter("wheel_radius").value
        self.declare_parameter("transform_rate", 1.0)
        self.transform_rate__ = self.get_parameter("transform_rate").value
        self.declare_parameter("tune_velocity", 1.0)
        self.tune_vel__ = self.get_parameter("tune_velocity").value
        self.declare_parameter("goal_position_x", 0.0)
        self.goal_pos__ = self.get_parameter("goal_position_x").value
        self.declare_parameter("thread_timing", 30.0)
        self.thread_timing__ = self.get_parameter("thread_timing").value
        self.declare_parameter("thread_timeout_tune", 2.0)
        self.carlinhos_pro__ = self.get_parameter("thread_timeout_tune").value
        self.declare_parameter("max_linear_velocity", 1.0)
        self.max_linear_velocity__ = self.get_parameter("max_linear_velocity").value

        self.declare_parameter("uros_encoder_topic", "micro_encoders")
        self.uros_encoder_topic__ = self.get_parameter("uros_encoder_topic").value
        self.declare_parameter("uros_imu_topic", "micro_imu")
        self.uros_imu_topic__ = self.get_parameter("uros_imu_topic").value
        self.declare_parameter("uros_temperature_topic", "micro_temp")
        self.uros_temperature_topic__ = self.get_parameter(
            "uros_temperature_topic"
        ).value
        self.declare_parameter("uros_lidar_topic", "micro_laser")
        self.uros_lidar_topic__ = self.get_parameter("uros_lidar_topic").value
        self.declare_parameter("odom_topic", "odom")
        self.odom_topic__ = self.get_parameter("odom_topic").value
        self.declare_parameter("joint_states_topic", "joint_states")
        self.joint_state_topic__ = self.get_parameter("joint_states_topic").value
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.cmd_vel_topic__ = self.get_parameter("cmd_vel_topic").value

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
        self.send_cmd_vel_back_ = self.create_publisher(
            TwistStamped, f"{self.lf_node_name_}/cmd_vel", self.qos_profile_
        )
        self.odom_publisher_ = self.create_publisher(
            Odometry, f"{self.odom_topic__}", self.qos_profile_
        )
        self.joint_state_publisher_ = self.create_publisher(
            JointState, f"{self.joint_state_topic__}", self.qos_profile_
        )
        self.get_logger().info("Controller action server has started.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on activate")  ## REMOVE WHEN DONE
        self.cmd_vel_subscriber = self.create_subscription(
            TwistStamped,
            f"{self.cmd_vel_topic__}",
            self.cmd_vel_subscription,
            self.qos_profile_,
        )
        self.uros_encoder_state_subscriber = self.create_subscription(
            JointState,
            f"{self.uros_encoder_topic__}",
            self.uros_encoder_subscription,
            self.qos_profile_,
        )
        self.uros_lidar_subscriber = self.create_subscription(
            LaserScan,
            f"{self.uros_lidar_topic__}",
            self.uros_laser_subscription,
            self.qos_profile_,
        )
        self.uros_imu_subscriber = self.create_subscription(
            Imu,
            f"{self.uros_imu_topic__}",
            self.uros_imu_subscription,
            self.qos_profile_,
        )
        self.uros_temp_subscriber = self.create_subscription(
            Temperature,
            f"{self.uros_temperature_topic__}",
            self.uros_temp_subscription,
            self.qos_profile_,
        )
        self.time_rate = self.create_timer(
            (1.0 / self.transform_rate__), self.timing_function
        )
        self.get_logger().info(
            f"\nActivated successfully with params:\nWheel Separation: {self.wheels_separation__}\nWheel Radius: {self.wheel_radius__}\nTransform Rate: {self.transform_rate__}"
        )
        self.server_activated_ = True
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on deactivate")  ## REMOVE WHEN DONE
        self.server_activated_ = False
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
        self.destroy_subscription(self.uros_encoder_state_subscriber)
        self.destroy_subscription(self.uros_lidar_subscriber)
        self.destroy_subscription(self.uros_imu_subscriber)
        self.destroy_subscription(self.uros_temp_subscriber)
        self.time_rate.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on shutdown")  ## REMOVE WHEN DONE
        self.control_server_.destroy()
        self.destroy_publisher(self.cmd_vel_subscriber)
        self.destroy_publisher(self.send_cmd_vel_back_)
        self.destroy_publisher(self.odom_publisher_)
        self.destroy_publisher(self.joint_state_publisher_)
        self.destroy_subscription(self.cmd_vel_subscriber)
        self.destroy_subscription(self.uros_encoder_state_subscriber)
        self.destroy_subscription(self.uros_lidar_subscriber)
        self.destroy_subscription(self.uros_imu_subscriber)
        self.destroy_subscription(self.uros_temp_subscriber)
        self.time_rate.destroy()
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
        self.destroy_subscription(self.uros_encoder_state_subscriber)
        self.destroy_subscription(self.uros_lidar_subscriber)
        self.destroy_subscription(self.uros_imu_subscriber)
        self.destroy_subscription(self.uros_temp_subscriber)
        self.time_rate.destroy()
        return super().on_error(state)

    def goal_callback(self, goal_request: ControlActions.Goal) -> GoalResponse:
        self.get_logger().info("Received a goal.")  ## REMOVE WHEN DONE
        if not self.server_activated_:
            self.get_logger().warn("Server not yet activated.")
            return GoalResponse.REJECT
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
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        request = goal_handle.request.request
        self.get_logger().info(f"Request: {request}")
        child_frame_id = goal_handle.request.child_frame_id
        self.get_logger().info(f"Child_Frame_Id: {child_frame_id}")

        result = ControlActions.Result()  # instance the result object
        feedback = ControlActions.Feedback()  # instance the feedback object

        joint_state = JointState()
        joint_state.name = ["Left_sprocket_base_joint", "Right_sprocket_base_joint"]
        joint_state.header.frame_id = "wheels_states"

        orientation = Quaternion()

        odom_trans = TransformStamped()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "base_footprint"

        twist_msg = TwistStamped()

        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"

        count_starting = 0
        total_elapsed_time = 0.0

        # Odometry starting point
        x = 0.0
        y = 0.0
        th = 0.0
        timeout_locks = self.carlinhos_pro__ * float(1 / self.thread_timing__)

        while rclpy.ok():
            if not goal_handle.is_active:
                result.result_msg = "Preempted by another goal, or node deactivated."
                return result

            if goal_handle.is_cancel_requested:
                result.result_msg = "Goal was cancel requested"
                return result

            if self.msgs_began_ and count_starting < 1:
                count_starting += 1
                starting_time = self.get_clock().now()

            with self.timing_lock_:
                acquired_cmd = self.cmd_flag_lock_.acquire(timeout=timeout_locks)
                acquired_encoder = self.encoder_flag_lock_.acquire(
                    timeout=timeout_locks
                )
                acquired_imu = self.imu_flag_lock_.acquire(timeout=timeout_locks)
                acquired_lidar = self.lidar_flag_lock_.acquire(timeout=timeout_locks)
                acquired_temp = self.temp_flag_lock_.acquire(timeout=timeout_locks)

                if (
                    acquired_cmd
                    and acquired_encoder
                    and acquired_imu
                    and acquired_lidar
                    and acquired_temp
                ):
                    # self.get_logger().info(f"Flags set:\nEncoder:\t{self.got_encoder_package_}\nTwist:\t{self.got_twist_package_}\nIMU:\t{self.got_imu_package_}\nLidar:\t{self.got_lidar_package_}\nTemp:\t{self.got_temp_package_}\nTiming:\t{self.got_timing_}\n")
                    try:
                        if (
                            request == "transform"
                            and self.got_encoder_package_
                            and self.got_twist_package_
                            and self.got_imu_package_
                            and self.got_lidar_package_
                            and self.got_temp_package_
                        ):
                            if self.msgs_began_:
                                current_time = self.get_clock().now()
                                dt = current_time - starting_time
                                dt = dt.to_msg().sec + dt.to_msg().nanosec / 1e9

                                right_motor_vel, left_motor_vel = (
                                    self.motor_velocity_encoders_
                                )

                                if (
                                    dt > 0
                                    and right_motor_vel > 0.0
                                    or left_motor_vel > 0.0
                                    and not self.goal_arrived_
                                    or self.goal_arrived_ is None
                                ):
                                    if self.changed_velocity_flag_:
                                        vx = self.tune_vel__ * (
                                            (right_motor_vel + left_motor_vel) / 2
                                        )
                                        vy = 0.0
                                        vth = (
                                            self.tune_vel__
                                            * (right_motor_vel - left_motor_vel)
                                            / self.wheels_separation__
                                        )
                                        self.changed_velocity_flag_ = False
                                    else:
                                        vx = (right_motor_vel + left_motor_vel) / 2
                                        vy = 0.0
                                        vth = (
                                            right_motor_vel - left_motor_vel
                                        ) / self.wheels_separation__

                                    delta_x = float((vx * cos(th) - vy * sin(th)) * dt)
                                    delta_y = float((vx * sin(th) + vy * cos(th)) * dt)
                                    delta_th = float(vth * dt)

                                    x += delta_x
                                    y += delta_y
                                    th += delta_th

                                    # BEGIN: Just a test so that we can go to some length to test the encoder and cmd_vels
                                    if self.goal_pos__ > 0:
                                        if (
                                            self.slow_down_inc_ == 1
                                            and x >= (0.45 * self.goal_pos__)
                                        ):
                                            self.tune_vel__ = 0.15
                                            self.slow_down_inc_ += 1
                                            self.changed_velocity_flag_ = True
                                        elif self.slow_down_inc_ == 2 and x >= (
                                            0.85 * self.goal_pos__
                                        ):
                                            self.tune_vel__ = 0.5
                                            self.slow_down_inc_ += 1
                                            self.changed_velocity_flag_ = True
                                    # END

                                    (
                                        orientation.x,
                                        orientation.y,
                                        orientation.z,
                                        orientation.w,
                                    ) = tf_transformations.quaternion_from_euler(
                                        0.0, 0.0, th
                                    )

                                    joint_state.header.stamp = current_time.to_msg()
                                    joint_state.position = [x, x]
                                    joint_state.velocity = [vx, vx]

                                    odom_msg.header.stamp = current_time.to_msg()
                                    odom_msg.pose.pose.position.x = x
                                    odom_msg.pose.pose.position.y = y
                                    odom_msg.pose.pose.position.z = 0.0
                                    odom_msg.twist.twist.linear.x = vx
                                    odom_msg.twist.twist.linear.y = 0.0
                                    odom_msg.twist.twist.linear.z = 0.0
                                    odom_msg.twist.twist.angular.x = 0.0
                                    odom_msg.twist.twist.angular.y = 0.0
                                    odom_msg.twist.twist.angular.z = vth
                                    odom_msg.pose.pose.orientation = orientation

                                    odom_trans.header.stamp = current_time.to_msg()
                                    odom_trans.transform.translation.x = x
                                    odom_trans.transform.translation.y = y
                                    odom_trans.transform.translation.z = 0.0
                                    odom_trans.transform.rotation = orientation

                                    twist_msg.header.stamp = current_time.to_msg()
                                    twist_msg.header.frame_id = (
                                        self.cmd_vel_header_frame_id_
                                    )
                                    twist_msg.twist.linear.x = vx
                                    twist_msg.twist.linear.y = 0.0
                                    twist_msg.twist.linear.z = 0.0
                                    twist_msg.twist.angular.x = 0.0
                                    twist_msg.twist.angular.y = 0.0
                                    twist_msg.twist.angular.z = vth

                                    if x >= self.goal_pos__:
                                        self.goal_arrived_ = True
                                        self.tune_vel__ = 0.0
                                        result.result_msg = f"Arrived at the set destination: x= {x}, y= {y}, theta= {th} in {total_elapsed_time} seconds"
                                        twist_msg.twist.linear.x = 0.0
                                        twist_msg.twist.angular.z = 0.0
                                        odom_msg.twist.twist.linear.x = 0.0
                                        odom_msg.twist.twist.angular.z = 0.0
                                        self.send_cmd_vel_back_.publish(twist_msg)
                                        self.odom_publisher_.publish(odom_msg)
                                        self.joint_state_publisher_.publish(joint_state)
                                        self.joint_state_broadcaster_.sendTransform(
                                            odom_trans
                                        )
                                        feedback.process = f"\n - Seconds elapsed:\t{dt}\n - Total Elapsed Time:\t{total_elapsed_time}\n - x:\t\t\t{x}\n - y:\t\t\t{y}\n - th:\t\t\t{th}\n - Velocities:\n - x:\t\t\t{twist_msg.twist.linear.x}\n - theta:\t\t\t{twist_msg.twist.angular.z}\n - Variations:\n - delta_x:\t\t{delta_x}\n - delta_y:\t\t{delta_y}\n - delta_theta:\t\t{delta_th}"
                                        goal_handle.publish_feedback(feedback)
                                        goal_handle.succeed()

                                        self.got_timing_ = False
                                        self.got_encoder_package_ = False
                                        self.got_twist_package_ = False
                                        self.got_imu_package_ = False
                                        self.got_lidar_package_ = False
                                        self.got_temp_package_ = False

                                        return result

                                    self.send_cmd_vel_back_.publish(twist_msg)
                                    self.odom_publisher_.publish(odom_msg)
                                    self.joint_state_publisher_.publish(joint_state)
                                    self.joint_state_broadcaster_.sendTransform(
                                        odom_trans
                                    )

                                    starting_time = current_time
                                    # total_elapsed_time += current_time.to_msg().sec + current_time.to_msg().nanosec/1e9
                                    total_elapsed_time += dt
                                    starting_time = current_time

                                    feedback.process = f"\n - Seconds elapsed:\t{dt}\n - Total Elapsed Time:\t{total_elapsed_time}\n - x:\t\t\t{x}\n - y:\t\t\t{y}\n - th:\t\t\t{th}\n - Velocities:\n - x:\t\t\t{vx}\n - theta:\t\t\t{vth}\n - Variations:\n - delta_x:\t\t{delta_x}\n - delta_y:\t\t{delta_y}\n - delta_theta:\t\t{delta_th}"
                                    goal_handle.publish_feedback(feedback)

                                    self.got_timing_ = False
                                    self.got_encoder_package_ = False
                                    self.got_twist_package_ = False
                                    self.got_imu_package_ = False
                                    self.got_lidar_package_ = False
                                    self.got_temp_package_ = False
                                else:
                                    if (
                                        right_motor_vel == 0.0
                                        and left_motor_vel == 0.0
                                        and not self.goal_arrived_
                                        or self.goal_arrived_ is None
                                    ):
                                        (
                                            orientation.x,
                                            orientation.y,
                                            orientation.z,
                                            orientation.w,
                                        ) = tf_transformations.quaternion_from_euler(
                                            0.0, 0.0, 0.0
                                        )
                                        twist_msg.twist.linear.x = (
                                            self.max_linear_velocity__
                                        )
                                        joint_state.header.stamp = current_time.to_msg()
                                        joint_state.position = [0.0, 0.0]
                                        joint_state.velocity = [
                                            self.max_linear_velocity__,
                                            self.max_linear_velocity__,
                                        ]

                                        odom_msg.header.stamp = current_time.to_msg()
                                        odom_msg.pose.pose.position.x = 0.0
                                        odom_msg.pose.pose.position.y = 0.0
                                        odom_msg.pose.pose.position.z = 0.0
                                        odom_msg.twist.twist.linear.x = (
                                            self.max_linear_velocity__
                                        )
                                        odom_msg.twist.twist.linear.y = 0.0
                                        odom_msg.twist.twist.linear.z = 0.0
                                        odom_msg.twist.twist.angular.x = 0.0
                                        odom_msg.twist.twist.angular.y = 0.0
                                        odom_msg.twist.twist.angular.z = 0.0
                                        odom_msg.pose.pose.orientation = orientation

                                        odom_trans.header.stamp = current_time.to_msg()
                                        odom_trans.transform.translation.x = 0.0
                                        odom_trans.transform.translation.y = 0.0
                                        odom_trans.transform.translation.z = 0.0
                                        odom_trans.transform.rotation = orientation

                                        twist_msg.header.stamp = current_time.to_msg()
                                        twist_msg.header.frame_id = (
                                            self.cmd_vel_header_frame_id_
                                        )
                                        twist_msg.twist.linear.x = (
                                            self.max_linear_velocity__
                                        )
                                        twist_msg.twist.linear.y = 0.0
                                        twist_msg.twist.linear.z = 0.0
                                        twist_msg.twist.angular.x = 0.0
                                        twist_msg.twist.angular.y = 0.0
                                        twist_msg.twist.angular.z = 0.0

                                        self.send_cmd_vel_back_.publish(twist_msg)
                                        self.odom_publisher_.publish(odom_msg)
                                        self.joint_state_publisher_.publish(joint_state)
                                        self.joint_state_broadcaster_.sendTransform(
                                            odom_trans
                                        )

                                        self.got_timing_ = False
                                        self.got_encoder_package_ = False
                                        self.got_twist_package_ = False
                                        self.got_imu_package_ = False
                                        self.got_lidar_package_ = False
                                        self.got_temp_package_ = False
                            else:
                                self.msgs_began_ = True
                                self.got_timing_ = False
                                self.got_encoder_package_ = False
                                self.got_twist_package_ = False
                                self.got_imu_package_ = False
                                self.got_lidar_package_ = False
                                self.got_temp_package_ = False

                    finally:
                        self.cmd_flag_lock_.release()
                        self.encoder_flag_lock_.release()
                        self.imu_flag_lock_.release()
                        self.lidar_flag_lock_.release()
                        self.temp_flag_lock_.release()
                else:
                    feedback.process = f"Some flags timed-out. Time out current time (s): {timeout_locks}\nFlags set:\nEncoder:\t{self.got_encoder_package_}\nTwist:\t{self.got_twist_package_}\nIMU:\t{self.got_imu_package_}\nLidar:\t{self.got_lidar_package_}\nTemp:\t{self.got_temp_package_}\nTiming:\t{self.got_timing_}\nLock_Encoder:\t{self.encoder_flag_lock_.locked()}\nLock_Twist:\t{self.cmd_flag_lock_.locked()}\nLock_IMU:\t\t{self.imu_flag_lock_.locked()}\nLock_Lidar:\t{self.lidar_flag_lock_.locked()}\nLock_Temp:\t\t{self.temp_flag_lock_.locked()}\nTiming_Lock:\t{self.timing_lock_.locked()}"
                    goal_handle.publish_feedback(feedback)

            if request == "empty" or request is None or request == "":
                time.sleep(1.0)
                feedback.process = "Goal request variable is empty."
                goal_handle.publish_feedback(feedback)

    def cmd_vel_subscription(self, twist_msgs: TwistStamped):
        self.cmd_vel_header_stamp_ = twist_msgs.header.stamp
        self.cmd_vel_header_frame_id_ = twist_msgs.header.frame_id
        self.cmd_vel_linear_ = twist_msgs.twist.linear
        self.cmd_vel_angular_ = twist_msgs.twist.angular
        if not self.got_twist_package_ or self.got_twist_package_ is None:
            with self.cmd_flag_lock_:
                self.got_twist_package_ = True

    def uros_encoder_subscription(self, motor_vels: JointState):
        self.motor_velocity_header_stamp_ = motor_vels.header.stamp
        self.motor_velocity_header_frame_id_ = motor_vels.header.frame_id
        self.motor_velocity_name_ = motor_vels.name
        self.motor_velocity_encoders_ = motor_vels.velocity
        if not self.got_encoder_package_ or self.got_encoder_package_ is None:
            with self.encoder_flag_lock_:
                self.got_encoder_package_ = True

    def uros_laser_subscription(self, laser_msgs: LaserScan):
        if not self.got_lidar_package_ or self.got_lidar_package_ is None:
            with self.lidar_flag_lock_:
                self.got_lidar_package_ = True

    def uros_imu_subscription(self, imu_msgs: Imu):
        if not self.got_imu_package_ or self.got_imu_package_ is None:
            with self.imu_flag_lock_:
                self.got_imu_package_ = True

    def uros_temp_subscription(self, temp_msgs: Temperature):
        if not self.got_temp_package_ or self.got_temp_package_ is None:
            with self.temp_flag_lock_:
                self.got_temp_package_ = True

    def timing_function(self):
        if not self.got_timing_ or self.got_timing_ is None:
            with self.timing_lock_:
                self.got_timing_ = True

    ############################### YET TO BE IMPLEMENTED ###############################

    # def timeout_subscription():
    #     with self.goal_lock_:
    #         # Set goal_handle as class attribute so it can be used outside of this callback
    #         self.goal_handle_ = goal_handle
    #     sub_time_out += 1
    #     if sub_time_out == 30:
    #         feedback.process = "Subscription timed out. Aborting current goal."
    #         goal_handle.publish_feedback(feedback)
    #         result.result_msg = "Goal finished due to subscription timeout"
    #         sub_time_out = 0
    #         starting_time = self.get_clock().now()
    #         x = 0.0
    #         y = 0.0
    #         th = 0.0
    #         dt = 0.0
    #         return result

    # def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
    #     # Queue will be implemented when we use Nav2 for sending the goals to the simple api fw
    #     with self.goal_lock_:
    #         if self.goal_handle_ is not None:
    #             self.goal_queue_.append(goal_handle)
    #         else:
    #             goal_handle.execute()

    # def next_in_queue(self):
    #     # Queue will be implemented when we use Nav2 for sending the goals to the simple api fw
    #     with self.goal_lock_:
    #         if len(self.goal_queue_) > 0:
    #             self.goal_queue_.pop(0).execute()
    #         else:
    #             self.goal_handle_ = None

    #####################################################################################


def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
