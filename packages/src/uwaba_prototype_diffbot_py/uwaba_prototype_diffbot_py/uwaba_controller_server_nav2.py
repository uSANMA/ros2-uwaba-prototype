#!/usr/bin/env python3
import rclpy
import threading
import time
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

from geometry_msgs.msg import TwistStamped, Quaternion, Vector3, Twist
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
        self.qos_profile_micro_ = QoSProfile(
            depth=10, reliability=2, durability=2, liveliness=1
        )
        self.qos_profile_ = 10
        self.server_activated_ = False
        self.goal_handle_: ServerGoalHandle = None
        self.goal_lock_ = threading.Lock()
        self.timing_lock_ = threading.Lock()
        self.got_timing_ = False
        self.msgs_began_ = False
        self.got_twist_package_ = False
        self.got_encoder_package_ = False
        self.got_imu_package_ = False
        self.got_lidar_package_ = False
        self.got_temp_package_ = False
        self.joint_state_broadcaster_ = TransformBroadcaster(self, self.qos_profile_)
        self.joint_state_left_wheel_ = 0.0
        self.joint_state_right_wheel_ = 0.0
        self.starting_time_ = None

        # Imu covariance fill
        # self.imu_covariance_fill_ = np.zeros((9,))
        # for i in range(0, len(self.imu_covariance_fill_)):
        #     self.imu_covariance_fill_[i] += 0.001
        self.imu_covariance_fill_ = np.full((9,), 0.001)

        # Subscription parameters
        # self.covariance_fill_ = np.zeros((36,))
        # for j in range(0, len(self.covariance_fill_)):
        #     self.covariance_fill_[j] += 0.001
        self.covariance_fill_ = np.full((36,), 0.001)

        self.cmd_vel_ = TwistStamped()
        self.cmd_vel_.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_.header.frame_id = ""
        self.cmd_vel_.twist.linear = Twist().linear
        self.cmd_vel_.twist.angular = Twist().angular
        self.motor_velocity_ = JointState()
        self.motor_velocity_.header.stamp = self.get_clock().now().to_msg()
        self.motor_velocity_.header.frame_id = ""
        self.motor_velocity_.name = ["", ""]
        self.motor_velocity_.velocity = []

        # ROS 2 parameters
        self.declare_parameter("wheels_separation", 0.0)
        self.wheels_separation__ = self.get_parameter("wheels_separation").value
        self.declare_parameter("wheel_radius", 0.0)
        self.wheel_radius__ = self.get_parameter("wheel_radius").value

        self.declare_parameter("transform_rate", 1.0)
        self.transform_rate__ = self.get_parameter("transform_rate").value

        self.declare_parameter("joint_state_rate", 1.0)
        self.joint_state_rate__ = self.get_parameter("joint_state_rate").value

        self.declare_parameter("cmd_vel_rate", 1.0)
        self.cmd_vel_rate__ = self.get_parameter("cmd_vel_rate").value

        self.declare_parameter("odom_rate", 1.0)
        self.odom_rate__ = self.get_parameter("odom_rate").value

        self.declare_parameter("imu_rate", 1.0)
        self.imu_rate__ = self.get_parameter("imu_rate").value

        self.declare_parameter("scan_rate", 1.0)
        self.scan_rate__ = self.get_parameter("scan_rate").value

        self.declare_parameter("tune_velocity", 1.0)
        self.tune_vel__ = self.get_parameter("tune_velocity").value
        self.declare_parameter("goal_position_x", 0.0)
        self.goal_pos__ = self.get_parameter("goal_position_x").value
        self.declare_parameter("goal_orientation_z", 0.0)
        self.goal_ori__ = self.get_parameter("goal_orientation_z").value
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
        self.declare_parameter("imu_topic", "cmd_vel")
        self.imu_topic__ = self.get_parameter("imu_topic").value
        self.declare_parameter("scan_topic", "scan")
        self.scan_topic__ = self.get_parameter("scan_topic").value

        # Odometry starting point
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.dt = 0.0

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on configure")  ## REMOVE WHEN DONE
        self.control_server_ = ActionServer(
            self,
            ControlActions,
            "uwaba_prototype/control_server",
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.send_cmd_vel_back_ = self.create_publisher(
            TwistStamped,
            f"{self.lf_node_name_}/cmd_vel",
            self.qos_profile_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.odom_publisher_ = self.create_publisher(
            Odometry,
            f"{self.odom_topic__}",
            self.qos_profile_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.joint_state_publisher_ = self.create_publisher(
            JointState,
            f"{self.joint_state_topic__}",
            self.qos_profile_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.imu_publisher_ = self.create_publisher(
            Imu,
            f"{self.imu_topic__}",
            self.qos_profile_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.scan_publisher_ = self.create_publisher(
            LaserScan,
            f"{self.scan_topic__}",
            self.qos_profile_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Controller action server has started.")
        self.joint_state = JointState()
        self.joint_state.name = [
            "Left_sprocket_base_joint",
            "Right_sprocket_base_joint",
        ]
        self.joint_state.header.frame_id = "wheels_states"

        self.orientation = Quaternion()
        self.angular_velocity = Vector3()
        self.linear_acceleration = Vector3()

        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = "odom"
        self.odom_trans.child_frame_id = "base_footprint"

        self.twist_msg = TwistStamped()

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"
        self.odom_msg.pose.covariance = self.covariance_fill_
        self.odom_msg.twist.covariance = self.covariance_fill_

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu"
        (
            self.imu_msg.linear_acceleration.x,
            self.imu_msg.linear_acceleration.y,
            self.imu_msg.linear_acceleration.z,
        ) = [0.0, 0.0, 0.0]
        (
            self.imu_msg.angular_velocity.x,
            self.imu_msg.angular_velocity.y,
            self.imu_msg.angular_velocity.z,
        ) = [0.0, 0.0, 0.0]
        self.imu_msg.orientation_covariance = self.imu_covariance_fill_
        self.imu_msg.angular_velocity_covariance = self.imu_covariance_fill_
        self.imu_msg.linear_acceleration_covariance = self.imu_covariance_fill_
        
        self.scan_msg = LaserScan()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on activate")  ## REMOVE WHEN DONE
        self.cmd_vel_subscriber = self.create_subscription(
            TwistStamped,
            f"{self.cmd_vel_topic__}",
            self.cmd_vel_subscription,
            self.qos_profile_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.uros_encoder_state_subscriber = self.create_subscription(
            JointState,
            f"{self.uros_encoder_topic__}",
            self.uros_encoder_subscription,
            self.qos_profile_micro_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.uros_lidar_subscriber = self.create_subscription(
            LaserScan,
            f"{self.uros_lidar_topic__}",
            self.uros_laser_subscription,
            self.qos_profile_micro_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.uros_imu_subscriber = self.create_subscription(
            Imu,
            f"{self.uros_imu_topic__}",
            self.uros_imu_subscription,
            self.qos_profile_micro_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.uros_temp_subscriber = self.create_subscription(
            Temperature,
            f"{self.uros_temperature_topic__}",
            self.uros_temp_subscription,
            self.qos_profile_micro_,
            callback_group=ReentrantCallbackGroup(),
        )

        self.transform_publish_rate = self.create_timer(
            (1.0 / self.transform_rate__),
            self.transform_publish,
            callback_group=ReentrantCallbackGroup(),
        )

        self.joint_state_publish_rate = self.create_timer(
            (1.0 / self.joint_state_rate__),
            self.joint_state_publish,
            callback_group=ReentrantCallbackGroup(),
        )

        self.cmd_vel_publish_rate = self.create_timer(
            (1.0 / self.cmd_vel_rate__),
            self.cmd_vel_publish,
            callback_group=ReentrantCallbackGroup(),
        )

        self.odom_publish_rate = self.create_timer(
            (1.0 / self.odom_rate__),
            self.odom_publish,
            callback_group=ReentrantCallbackGroup(),
        )

        self.imu_publish_rate = self.create_timer(
            (1.0 / self.imu_rate__),
            self.imu_publish,
            callback_group=ReentrantCallbackGroup(),
        )

        self.scan_publish_rate = self.create_timer(
            (1.0 / self.scan_rate__),
            self.scan_publish,
            callback_group=ReentrantCallbackGroup(),
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
                self.msgs_began_ = False
                self.goal_handle_.abort()
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
        self.transform_publish_rate.destroy()
        self.joint_state_publish_rate.destroy()
        self.cmd_vel_publish_rate.destroy()
        self.odom_publish_rate.destroy()
        self.imu_publish_rate.destroy()
        self.scan_publish_rate.destroy()
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
        self.transform_publish_rate.destroy()
        self.joint_state_publish_rate.destroy()
        self.cmd_vel_publish_rate.destroy()
        self.odom_publish_rate.destroy()
        self.imu_publish_rate.destroy()
        self.scan_publish_rate.destroy()
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
        self.transform_publish_rate.destroy()
        self.joint_state_publish_rate.destroy()
        self.cmd_vel_publish_rate.destroy()
        self.odom_publish_rate.destroy()
        self.imu_publish_rate.destroy()
        self.scan_publish_rate.destroy()
        return super().on_error(state)

    def goal_callback(self, goal_request: ControlActions.Goal) -> GoalResponse:
        self.get_logger().info("Received a goal.")  ## REMOVE WHEN DONE
        if not self.server_activated_:
            self.get_logger().warn("Server not yet activated.")
            return GoalResponse.REJECT

        if goal_request.request == "set_goal":
            self.get_logger().info(
                f"Goal {goal_request.request} accepted. Destination was set at: {goal_request.goal_request}"
            )
            return GoalResponse.ACCEPT

        elif goal_request.request == "activate main loop":
            self.get_logger().info(
                f"Goal {goal_request.request.upper()} accepted. Starting loop execution."
            )
            return GoalResponse.ACCEPT

        with self.goal_lock_:
            # Policy: Goal preemption (must be after some goal are already valid thus preempting)
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().warn("A goal is already active, aborting new goal.")
                self.goal_handle_.abort()

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:

        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        self.get_logger().warn(
            f"Received a cancel request, canceling with status {goal_handle.status}"
        )

        feedback = ControlActions.Feedback()
        current_time = self.get_clock().now()
        self.odom_msg = self.set_odom_pkg(
            current_time,
            self.odom_msg,
            self.x,
            self.y,
            0.0,
            0.0,
            self.th,
            self.orientation,
        )
        self.odom_trans = self.set_state_transform(
            current_time,
            self.odom_trans,
            self.x,
            self.y,
            self.th,
            self.orientation,
        )
        self.twist_msg = self.set_twist_pkg(
            current_time,
            self.twist_msg,
            self.cmd_vel_.header.frame_id,
            0.0,
            0.0,
        )
        feedback.process = f"--> Goal cancelled successfully.\n--> Robot is currently at:\n---> x:\t{self.x}\n---> y:\t{self.y}\n---> th:\t{self.th}\n"
        goal_handle.publish_feedback(feedback)
        goal_handle.abort()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> ControlActions:
        self.get_logger().info("Executing goal")
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        result = ControlActions.Result()  # instance the result object
        feedback = ControlActions.Feedback()  # instance the feedback object

        total_elapsed_time = 0.0
        self.starting_time_ = self.get_clock().now()

        while rclpy.ok():

            if not goal_handle.is_active:
                result.result_msg = "Preempted by another goal."
                return result

            if goal_handle.is_cancel_requested:
                result.result_msg = "Goal was cancel requested"
                return result

            with self.timing_lock_:
                current_time = self.get_clock().now()
                self.dt = current_time - self.starting_time_
                self.dt = self.dt.to_msg().sec + self.dt.to_msg().nanosec / 1e9

                if self.motor_velocity_.velocity:
                    right_motor_vel, left_motor_vel = self.motor_velocity_.velocity
                else:
                    right_motor_vel, left_motor_vel = [0.0, 0.0]

                self.vx = (right_motor_vel + left_motor_vel) / 2.0
                self.vth = (right_motor_vel - left_motor_vel) / self.wheels_separation__

                self.odom_calc(self.dt, self.vx, self.vth)

                self.joint_state = self.set_joint_state_pkg(
                    current_time, self.joint_state, self.vx
                )
                self.odom_msg = self.set_odom_pkg(
                    current_time,
                    self.odom_msg,
                    self.x,
                    self.y,
                    self.vx,
                    self.vth,
                    self.th,
                    self.orientation,
                )
                self.imu_msg = self.set_imu_pkg(
                    current_time,
                    self.imu_msg,
                    self.imu_msg.header.frame_id,
                    self.th,
                    self.orientation,
                    self.imu_msg.angular_velocity,
                    self.imu_msg.linear_acceleration,
                )
                self.odom_trans = self.set_state_transform(
                    current_time,
                    self.odom_trans,
                    self.x,
                    self.y,
                    self.th,
                    self.orientation,
                )
                self.twist_msg = self.set_twist_pkg(
                    current_time,
                    self.twist_msg,
                    self.cmd_vel_.header.frame_id,
                    self.cmd_vel_.twist.linear.x,
                    self.cmd_vel_.twist.angular.z,
                )

                total_elapsed_time += self.dt
                self.starting_time_ = current_time

            self.reset_flags()

        self.reset_flags()
        result.result_msg = "Something failed... Aborting"
        feedback.process = result.result_msg
        goal_handle.publish_feedback(feedback)
        goal_handle.abort()
        return result

    def cmd_vel_subscription(self, twist_msgs: TwistStamped):
        with self.timing_lock_:
            self.cmd_vel_.header.stamp = twist_msgs.header.stamp
            self.cmd_vel_.header.frame_id = twist_msgs.header.frame_id
            self.cmd_vel_.twist.linear = twist_msgs.twist.linear
            self.cmd_vel_.twist.angular = twist_msgs.twist.angular
            self.got_twist_package_ = True

    def uros_encoder_subscription(self, motor_vels: JointState):
        with self.timing_lock_:
            self.motor_velocity_.header.stamp = motor_vels.header.stamp
            self.motor_velocity_.header.frame_id = motor_vels.header.frame_id
            self.motor_velocity_.name = motor_vels.name
            if motor_vels.velocity:
                self.motor_velocity_.velocity = motor_vels.velocity
            self.got_encoder_package_ = True

    def uros_laser_subscription(self, laser_msgs: LaserScan):
        with self.timing_lock_:
            self.got_lidar_package_ = True

    def uros_imu_subscription(self, imu_msgs: Imu):
        with self.timing_lock_:
            self.imu_msg.header.stamp = imu_msgs.header.stamp
            self.imu_msg.header.frame_id = imu_msgs.header.frame_id
            self.imu_msg.orientation = imu_msgs.orientation
            self.imu_msg.angular_velocity = imu_msgs.angular_velocity
            self.imu_msg.linear_acceleration = imu_msgs.linear_acceleration
            self.got_imu_package_ = True

    def uros_temp_subscription(self, temp_msgs: Temperature):
        with self.timing_lock_:
            self.got_temp_package_ = True

    def set_imu_pkg(
        self,
        current_time,
        set_imu_msg: Imu,
        frame_id,
        th,
        set_orientation: Quaternion,
        set_angular_velocity: Vector3,
        set_linear_acceleration: Vector3,
    ) -> Imu:
        set_imu_msg.header.stamp = current_time.to_msg()
        set_imu_msg.header.frame_id = frame_id
        set_imu_msg.orientation = self.orientation_calc(set_orientation, th)
        set_imu_msg.angular_velocity = set_angular_velocity
        set_imu_msg.linear_acceleration = set_linear_acceleration
        return set_imu_msg

    def set_twist_pkg(
        self, current_time, set_twist_msg: TwistStamped, frame_id, vx, vth
    ) -> TwistStamped:
        set_twist_msg.header.stamp = current_time.to_msg()
        set_twist_msg.header.frame_id = frame_id
        set_twist_msg.twist.linear.x = vx
        set_twist_msg.twist.linear.y = 0.0
        set_twist_msg.twist.linear.z = 0.0
        set_twist_msg.twist.angular.x = 0.0
        set_twist_msg.twist.angular.y = 0.0
        set_twist_msg.twist.angular.z = vth
        return set_twist_msg

    def set_odom_pkg(
        self,
        current_time,
        set_odom_msg: Odometry,
        pos_x,
        pos_y,
        vx,
        vth,
        th,
        set_orientation: Quaternion,
    ) -> Odometry:
        set_odom_msg.header.stamp = current_time.to_msg()
        set_odom_msg.pose.pose.position.x = pos_x
        set_odom_msg.pose.pose.position.y = pos_y
        set_odom_msg.pose.pose.position.z = 0.0
        set_odom_msg.twist.twist.linear.x = vx
        set_odom_msg.twist.twist.linear.y = 0.0
        set_odom_msg.twist.twist.linear.z = 0.0
        set_odom_msg.twist.twist.angular.x = 0.0
        set_odom_msg.twist.twist.angular.y = 0.0
        set_odom_msg.twist.twist.angular.z = vth
        set_odom_msg.pose.pose.orientation = self.orientation_calc(set_orientation, th)
        return set_odom_msg

    def set_joint_state_pkg(
        self, current_time, set_joint_state: JointState, vx
    ) -> JointState:
        set_joint_state.header.stamp = current_time.to_msg()
        set_joint_state.position = [self.x, self.x]
        set_joint_state.velocity = [vx, vx]
        return set_joint_state

    def set_state_transform(
        self,
        current_time,
        set_odom_trans: TransformStamped,
        pos_x,
        pos_y,
        th,
        set_orientation: Quaternion,
    ) -> TransformStamped:
        set_odom_trans.header.stamp = current_time.to_msg()
        set_odom_trans.transform.translation.x = pos_x
        set_odom_trans.transform.translation.y = pos_y
        set_odom_trans.transform.translation.z = 0.0
        set_odom_trans.transform.rotation = self.orientation_calc(set_orientation, th)
        return set_odom_trans

    def orientation_calc(self, set_orientation: Quaternion, az) -> Quaternion:
        set_orientation.x, set_orientation.y, set_orientation.z, set_orientation.w = (
            tf_transformations.quaternion_from_euler(0.0, 0.0, az)
        )
        return set_orientation

    def odom_calc(self, dt, vx, vth, vy=0.0) -> float:
        delta_x = float((vx * cos(self.th) - vy * sin(self.th)) * dt)
        delta_y = float((vx * sin(self.th) + vy * cos(self.th)) * dt)
        delta_th = float(vth * dt)

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        return delta_x, delta_y, delta_th

    def transform_publish(self):
        with self.timing_lock_:
            self.joint_state_broadcaster_.sendTransform(self.odom_trans)

    def joint_state_publish(self):
        with self.timing_lock_:
            self.joint_state_publisher_.publish(self.joint_state)

    def cmd_vel_publish(self):
        with self.timing_lock_:
            self.send_cmd_vel_back_.publish(self.twist_msg)

    def odom_publish(self):
        with self.timing_lock_:
            self.odom_publisher_.publish(self.odom_msg)

    def imu_publish(self):
        with self.timing_lock_:
            self.imu_publisher_.publish(self.imu_msg)

    def scan_publish(self):
        with self.timing_lock_:
            self.scan_publisher_.publish(self.scan_msg)

    # def send_transforms(
    #     self,
    #     twist_msg: TwistStamped = None,
    #     odom_msg: Odometry = None,
    #     joint_state: JointState = None,
    #     odom_trans: TransformStamped = None,
    #     imu_msg: Imu = None,
    # ):
    #     if twist_msg is not None:
    #         self.send_cmd_vel_back_.publish(twist_msg)
    #     if odom_msg is not None:
    #         self.odom_publisher_.publish(odom_msg)
    #     if joint_state is not None:
    #         self.joint_state_publisher_.publish(joint_state)
    #     if odom_trans is not None:
    #         self.joint_state_broadcaster_.sendTransform(odom_trans)
    #     if imu_msg is not None:
    #         self.imu_publisher_.publish(imu_msg)

    def reset_flags(self):
        with self.timing_lock_:
            self.got_encoder_package_ = False
            self.got_twist_package_ = False
            self.got_imu_package_ = False
            self.got_lidar_package_ = False
            self.got_temp_package_ = False


def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
