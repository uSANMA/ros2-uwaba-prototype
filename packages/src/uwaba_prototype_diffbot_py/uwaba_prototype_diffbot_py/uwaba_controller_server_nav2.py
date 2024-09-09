#!/usr/bin/env python3
import rclpy
import rclpy.logging
import threading
import rclpy.node
import tf_transformations
import numpy as np
from math import sin, cos, pi, atan2, sqrt

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.qos import QoSProfile
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from tf2_ros import TransformBroadcaster, TransformStamped, StaticTransformBroadcaster

from uwaba_prototype_interfaces.action import ControlActions

from geometry_msgs.msg import TwistStamped, Quaternion, Vector3, Twist
from sensor_msgs.msg import JointState, Imu, LaserScan, Temperature, BatteryState
from nav_msgs.msg import Odometry

from uwaba_prototype_interfaces.msg import EncoderMsg


class ControllerServer(LifecycleNode):
    def __init__(self):
        self.lf_node_name_ = "uwaba_controller_server_node"
        super().__init__(f"{self.lf_node_name_}")
        # Constructor parameters
        self.qos_profile_micro_ = QoSProfile(
            depth=10, reliability=2, durability=2, liveliness=1
        )
        self.qos_profile_tf_static_ = QoSProfile(
            depth=10, reliability=0, durability=0, liveliness=1
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
        self.joint_state_left_wheel_ = 0.0
        self.joint_state_right_wheel_ = 0.0
        self.cmd_stop_flag_ = False
        self.first_encoder_msg_flag_ = False
        self.first_imu_msg_flag_ = False
        self.encoder_last_time_ = self.get_clock().now().to_msg()
        self.imu_last_time_ = self.get_clock().now().to_msg()

        self.declare_parameter("wheels_separation", 0.0)
        self.wheels_separation__ = self.get_parameter("wheels_separation").value
        self.declare_parameter("wheel_radius", 0.0)
        self.wheel_radius__ = self.get_parameter("wheel_radius").value
        self.declare_parameter("base_length", 0.0)
        self.base_length__ = self.get_parameter("base_length").value
        self.declare_parameter("base_width", 0.0)
        self.base_width__ = self.get_parameter("base_width").value
        self.declare_parameter("base_height", 0.0)
        self.base_height__ = self.get_parameter("base_height").value
        self.declare_parameter("transform_broadcast_rate", 1.0)
        self.transform_broadcast_rate__ = self.get_parameter(
            "transform_broadcast_rate"
        ).value
        self.declare_parameter("joint_state_rate", 1.0)
        self.joint_state_rate__ = self.get_parameter("joint_state_rate").value
        self.declare_parameter("odom_rate", 1.0)
        self.odom_rate__ = self.get_parameter("odom_rate").value
        self.declare_parameter("lidar_frame", "laser_frame")
        self.lidar_frame__ = self.get_parameter("lidar_frame").value
        self.declare_parameter("main_frame", "base_footprint")
        self.main_frame__ = self.get_parameter("main_frame").value
        self.declare_parameter("robot_base_frame", "base_link")
        self.robot_base_frame__ = self.get_parameter("robot_base_frame").value
        self.declare_parameter("odom_frame", "odom")
        self.odom_frame__ = self.get_parameter("odom_frame").value
        self.declare_parameter("imu_frame", "imu_frame")
        self.imu_frame__ = self.get_parameter("imu_frame").value
        self.declare_parameter("temperature_frame", "temp_frame")
        self.temperature_frame__ = self.get_parameter("temperature_frame").value
        self.declare_parameter("battery_frame", "bat_frame")
        self.battery_frame__ = self.get_parameter("battery_frame").value

        self.declare_parameter("cmd_vel_back", 1.0)
        self.cmd_vel_back_rate__ = self.get_parameter("cmd_vel_back").value
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
        self.declare_parameter("uros_battery_topic", "micro_batterypack")
        self.uros_battery_topic__ = self.get_parameter("uros_battery_topic").value
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
        self.declare_parameter("temperature_topic", "temp")
        self.temperature_topic__ = self.get_parameter("temperature_topic").value
        self.declare_parameter("battery_pack_topic", "bat")
        self.battery_pack_topic__ = self.get_parameter("battery_pack_topic").value
        self.declare_parameter("comp_filter_alpha", 1.0)
        self.comp_filter_alpha__ = self.get_parameter("comp_filter_alpha").value
        self.declare_parameter("imu_dimensions_xyz", [0.0, 0.0, 0.0])
        self.imu_dimensions_xyz__ = self.get_parameter("imu_dimensions_xyz").value
        self.declare_parameter("imu_rotation_rpy", [0.0, 0.0, 0.0])
        self.imu_rotation_rpy__ = self.get_parameter("imu_rotation_rpy").value

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(
            "Server on configure. Now creating publishers and instantiating objects..."
        )
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
        self.temperature_publisher_ = self.create_publisher(
            Temperature,
            f"{self.temperature_topic__}",
            self.qos_profile_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.battery_pack_publisher_ = self.create_publisher(
            BatteryState,
            f"{self.battery_pack_topic__}",
            self.qos_profile_,
            callback_group=ReentrantCallbackGroup(),
        )

        # Odometry starting point
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.vth = 0.0
        self.imu_dt = 0.0
        self.enc_dt = 0.0
        self.total_elapsed_time = 0.0
        self.left_wheel_pos_ = 0.0
        self.right_wheel_pos_ = 0.0
        self.gear_ratio_ = 18.8
        self.left_encoder = 0.0
        self.right_encoder = 0.0
        self.null_ori = np.full((9,), -1)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alpha = (
            self.comp_filter_alpha__
        )  # Complementary filter constant (0 < alpha < 1)
        omegaT = 7.2921159e-5
        latitude_cornelio = -23.1883438
        self.earthRotationZ = omegaT * cos((latitude_cornelio * pi) / 180.0)
        (self.imu_pos_x, self.imu_pos_y, self.imu_pos_z) = self.imu_dimensions_xyz__
        (self.imu_ori_r, self.imu_ori_p, self.imu_ori_y) = self.imu_rotation_rpy__

        self.tf_broadcaster = TransformBroadcaster(self, self.qos_profile_)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self, self.qos_profile_tf_static_)
        self.imu_tf = TransformStamped()
        self.odom_tf = TransformStamped()

        self.orientation = Quaternion()
        self.orientation_imu = Quaternion()

        self.angular_velocity = Vector3()

        self.linear_acceleration = Vector3()

        self.cmd_vel_ = TwistStamped()
        self.cmd_vel_.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_.header.frame_id = ""
        self.cmd_vel_.twist.linear = Twist().linear
        self.cmd_vel_.twist.angular = Twist().angular

        self.encoder_readings_ = EncoderMsg()

        self.joint_state = JointState()

        self.js_frame_id = "wheels_states"
        self.js_states_name = [
            "Left_sprocket_base_joint",
            "Right_sprocket_base_joint",
        ]

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.main_frame__
        self.odom_msg.child_frame_id = self.odom_frame__

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = self.imu_frame__

        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = self.lidar_frame__

        self.temp_msg = Temperature()
        self.temp_msg.header.frame_id = self.temperature_frame__

        self.bat_msg = BatteryState()
        self.bat_msg.header.frame_id = self.battery_frame__

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(
            "Server on activate. Now creating subscribers and publishers' timer callbacks..."
        )

        # self.starting_time_ = self.get_clock().now()
        

        self.cmd_vel_subscriber = self.create_subscription(
            TwistStamped,
            f"{self.cmd_vel_topic__}",
            self.cmd_vel_subscription,
            self.qos_profile_,
            callback_group=ReentrantCallbackGroup(),
        )
        self.uros_encoder_state_subscriber = self.create_subscription(
            EncoderMsg,
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
        self.uros_bate_subscriber = self.create_subscription(
            BatteryState,
            f"{self.uros_battery_topic__}",
            self.uros_bat_subscription,
            self.qos_profile_micro_,
            callback_group=ReentrantCallbackGroup(),
        )

        self.joint_state_publish_timer = self.create_timer(
            (1.0 / self.joint_state_rate__),
            self.joint_state_publish,
            callback_group=ReentrantCallbackGroup(),
        )
        self.odom_publish_timer = self.create_timer(
            (1.0 / self.odom_rate__),
            self.odom_publish,
            callback_group=ReentrantCallbackGroup(),
        )
        self.cmd_vel_back_timer = self.create_timer(
            (1.0 / self.cmd_vel_back_rate__),
            self.cmd_vel_back,
            callback_group=ReentrantCallbackGroup(),
        )
        self.main_execution_timer = self.create_timer(
            (1.0 / 35.0),
            self.main_execution,
            callback_group=ReentrantCallbackGroup(),
        )

        self.server_activated_ = True
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on deactivate")
        self.server_activated_ = False
        with self.goal_lock_:
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.msgs_began_ = False
                self.goal_handle_.abort()
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on cleanup ")
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
        self.joint_state_publish_timer.destroy()
        self.odom_publish_timer.destroy()
        # self.agent_checker_timer.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info("Server on shutdown")
        self.server_activated_ = False
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
        self.joint_state_publish_timer.destroy()
        self.odom_publish_timer.destroy()
        # self.agent_checker_timer.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.server_activated_ = False
        self.get_logger().error(f"\033[1;31;40mThere was an error! {state}\033[0m")
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
        self.joint_state_publish_timer.destroy()
        self.odom_publish_timer.destroy()
        # self.agent_checker_timer.destroy()
        return super().on_error(state)

    def goal_callback(self, goal_request: ControlActions.Goal) -> GoalResponse:
        self.get_logger().info("Received a goal.")
        if not self.server_activated_:
            self.get_logger().warn("Server not yet activated. Rejecting any goals...")
            return GoalResponse.REJECT

        if goal_request.request == "stop" and not self.cmd_stop_flag_:
            self.cmd_stop_flag_ = True
            self.get_logger().info(f"Goal '{goal_request.request}' accepted.")
            return GoalResponse.ACCEPT

        elif goal_request.request == "continue" and self.cmd_stop_flag_:
            self.cmd_stop_flag_ = False
            self.get_logger().info(f"Goal '{goal_request.request}' accepted.")
            return GoalResponse.ACCEPT

        with self.goal_lock_:
            # Policy: Goal preemption (must be after some goal are already valid thus preempting)
            if self.goal_handle_ is not None and self.goal_handle_.is_active:
                self.get_logger().warn(
                    "A goal is already active, aborting current goal."
                )
                self.goal_handle_.abort()

    def cancel_callback(self, goal_handle: ServerGoalHandle) -> CancelResponse:

        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        self.get_logger().warn(
            f"Received a cancel request, canceling with status {goal_handle.status}"
        )

        feedback = ControlActions.Feedback()
        feedback.process = f"--> Goal cancelled successfully.\n--> Robot is currently at:\n---> x:\t{self.x}\n---> y:\t{self.y}\n---> th:\t{self.th}\n"

        goal_handle.publish_feedback(feedback)
        goal_handle.abort()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle) -> ControlActions:
        with self.goal_lock_:
            self.goal_handle_ = goal_handle

        request = goal_handle.request.request
        self.get_logger().info(f"Executing goal {request}")

        result = ControlActions.Result()
        feedback = ControlActions.Feedback()

        if request == "stop":
            feedback.process = f"Server is being 'stop' requested."
            self.cmd_vel_back_timer.cancel()
            self.cmd_vel_.header.stamp = self.get_clock().now().to_msg()
            self.cmd_vel_.header.frame_id = "stop action"
            cmd_zero = Vector3()
            cmd_zero.x, cmd_zero.y, cmd_zero.z = (0.0, 0.0, 0.0)
            self.cmd_vel_.twist.linear = cmd_zero
            self.cmd_vel_.twist.angular = cmd_zero
            self.send_cmd_vel_back_.publish(self.cmd_vel_)
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            result.result_msg = "The action was a success!"
            return result

        elif request == "continue":
            feedback.process = f"Server is being 'continue' requested."
            self.cmd_vel_back_timer.reset()
            goal_handle.publish_feedback(feedback)
            goal_handle.succeed()
            result.result_msg = "The action was a success!"
            return result

        else:
            result.result_msg = "Something failed. Aborting..."
            feedback.process = result.result_msg
            goal_handle.publish_feedback(feedback)
            goal_handle.abort()
            return result
    
    def main_execution(self):
        if 0.030 > self.imu_dt > 0.032:
            self.first_imu_msg_flag_ = False
            self.get_logger().info("Imu flag was set!")
        if 0.030 > self.enc_dt > 0.032:
            self.first_encoder_msg_flag_ = False
            self.get_logger().info("Encoder flag was set!")
    

    def uros_imu_subscription(self, imu_msgs: Imu):
        if not self.first_imu_msg_flag_:
            self.encoder_last_time_ = imu_msgs.header.stamp
            self.first_imu_msg_flag_ = True
        self.imu_msg.header.stamp = imu_msgs.header.stamp
        current_time = self.imu_msg.header.stamp
        self.imu_dt = (current_time.sec + (current_time.nanosec / 1e9)) - (
            self.imu_last_time_.sec + (self.imu_last_time_.nanosec / 1e9)
        )
        

        (
            self.imu_msg.angular_velocity.x,
            self.imu_msg.angular_velocity.y,
            self.imu_msg.angular_velocity.z,
        ) = (
            imu_msgs.angular_velocity.x,
            imu_msgs.angular_velocity.y,
            imu_msgs.angular_velocity.z,
        )
        (
            self.imu_msg.linear_acceleration.x,
            self.imu_msg.linear_acceleration.y,
            self.imu_msg.linear_acceleration.z,
        ) = (
            imu_msgs.linear_acceleration.x,
            imu_msgs.linear_acceleration.y,
            imu_msgs.linear_acceleration.z,
        )
        # self.ori_calc_comp_filter(
        #     self.alpha,
        #     dt,
        #     self.imu_msg.angular_velocity.x,
        #     self.imu_msg.angular_velocity.y,
        #     self.imu_msg.angular_velocity.z,
        #     self.imu_msg.linear_acceleration.x,
        #     self.imu_msg.linear_acceleration.y,
        #     self.imu_msg.linear_acceleration.z,
        # )
        self.ori_calc_simple(
            self.imu_dt,
            self.imu_msg.angular_velocity.x,
            self.imu_msg.angular_velocity.y,
            self.imu_msg.angular_velocity.z,
        )
        self.set_state_transform(
            self.robot_base_frame__,
            self.imu_frame__,
            self.get_clock().now().to_msg(),
            self.imu_tf,
            self.imu_pos_x,
            self.imu_pos_y,
            self.imu_pos_z,
            self.roll,
            self.pitch,
            self.yaw,
            self.orientation_imu,
        )
        self.tf_static_broadcaster.sendTransform(self.imu_tf)
        self.imu_msg.orientation = self.orientation_imu
        self.imu_publisher_.publish(self.imu_msg)

        self.imu_last_time_ = current_time

    def cmd_vel_subscription(self, twist_msgs: TwistStamped):
        self.cmd_vel_.header.stamp = twist_msgs.header.stamp
        self.cmd_vel_.header.frame_id = twist_msgs.header.frame_id
        self.cmd_vel_.twist.linear = twist_msgs.twist.linear
        self.cmd_vel_.twist.angular = twist_msgs.twist.angular

    def uros_encoder_subscription(self, encoder_vels: EncoderMsg):
        if not self.first_encoder_msg_flag_:
            self.encoder_last_time_ = encoder_vels.header.stamp
            self.first_encoder_msg_flag_ = True
        self.encoder_readings_.header.stamp = encoder_vels.header.stamp
        self.encoder_readings_.encoders = encoder_vels.encoders
        current_time = self.encoder_readings_.header.stamp
        self.enc_dt = (current_time.sec + (current_time.nanosec / 1e9)) - (
            self.encoder_last_time_.sec + (self.encoder_last_time_.nanosec / 1e9)
        )

        if encoder_vels.encoders:
            self.left_encoder, self.right_encoder = self.encoder_readings_.encoders

            self.vx = (self.left_encoder + self.right_encoder) / 2.0
            self.vth = (
                self.left_encoder - self.right_encoder
            ) / self.wheels_separation__

            self.pos_calc(self.enc_dt, self.vx, self.vth)
            self.quad_calc(self.orientation, self.th)

            self.left_wheel_pos_ += self.left_encoder
            self.right_wheel_pos_ += self.right_encoder

            if abs(self.left_wheel_pos_) >= (2.0 * pi):
                self.left_wheel_pos_ = 0.0
            if abs(self.right_wheel_pos_) >= (2.0 * pi):
                self.right_wheel_pos_ = 0.0

        self.encoder_last_time_ = current_time

    def uros_laser_subscription(self, laser_msgs: LaserScan):
        self.scan_msg.header.stamp = laser_msgs.header.stamp
        self.scan_msg.angle_min = laser_msgs.angle_min
        self.scan_msg.angle_max = laser_msgs.angle_max
        self.scan_msg.angle_increment = laser_msgs.angle_increment
        self.scan_msg.time_increment = laser_msgs.time_increment
        self.scan_msg.scan_time = laser_msgs.scan_time
        self.scan_msg.range_min = laser_msgs.range_min
        self.scan_msg.range_max = laser_msgs.range_max
        self.scan_msg.ranges = laser_msgs.ranges
        self.scan_msg.intensities = laser_msgs.intensities
        self.scan_publisher_.publish(self.scan_msg)

    def uros_temp_subscription(self, temp_msgs: Temperature):
        self.temp_msg.header.stamp = temp_msgs.header.stamp
        self.temp_msg.temperature = temp_msgs.temperature
        self.temp_msg.variance = temp_msgs.variance
        self.temperature_publisher_.publish(self.temp_msg)

    def uros_bat_subscription(self, bat_msgs: BatteryState):
        # self.bat_msg.header.stamp = bat_msgs.header.stamp
        # self.bat_msg.voltage = bat_msgs.voltage
        # self.bat_msg.temperature = bat_msgs.temperature
        # self.bat_msg.current = bat_msgs.current
        # self.bat_msg.charge = bat_msgs.charge
        # self.bat_msg.capacity = bat_msgs.capacity
        # self.bat_msg.design_capacity = bat_msgs.design_capacity
        # self.bat_msg.percentage = bat_msgs.percentage
        # self.bat_msg.power_supply_status = bat_msgs.power_supply_status
        # self.bat_msg.power_supply_health = bat_msgs.power_supply_health
        # self.bat_msg.power_supply_technology = bat_msgs.power_supply_technology
        # self.bat_msg.present = bat_msgs.present
        # self.bat_msg.cell_voltage = bat_msgs.cell_voltage
        # self.bat_msg.cell_temperature = bat_msgs.temperature
        # self.bat_msg.location = bat_msgs.location
        # self.bat_msg.serial_number = bat_msgs.serial_number
        # self.battery_pack_publisher_.publish(self.bat_msg)
        pass

    def joint_state_publish(self):
        self.set_joint_state_pkg(
            self.get_clock().now().to_msg(),
            self.joint_state,
            self.js_frame_id,
            self.js_states_name,
            [self.left_wheel_pos_, self.right_wheel_pos_],
            [self.left_encoder, self.right_encoder],
        )
        self.joint_state_publisher_.publish(self.joint_state)

    def odom_publish(self):
        self.set_odom_pkg(
            self.get_clock().now().to_msg(),
            self.odom_msg,
            self.x,
            self.y,
            self.vx,
            self.vth,
            self.th,
            self.orientation,
        )
        self.odom_publisher_.publish(self.odom_msg)
        self.set_state_transform(
            self.odom_frame__,
            self.main_frame__,
            self.get_clock().now().to_msg(),
            self.odom_tf,
            self.x,
            self.y,
            self.z,
            0.0,
            0.0,
            self.th,
            self.orientation,
        )
        self.tf_broadcaster.sendTransform(self.odom_tf)

    def cmd_vel_back(self):
        self.send_cmd_vel_back_.publish(self.cmd_vel_)

    def pos_calc(self, dt, vx, vth, vy=0.0):
        # For differential bots the lateral velocity is zero, so vy = 0.0
        ## This is a transformation matrix so we can exchange info between fixed and moving references
        delta_x = vx * cos(self.th) * dt
        delta_y = vx * sin(self.th) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

    def ori_calc_comp_filter(self, alpha, dt, gyro_x, gyro_y, gyro_z, ax, ay, az):
        roll_gyro = self.roll + gyro_x * dt
        pitch_gyro = self.pitch + gyro_y * dt
        yaw_gyro = self.yaw + gyro_z * dt

        roll_accel = atan2(ay, sqrt(az**2 + az**2))
        pitch_accel = atan2(-ax, sqrt(ay**2 + az**2))

        self.roll = alpha * roll_gyro + (1 - alpha) * roll_accel
        self.pitch = alpha * pitch_gyro + (1 - alpha) * pitch_accel
        self.yaw = yaw_gyro

        self.quad_calc(self.orientation_imu, self.yaw, self.roll, self.pitch)

    def ori_calc_simple(self, dt, gyro_x, gyro_y, gyro_z):
        self.roll += gyro_x * dt
        self.pitch += gyro_y * dt
        self.yaw += gyro_z * dt

        self.quad_calc(self.orientation_imu, self.yaw, self.roll, self.pitch)

    # This is an array filler for covariances
    def null_cov_n(self, n=0.0, value=-1.0):
        return np.full((n,), value)

    def quad_calc(self, set_orientation: Quaternion, az, ax=0.0, ay=0.0) -> Quaternion:
        set_orientation.x, set_orientation.y, set_orientation.z, set_orientation.w = (
            tf_transformations.quaternion_from_euler(ax, ay, az)
        )
        return set_orientation

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
        set_odom_msg.header.stamp = current_time
        set_odom_msg.pose.pose.position.x = pos_x
        set_odom_msg.pose.pose.position.y = pos_y
        set_odom_msg.pose.pose.position.z = 0.0
        set_odom_msg.twist.twist.linear.x = vx
        set_odom_msg.twist.twist.linear.y = 0.0
        set_odom_msg.twist.twist.linear.z = 0.0
        set_odom_msg.twist.twist.angular.x = 0.0
        set_odom_msg.twist.twist.angular.y = 0.0
        set_odom_msg.twist.twist.angular.z = vth
        set_odom_msg.pose.pose.orientation = self.quad_calc(set_orientation, th)
        return set_odom_msg

    def set_joint_state_pkg(
        self,
        current_time,
        set_joint_state: JointState,
        frame_id: str,
        name: list[str],
        pos: list[float],
        vel: list[float],
    ):
        set_joint_state.header.stamp = current_time
        set_joint_state.header.frame_id = frame_id
        set_joint_state.name = name
        set_joint_state.position = pos
        set_joint_state.velocity = vel

    def set_state_transform(
        self,
        frame_id,
        child_frame_id,
        current_time,
        set_tf: TransformStamped,
        pos_x,
        pos_y,
        pos_z,
        roll,
        pitch,
        yaw,
        set_orientation: Quaternion,
    ):
        set_tf.header.stamp = current_time
        set_tf.header.frame_id = frame_id
        set_tf.child_frame_id = child_frame_id
        # Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
        set_tf.transform.translation.x = pos_x
        set_tf.transform.translation.y = pos_y
        set_tf.transform.translation.z = pos_z
        set_tf.transform.rotation = self.quad_calc(set_orientation, yaw, roll, pitch)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerServer()
    rclpy.spin(node, MultiThreadedExecutor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
