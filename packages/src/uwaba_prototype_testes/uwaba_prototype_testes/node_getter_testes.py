#! /usr/bin/env python
import rclpy
from rclpy.node import Node
import time


class NodeGetTest(Node):
    def __init__(self):
        super().__init__("node_get_test")
        self.get_logger().info("\033[90;1m Get Node Test Has Started\033[0m")

        self.timer = self.create_timer(2.5, callback=self.timing_test_pause)
        #self.timer2 = self.create_timer(1.0, callback=self.timer_handler)
        #self.timer3 = self.create_timer(1.0, callback=self.timerzin)

    def timing_test_pause(self):
        active_imu_nodes = self.get_publishers_info_by_topic("/micro_imu")
        active_scan_nodes = self.get_publishers_info_by_topic("/micro_laserscan")
        active_encoder_nodes = self.get_publishers_info_by_topic("/micro_encoders")
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
        if active_imu_nodes and active_encoder_nodes and active_scan_nodes:
            for info in active_imu_nodes:
                self.get_logger().info(
                    f"\n---\t- \033[95;1mTopic Hz:\033[0m \033[92m{...}\033[0m\
                    \n\t- \033[95;1mNodes found:\033[0m \033[94m{info.node_name}\033[0m\
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
        else:
            self.get_logger().info("\n\033[93;1m---\n\tTopics are idle!!!\n---\033[0m")
            

    def timer_handler(self):
        # active_nodes = self.get_publishers_info_by_topic("/cmd_vel")
        # for info in active_nodes:
        #     self.get_logger().info(
        #         f"\n---\n\t- \033[95;1mNodes found:\033[0m \033[94m{info.node_name}\033[0m\
        #         \n\t- \033[95;1mTopic Type:\033[0m \033[94;4m{info.topic_type}\033[0m\
        #         \n\t- \033[95;1mEndpoint GID:\033[0m \033[94m{info.endpoint_gid}\033[0m\
        #         \n\t- \033[95;1mEndpoint Type:\033[0m \033[94m{info.endpoint_type}\033[0m\
        #         \n\t- \033[95;1mNode Namespace:\033[0m \033[94m{info.node_namespace}\033[0m\
        #         \n\t- \033[95;1mQoS:\033[0m\
        #         \n\t\t-- \033[95mHistory:\t\t\t\t\033[0m \033[94m{info.qos_profile.history}\033[0m\
        #         \n\t\t-- \033[95mDepth:\t\t\t\t\033[0m \033[94m{info.qos_profile.depth}\033[0m\
        #         \n\t\t-- \033[95mReliability:\t\t\t\t\033[0m \033[94m{info.qos_profile.reliability}\033[0m\
        #         \n\t\t-- \033[95mDurability:\t\t\t\t\033[0m \033[94m{info.qos_profile.durability}\033[0m\
        #         \n\t\t-- \033[95mLifespan:\t\t\t\t\033[0m \033[94m{info.qos_profile.lifespan}\033[0m\
        #         \n\t\t-- \033[95mDeadline:\t\t\t\t\033[0m \033[94m{info.qos_profile.deadline}\033[0m\
        #         \n\t\t-- \033[95mLiveliness:\t\t\t\t\033[0m \033[94m{info.qos_profile.liveliness}\033[0m\
        #         \n\t\t-- \033[95mLiveliness Lase Duration:\t\t\033[0m \033[94m{info.qos_profile.liveliness_lease_duration}\033[0m\
        #         \n\t\t-- \033[95mAvoid ROS Namespace Conventions:\t\033[0m \033[94m{info.qos_profile.avoid_ros_namespace_conventions}\033[0m\n---"
        #     )
        if self.timer.is_ready():
            self.get_logger().info("Timer 1 is ready")
        if self.timer3.is_ready():
            self.get_logger().info("Timer 3 is ready")
    
    def timerzin(self):
        ...


def main(args=None):
    rclpy.init(args=args)
    node = NodeGetTest()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
