#! /usr/bin/env python
import rclpy
from rclpy.node import Node
import time


class NodeGetTest(Node):
    def __init__(self):
        super().__init__("node_get_test")
        self.a = 0.0
        # topics = self.get_topic_names_and_types()
        # self.get_logger().info(f"{topics}")
        # [
        #     self.get_logger().info(f"Found {topic[0]}")
        #     for topic in topics
        #     if topic[0] in ["/chatter", "/rosout", "/parameter_events", "/cmd_vel", "/imu", "/odom"]
        # ]

        self.timer = self.create_timer(1.0, callback=self.timing_test_pause)
        self.timer2 = self.create_timer(1.0, callback=self.timer_handler)
        
        
        self.get_logger().info(f"{self.get_node_names_and_namespaces_with_enclaves()}")

    def timing_test_pause(self):
        # self.a += 1.0
        # self.get_logger().info(f"Counter: {self.a}")
        # nodes = self.get_node_names()
        # # self.get_logger().info(f"{nodes}")
        # [
        #     self.get_logger().error(f"Found {node}")
        #     for node in nodes
        #     if node in ["pingpong_node"]
        # ]
        pubs = self.get_publishers_info_by_topic("/microROS/ping")
        self.get_logger().info(f"Pubs - {pubs}")

    def timer_handler(self):
        # if self.a > 5.0:
        #     self.timer.cancel()
        #     self.get_logger().warn("Timer cancelled.")
        #     self.a = 0.0
        #     time.sleep(2.0)
        #     self.timer.reset()
        #     self.get_logger().warn("Timer reset.")
        # topics = self.get_topic_names_and_types()
        # # self.get_logger().info(f"{topics}")
        # [
        #     self.get_logger().warn(f"Found {topic[0]}")
        #     for topic in topics
        #     if topic[0] in ["/chatter", "/rosout", "/parameter_events", "/cmd_vel", "/imu", "/odom"]
        # ]
        subs = self.get_subscriptions_info_by_topic("/microROS/ping")
        self.get_logger().warn(f"Subs - {subs}")


def main(args=None):
    rclpy.init(args=args)
    node = NodeGetTest()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
