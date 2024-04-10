#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
 
 
class UrosCmdVelTeste(Node):
    def __init__(self):
        super().__init__("Uros_Test_Node")
        self.msg_ = Int32()
        self.msg_.data = 0
        self.timer_freq_ = 1
        self.data_publisher_ = self.create_publisher(Int32, "cmd_vel", 10)
        self.data_publisher2_ = self.create_publisher(Int32, "Encoder_Msgs", 10)
        self.data_subscriber_ = self.create_subscription(Int32, "/motor/encoder", self.subscription_cb, 10)
        self.timer_ = self.create_timer(1/self.timer_freq_, self.publish_cmd)
        
    def publish_cmd(self):
        self.msg_.data += 1
        self.data_publisher_.publish(self.msg_)
        self.get_logger().info("Dado: " + str(self.msg_))
        
    def subscription_cb(self, msg: Int32):
        self.data_publisher2_.publish(msg)
        self.get_logger().warn("Dado recebido: " + str(msg))

 
def main(args=None):
    rclpy.init(args=args)
    node = UrosCmdVelTeste()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
    