#! /usr/bin/env python
import rclpy
from rclpy.node import Node


class TimingTestes(Node):
    def __init__(self):
        super().__init__("timing_testes_node")
        self.time_rate_ = self.create_timer(1 / 10, self.timing_function)
        self.time_start_ = self.get_clock().now()
        self.t_delta = 1 / self.time_rate_.timer_period_ns
        self.t_next = (
            self.time_start_.to_msg().sec + self.time_start_.to_msg().nanosec / 1e9
        ) + self.t_delta
        self.time_then = (
            self.get_clock().now().to_msg().sec
            + self.get_clock().now().to_msg().nanosec / 1e9
        )

    def timing_function(self):
        now = (
            self.get_clock().now().to_msg().sec
            + self.get_clock().now().to_msg().nanosec / 1e9
        )
        if now > self.t_next:
            elapsed = now - self.time_then
            self.then = now
            self.get_logger().info(
                f"\n- Time tick:\t\t{self.t_next}\n- Time now (epoch[s]):\t{self.get_clock().now().to_msg().nanosec/1e9}\n- Elapsed:\t\t{elapsed}\n- Time Now:\t\t{now}"
            )
        else:
            self.get_logger().warn(
                f"\n- Time tick:\t\t{self.t_next}\n- Time now (epoch[s]):\t{self.get_clock().now().to_msg().nanosec}\n- Elapsed:\t\t NaN\n- Time Now:\t\t{now}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = TimingTestes()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
