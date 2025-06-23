#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from autominy_msgs.msg import Speed

class SpeedSubNode(Node):

    def __init__(self):
        super().__init__("speed_subscriber")
        # Create Subscriber using (Type, Topic, Callback, Buffer (10 again))
        self.speed_sub_ = self.create_subscription(
                Speed, "/sensors/speed", self.speed_callback, 10)

    def speed_callback(self, msg: Speed):
        self.get_logger().info("Current Speed: %s" % msg.value)

def main(args=None):
    rclpy.init(args=args)
    node = SpeedSubNode()
    # Keep Node running
    rclpy.spin(node)
    # Optional - detroy node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
