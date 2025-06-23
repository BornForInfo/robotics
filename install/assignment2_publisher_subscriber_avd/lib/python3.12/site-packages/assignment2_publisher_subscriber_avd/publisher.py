#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Import Types of publishing messages
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SpeedCommand

class DriveCircleNode(Node):

    def __init__(self):
        super().__init__("drive_circle")

        # Init publisher with (Type, Topic, Buffer (Can just be 10 i guess))
        # steering_pub_ for steering direction
        self.steering_pub_ = self.create_publisher(NormalizedSteeringCommand, "/actuators/steering_normalized", 10)
        # speed_pub for velocity (speed)
        self.speed_pub_ = self.create_publisher(SpeedCommand, "/actuators/speed", 10)

        # Once we start the Node -> reguarly call the callback fnc -
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info("DriveCircle Node started")

    def timer_callback(self):
        # Steering Publisher
        msg_steer = NormalizedSteeringCommand()
        msg_steer.value = 1.0
        self.steering_pub_.publish(msg_steer)
        # Speed Publisher
        msg_speed = SpeedCommand()
        msg_speed.value = 0.3
        self.speed_pub_.publish(msg_speed)
        # Logger to make sure I publish at least
        self.get_logger().info(f"Publishing - Steering: {msg_steer.value} | Speed: {msg_speed.value}")

def main(args=None):
    rclpy.init(args=args)
    node = DriveCircleNode()
    # Make sure Node keeps running with spin()
    rclpy.spin(node)
    # Optional - destroy node
    rclpy.detroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
