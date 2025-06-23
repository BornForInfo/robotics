import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from autominy_msgs.msg import SpeedCommand, SteeringCommand
import math
import numpy as np

class Drive(Node):

    def __init__(self):
        super().__init__("drive_maze")

        self.lidar_sub = self.create_subscription(LaserScan, '/sensors/rplidar/scan', self.lidar_callback, 10)

        self.drive_pub = self.create_publisher(SpeedCommand, "/actuators/speed", 10)
        self.steer_pub = self.create_publisher(SteeringCommand, "/actuators/steering", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.obstacle_front = False
        self.turn_direction = 0

    def lidar_callback(self, msg):
        #converting to array easier implmentations
        ranges = np.array(msg.ranges)

        #define front, left and right sectors
        min_threshold = 1
        left = ranges[0:90]
        right = ranges[270:][::-1]
        for i in range(len(left)):
            if left[i] > min_threshold and right[i] > min_threshold:
                left[i] = min_threshold
                right[i] = min_threshold
        threshold = 0.5
        dif = np.sum(np.subtract(left,right))
        if abs(dif) > threshold:
            if dif > 0:
                self.turn_direction = -1
            else:
                self.turn_direction = 1
        else:
            self.turn_direction = 0

        self.get_logger().info(f"range {ranges}")

    def timer_callback(self):
        msg_drive = SpeedCommand()
        msg_drive.value = 0.3
        self.drive_pub.publish(msg_drive)

        msg_steering = SteeringCommand()
        if self.turn_direction == -1:
            msg_steering.value = 0.5
        elif self.turn_direction == 1:
            msg_steering.value = -0.5
        else:
            msg_steering.value = 0.0

        self.steer_pub.publish(msg_steering)

def main(args=None):
    rclpy.init(args=args)
    node = Drive()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
