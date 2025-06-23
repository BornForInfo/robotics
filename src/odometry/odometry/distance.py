import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from math import sqrt

class OdometryDistance(Node):
    def __init__(self):
        super().__init__('odom_distance_node')
        self.subscriber = self.create_subscription(Odometry, '/simulation/odom_ground_truth', self.odom_callback, 10)
        self.start_position = None
        self.total_distance = 0.0
        self.last_position = None

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.start_position is None:
            self.start_position = (x,y)
            self.last_position = (x,y)
            self.get_logger().info(f'Start position: {self.start_position}')
            return

        dx = x - self.last_position[0]
        dy = y - self.last_position[1]
        distance = sqrt(dx**2 + dy**2)
        self.total_distance += distance
        self.last_position = (x,y)

        self.get_logger().info(f"total distance: {self.total_distance:.3f} meters")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
