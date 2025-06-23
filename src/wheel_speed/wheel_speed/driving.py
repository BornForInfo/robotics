import rclpy #write ROS2 Python
from rclpy.node import Node
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand

class DriveDistance(Node):
    def __init__(self):
        super().__init__('drive_distance_node')
        self.steering_pub = self.create_publisher(NormalizedSteeringCommand, '/actuators/steering_normalized', 10)
        self.speed_pub = self.create_publisher(SpeedCommand, '/actuators/speed', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = self.get_clock().now() #stores time during init
        self.duration = 6.7 #approx duration for 2 meters 0.3m/s
        self.active = True

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds*1e-9
        if elapsed <= self.duration:
            steering_msg = NormalizedSteeringCommand() #creates msg
            steering_msg.value = 1.0 #fills msg with value
            self.steering_pub.publish(steering_msg) #publishes msg

            speed_msg = SpeedCommand()
            speed_msg.value = 0.3
            self.speed_pub.publish(speed_msg)

            self.get_logger().info(f'Driving... t={elapsed:.2f}s')
        elif elapsed > self.duration:
            steering_msg = NormalizedSteeringCommand()
            steering_msg.value = 0.0
            self.steering_pub.publish(steering_msg)

            speed_msg = SpeedCommand()
            speed_msg.value = 0.0
            self.speed_pub.publish(speed_msg)

            self.get_logger().info('Stopped')
            self.active = False

def main(args=None):
    rclpy.init(args=args) #init
    node = DriveDistance() #call Node
    rclpy.spin(node) #loop node
    node.destroy_node() #end node
    rclpy.shutdown() #terminate

if __name__ == '__main__':
    main()
