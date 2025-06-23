import rclpy
from rclpy.node import Node
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand

class SteeringSpeedPublisher(Node):

    def __init__(self):
        super().__init__('steering_speed_publisher')
        self.steering_publisher = self.create_publisher(NormalizedSteeringCommand, '/actuators/steering_normalized', 10)
        self.speed_publisher = self.create_publisher(SpeedCommand, '/actuators/speed', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        steering_msg = NormalizedSteeringCommand()
        steering_msg.value = 1.0
        self.steering_publisher.publish(steering_msg)
        self.get_logger().info(f'Publishing steering command: {steering_msg.value}')

        speed_msg = SpeedCommand()
        speed_msg.value = 0.3
        self.speed_publisher.publish(speed_msg)
        self.get_logger().info(f'Publishing speed command: {speed_msg.value} m/s ')

def main(args=None):
    rclpy.init(args=args)

    steering_speed_publisher = SteeringSpeedPublisher()

    rclpy.spin(steering_speed_publisher)
    steering_speed_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
