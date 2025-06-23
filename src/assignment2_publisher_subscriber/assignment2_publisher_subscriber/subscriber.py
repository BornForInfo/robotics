import rclpy
from rclpy.node import Node
from autominy_msgs.msg import Speed

class SpeedSubscriber(Node):

    def __init__(self):
        super().__init__('speed_subscriber')
        self.subscription = self.create_subscription(Speed, '/sensors/speed', self.speed_callback, 10)
        self.subscription

    def speed_callback(self,msg):
        self.get_logger().info(f'Received speed:{msg.value} m/s')

def main(args=None):

    rclpy.init(args=args)

    speed_subscriber = SpeedSubscriber()
    rclpy.spin(speed_subscriber)
    speed_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
