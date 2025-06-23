import rclpy
from rclpy.node import Node
from autominy_msgs.msg import Tick


class TickCounter(Node):
    def __init__(self):
        super().__init__('tick_counter')
        self.subscription = self.create_subscription(Tick, '/sensors/arduino/ticks', self.tick_callback, 10)
        self.total_ticks = 0

    def tick_callback(self, msg):
        self.total_ticks += msg.value

        self.get_logger().info(f"Total Ticks Counted: {self.total_ticks}")

def main(args=None):
    rclpy.init(args=args)
    node = TickCounter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
