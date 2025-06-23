import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraExtractor(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        self.subscription = self.create_subscription(
            CameraInfo,
            '/sensors/camera/infra1/camera_info',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg: CameraInfo):
        fx = msg.k[0]
        fy = msg.k[4]
        cx = msg.k[2]
        cy = msg.k[5]

        k1 = msg.d[0]
        k2 = msg.d[1]
        t1 = msg.d[2]
        t2 = msg.d[3]
        k3 = msg.d[4]

        self.get_logger().info(f' Intrinsic Parameters fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}')
        self.get_logger().info(f' Distortion coefficients k1: {k1}, k2: {k2}, t1: {t1}, t2: {t2}, k3: {k3}')

        self.destroy_subscription(self.subscription)

def main(args = None):
    rclpy.init(args=args)
    node = CameraExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
