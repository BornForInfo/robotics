import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R


class Image_Thresholder(Node):
    def __init__(self):
        super().__init__('image_threshold_node')
        self.bridge = CvBridge() #conversion ROS2 OpenCV
        self.publisher = self.create_publisher(Image, '/binary_image', 10)
        self.subscription = self.create_subscription(
            Image,
            '/sensors/camera/infra1/image_rect_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        threshold = 200
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8') #mono8 for grayImages
        #print("image shape:", gray.shape)
        _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
        cv2.rectangle(binary, (0, 0), (640, 110), (0,), -1) #topleft
        cv2.rectangle(binary, (0, 110), (50, 160), (0,), -1) #undertopleft
        cv2.rectangle(binary, (0, 300), (640, 480), (0,), -1) #bottemleft
        cv2.rectangle(binary, (220,190 ), (480, 300), (0,), -1) #center

        height, width = binary.shape
        region = []
        rows, cols = 7, 2
        h_step, w_step = height // rows, width // cols
        centers = []

        for row in range(1,4):
            for col in range(cols):
                x_start = col * w_step
                x_end = (col+1) * w_step
                y_start = row * h_step
                y_end = (row+1) * h_step

                region = binary[y_start:y_end, x_start:x_end]
                white_pixel = np.column_stack(np.where(region == 255))

                if white_pixel.size > 0:
                    mean_yx = np.mean(white_pixel, axis=0)
                    center_x = int(mean_yx[1]) + x_start
                    center_y = int(mean_yx[0]) + y_start

                    centers.append((center_x, center_y))
                    cv2.circle(binary, (center_x,center_y), 2, (20,), -1)

#        calculating optimal rows,cols
#        for i in range(1,rows):
#            y = i*height // rows
#            cv2.line(binary, (0, y), (width, y), (128,), 1)
#        for i in range(1,cols):
#            x = i * width // cols
#            cv2.line(binary, (x, 0), (x, height), (128,), 1)

        object_points = np.array([
            [1.1, 0.2, 0.0], #top left
            [1.1, -0.2, 0.0], #top right
            [0.8, 0.2, 0.0], #middle left
            [0.8, -0.2, 0.0], #middle right
            [0.5, 0.2, 0.0], #bottem left
            [0.5, -0.2, 0.0] #bottem right
        ], dtype = np.float32)
        plain_points = np.array([[267, 122], [417, 113], [246, 162], [445, 150], [206, 237], [504, 224]], dtype = np.float32)

        intrinsic_matrix = np.array([[383.7944641113281, 0, 322.3056945800781], [0, 383.7944641113281, 241.67051696777344], [0,0,1]], dtype = np.float32)
        distortion = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype = np.float32)

        success, rmat, tvec = cv2.solvePnP(object_points, plain_points, intrinsic_matrix, distortion)
        if success:
            print("rosrigis: ", rmat)
            print("translation: ", tvec)
        rotation, _ = cv2.Rodrigues(rmat)
        print(f"rotation matrix{rotation}")
        r = R.from_matrix(rotation)
        roll, pitch, yaw = r.as_euler('xyz', degrees = False)
        print(f"Roll: {roll}, Pitch {pitch}, Yaw {yaw}")
        r = R.from_euler('ZYX', [yaw, pitch, roll])
        qx, qy, qz, qw = r.as_quat()
        #print(f"qx: {qx}, qy: {qy}, qz: {qz}, qw: {qw}")
        #print("true rotation:", rotation)
        #print("center cords: ", centers)
        #print("object_points", object_points)
        binary_msg = self.bridge.cv2_to_imgmsg(binary, encoding='mono8')
        self.publisher.publish(binary_msg)
        self.get_logger().info('Published binary image')


def main(args=None):
    rclpy.init(args=args)
    node = Image_Thresholder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()
