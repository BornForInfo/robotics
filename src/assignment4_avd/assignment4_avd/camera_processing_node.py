import rclpy
from rclpy.node import Node

import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


class NameTodo(Node):

    def __init__(self):
        super().__init__('nametodo')

        self.real_coords = np.array([[.5, .2, .0], [.8, .2, .0], [1.1, .2, .0], [1.1, -.2, .0], [.8, -.2, .0], [.5, -.2, .0]])
 
        self.camera_image_sub_ = self.create_subscription(
                Image, "/sensors/camera/infra1/image_rect_raw",
                self.process_image, 10)

        self.camera_info_sub_ = self.create_subscription(
                CameraInfo, "/sensors/camera/infra1/camera_info",
                self.get_camera_parameters, 10)

        self.camera_bin_pub_ = self.create_publisher(Image, "/processed/camera/masked_bin", 10)

        self.timer_ = self.create_timer(1, self.timer_callback)
        self.get_logger().info("Camera processing started...")

    def timer_callback(self):
        camera_bin_msg = CvBridge().cv2_to_imgmsg(self.masked_image)
        self.camera_bin_pub_.publish(camera_bin_msg)
        self.get_logger().info("Publishing binary camera image")
        print(self.real_coords)
        print(self.image_points)
        print(self.camera_matrix)
        print(self.dist_coeff)
        try:
            ret_val, rvec, tvec = cv2.solvePnP(self.real_coords, self.image_points, self.camera_matrix, self.dist_coeff)
            self.get_logger().info(f"\nRodrigues/Rotation Vectors:\n{rvec}\n{"-"*20}\nTranslation vector:\n{tvec}\n")
        except Exception as e:
            self.get_logger().warning(f"Exeption occurred when calling solvePnP: {e}")
        try:
            rotation, _ = cv2.Rodrigues(rvec)
            self.get_logger().info(f"Rodrigues Vector:\n{rotation}")
        except Exception as e:
            self.get_logger().warning(f"Exeption occurred when calling Rodrigues function: {e}")
        

    def get_camera_parameters(self, msg:CameraInfo):
        self.camera_matrix = np.reshape(np.array(msg.k), (-1,3))
        self.dist_coeff = np.array(msg.d)

    def process_image(self, imgmsg:Image):
        img = CvBridge().imgmsg_to_cv2(imgmsg, "mono8")
        img = cv2.GaussianBlur(img, (5,5),0)

        mask = np.zeros_like(img[:,:])
        mask_regions = [
            ((0,375),(640,480)),
            ((0,0),(640,70)),
            ((260,245),(480,420)),
            ((500,70),(640,100)),
            ((0,70),(295,100)),
            ((0,100),(240,115))
        ]
        for top_left, bot_right in mask_regions:
            cv2.rectangle(mask, top_left, bot_right, 255, -1)
        mask = cv2.bitwise_not(mask)
        
        ret, bin_img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
        
        self.masked_image = cv2.bitwise_and(bin_img, bin_img, mask=mask)

        self.find_white_pixel_pos()

    def find_white_pixel_pos(self):
        bin_img = cv2.morphologyEx(self.masked_image, cv2.MORPH_OPEN, np.ones((2,2), np.uint8))
        white_pixels = np.column_stack(np.where(bin_img == 255)).astype(np.float32)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        comp, labels, centers = cv2.kmeans(white_pixels, 6, None, criteria, 10 , cv2.KMEANS_PP_CENTERS)
        centers = [tuple(map(int, center[::-1])) for center in centers]
        marked_image = cv2.cvtColor(self.masked_image, cv2.COLOR_GRAY2BGR)
        for center in centers:
            cv2.circle(marked_image, center, radius=5, color=(255,0,0), thickness=-1)
        self.show_img(marked_image)
        self.image_points = np.array(sorted(centers, key=lambda element: (element[0], element[1])), dtype=np.float32)

    def show_img(self, img):
        cv2.imshow('image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    camera_node = NameTodo()
    rclpy.spin(camera_node)

    # Destroy node
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
