import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class Ransac(Node):
    def __init__(self):
        super().__init__('image_cropper')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/sensors/camera/infra1/image_rect_raw', self.image_callback, 10)
        self.pub = self.create_publisher(Image, '/cropped/image', 10)

    def point_distance(self, p1, p2, p3):
        tmp = (p2[0] - p1[0])*p3[1] - (p2[1] - p1[1])*p3[0] + p2[1]*p1[0] - p2[0]*p1[1]
        return abs(tmp)/np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

    def ransac(self, binary, iterations, threshold=2.0, minimum_inliner=1000):
        #threshold = 2.0
        points = np.argwhere(binary == 255)
        #minimum_inliner = 500
        final_points = []
        final_points_inliner = []
        random_points_list = []
        random_points_inliner = []
        rng = np.random.default_rng()
        for i in range(iterations):
            random_points = rng.choice(points, size = 2, replace = False)
            random_points_list.append(random_points)
            random_points_inliner.append(0)
            for p in points:
                dist = self.point_distance(random_points[0], random_points[1], p)
                if dist < threshold:
                    random_points_inliner[-1] += 1
        for i in range(len(random_points_list)):
            if random_points_inliner[i] > minimum_inliner:
                final_points.append(random_points_list[i])
                final_points_inliner.append(random_points_inliner[i])
        #print(final_points[0][0], final_points[0][1], final_points)
        #threshold again

        filtered_lines = self.filter_similar_lines(final_points, final_points_inliner)
        return filtered_lines


    def filter_similar_lines(self, points, inliers, threshold=30):
        keep = [True] * len(points)
        for i in range(len(points)):
            if not keep[i]:
                continue
            for j in range(i + 1, len(points)):
                if not keep[j]:
                    continue
                dist_x = self.point_distance(points[i][0], points[i][1], points[j][0])
                dist_y = self.point_distance(points[i][0], points[i][1], points[j][1])
                if dist_x < threshold and dist_y < threshold:
                    if inliers[i] >= inliers[j]:
                        keep[j] = False
                    else:
                        keep[i] = False
                        break  # No need to check further for i
        filtered_points = [p for p, k in zip(points, keep) if k]
        return filtered_points

    def image_callback(self, msg):
        threshold = 200
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        mask = np.zeros_like(cv_image[:,:])
        mask_regions = [
                ((0,0),(640,110)),
                ((580,110),(640, 400)),
                ((500,110),(640, 300)),
                ((440, 110),(640, 200))
        ]
        for top_left, bot_right in mask_regions:
            cv2.rectangle(mask, top_left, bot_right, 255, -1)
        mask = cv2.bitwise_not(mask)

        _, binary = cv2.threshold(cv_image, threshold, 255, cv2.THRESH_BINARY)
        cropped = binary[150:480, 0:640]
        masked_img = cv2.bitwise_and(binary, binary, mask=mask)
        """
        cv2.imshow('image', cv_image)
        cv2.imshow('masked', cv2.bitwise_and(cv_image, cv_image, mask=mask))
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        """
        # calling ransac
        ransac_lines = self.ransac(masked_img, 100, threshold=1.5, minimum_inliner=500)
        #print(ransac_lines)
        for line in ransac_lines:
            # crop original image if using cropped
            cv2.line(cv_image, line[0][::-1], line[1][::-1], (0, 0 , 0), 5)

        #convert back to ROS message
        cropped_msg = self.bridge.cv2_to_imgmsg(cv_image)
        #mask_msg = self.bridge.cv2_to_imgmsg(masked_img)

        #publish cropped image
        cropped_msg.header = msg.header
        self.pub.publish(cropped_msg)
        self.get_logger().info(f"Published image :)")

def main(args=None):
    rclpy.init(args=args)
    node = Ransac()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
        for i in range(len(final_points)):
            for j, p in enumerate (final_points):
                dist = self.point_distance(final_points[i][0], final_points[i][1], p[0])
                if dist < threshold and final_points_inliner[i] < final_points_inliner[j]:
                    final_points.pop(i)
                    final_points_inliner.pop(i)

"""
