import rclpy
from rclpy.node import Node
import cv2 as cv

from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import pyrealsense2 as rs2

if not hasattr(rs2, 'intrinsics'):
    import pyrealsense2.pyrealsense2 as rs2

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion
import math

"""
-y
|
|
|-x__ __ __ __ __+x
|
|
+y
"""


def quaternion_from_euler(ai, aj, ak):
    """Compute a quaternion from Euler angles.

    Args:
        ai (float): x
        aj (float): y
        ak (float): z

    Returns:
        q: the Quaternion representation
    """
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = Quaternion()
    q.x = cj * sc - sj * cs
    q.y = cj * ss + sj * cc
    q.z = cj * cs - sj * sc
    q.w = cj * cc + sj * ss

    return q


class ImageProcessNode(Node):
    """
    Node that reads images and draws on them using open cv.

    Publishers
    ----------
    new_image (sensor_msgs/msg/Image): The image after post procesasing

    Subscribers
    -----------
    image (sensor_msgs/msg/Image): The image on which to do the processing
    """

    def __init__(self):
        super().__init__('image_processor_colors')
        self.bridge = CvBridge()
        self.create_subscription(msg_Image, 'rgb_image', self.rgb_process, 10)
        self.create_subscription(
            msg_Image, 'depth_image', self.depth_process, 10
        )
        self.create_subscription(
            CameraInfo, 'color_camera_info', self.imageDepthInfoCallback, 1
        )

        self.pub_redball = self.create_publisher(msg_Image, 'red_ball', 10)


        self.pub_table = self.create_publisher(msg_Image, 'table', 10)

        timer_period = 0.05  # secs
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cx = None
        self.cy = None
        self.depth_value = None
        self.depth_scale = 0.001

        self.rgb_image_height = None
        self.rgb_image_width = None

        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

        self.red_ball_x = None
        self.red_ball_y = None
        self.red_ball_z = None

        self.has_red_ball = False

        self.tf_broadcaster = TransformBroadcaster(self)


    def imageDepthInfoCallback(self, cameraInfo):
        """_summary_

        Args:
            cameraInfo (_type_): _description_
        """
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
        except CvBridgeError as e:
            print(e)
            return

    def detect_table(self, image):
        """_summary_

        Args:
            image (_type_): _description_
        """
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        # isolate green surface
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        green_mask = cv.inRange(hsv_image, lower_green, upper_green)
        green_table = cv.bitwise_and(hsv_image, hsv_image, mask=green_mask)
        new_msg = self.bridge.cv2_to_imgmsg(green_table, encoding='bgr8')
        self.pub_table.publish(new_msg)

    def rgb_process(self, image):
        """_summary_

        Args:
            image (_type_): _description_
        """
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
        self.rgb_image_height, self.rgb_image_width, _ = cv_image.shape

        self.detect_table(cv_image)
        for color in self.hsv_dict:
            self.rgb_process_multiple_color_balls(cv_image, color)

        hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv.inRange(hsv_image, lower_red2, upper_red2)
        red_ball_mask = cv.bitwise_or(mask1, mask2)

        red_ball = cv.bitwise_and(hsv_image, hsv_image, mask=red_ball_mask)
        new_msg = self.bridge.cv2_to_imgmsg(red_ball, encoding='bgr8')

        # Find the center of mass (centroid) of the red ball
        cx, cy, largest_contour = self.find_center_of_mass(red_ball_mask)

        if largest_contour is not None:
            area = cv.contourArea(largest_contour)

            if area >= 100 and area <= 800:  # Area big enough to be a ball
                self.cx = cx
                self.cy = cy
                # self.get_logger().info(f"Red ball contour area: {area}")

                if cx and cy and self.depth_value:
                    self.has_red_ball = True
                    coords = self.pixel_to_world(cx, cy, self.depth_value)
                    if coords:
                        self.red_ball_x = coords[0]
                        self.red_ball_y = coords[1]
                        self.red_ball_z = coords[2]
            else:
                self.reset_redball()
        else:
            self.reset_redball()

        self.pub_redball.publish(new_msg)


    def depth_process(self, image):
        """Callback to process the depth image.


        Args:
            image (_type_): _description_
        """
        depth_image = self.bridge.imgmsg_to_cv2(
            image, desired_encoding='16UC1'
        )
        if self.cx and self.cy:
            # Scale cx and cy to match the depth image resolution
            cx_scaled = int(
                self.cx * (depth_image.shape[1] / self.rgb_image_width)
            )
            cy_scaled = int(
                self.cy * (depth_image.shape[0] / self.rgb_image_height)
            )
            self.depth_value = depth_image[cy_scaled, cx_scaled]

    def find_center_of_mass(self, mask):
        """_summary_

        Args:
            mask (_type_): _description_

        Returns:
            _type_: _description_
        """
        # Find contours in the mask
        contours, _ = cv.findContours(
            mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
        )

        if contours:
            # Get the largest contour by area
            largest_contour = max(contours, key=cv.contourArea)

            # Calculate the moments of the largest contour
            M = cv.moments(largest_contour)

            # Ensure the moment is not zero (to avoid division by zero)
            if M["m00"] != 0:
                # Calculate the center of mass (cx, cy)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                return cx, cy, largest_contour
        return None, None, None  # if no red ball is found

    def pixel_to_world(self, u, v, depth_value):
        """
        Convert pixel coordinates (u, v) and depth to world coordinates.
        Parameters:
            u (int): Pixel x-coordinate.
            v (int): Pixel y-coordinate.
            depth_value (float): Depth value at (u, v).
        Returns:
            tuple: (x, y, z) world coordinates in meters.
        """
        if self.intrinsics:
            fx = self.intrinsics.fx
            fy = self.intrinsics.fy
            cx = self.intrinsics.ppx
            cy = self.intrinsics.ppy

            # Convert pixel (u, v) and depth to world coordinates
            z = depth_value * self.depth_scale  # Convert depth to meters
            x = ((u - cx) * z / fx) + 0.008
            y = (v - cy) * z / fy

            return [x, y, z]
        return None

    def timer_callback(self):
        return

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()


def main():
    rclpy.init()
    n = ImageProcessNode()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()