import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraManegement(Node):
    def __init__(self):
        super().__init__('camera_manegement')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg:str):
        try:
            # Convert the ROS Image message to a OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            self.detect_and_draw_square(cv_image)

            # Display the image using OpenCV
            #cv2.imshow("Camera Image", cv_image)
            #cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def detect_and_draw_square(self, image):
        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Gray", gray)

        # Apply GaussianBlur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        cv2.imshow("Blur", blurred)

        # Use Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)

        # Find contours in the edge-detected image
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through each contour
        for contour in contours:
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # If the polygon has 4 vertices, it is a square (which corresponds to a cube in 2D)
            if len(approx) == 4:
                # Draw the contour
                cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)

                # Get the bounding box
                x, y, w, h = cv2.boundingRect(approx)

                # Draw the bounding box around the detected cube
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)


def main(args=None):
    rclpy.init(args=args)

    camera_management = CameraManegement()

    rclpy.spin(camera_management)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_management.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
