#!/usr/bin/env python3
# import numpy as np
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthSubscriber(Node):
    def __init__(self):
        super().__init__("depth_subscriber")

        # We can read from the camera through a simple subscription
        self.subscription = self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.depth_callback,
            10,
        )
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to a NumPy array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Example: Display the depth image using OpenCV
            cv2.imshow("Depth Image", depth_image)
            cv2.waitKey(1)

            # Example: Access a specific pixel's depth value
            depth_value = depth_image[240, 320]  # Pixel at row 240, column 320
            self.get_logger().info(f"Depth value at (240, 320): {depth_value}")

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthSubscriber()
    try:
        print("Spinning")
        rclpy.spin(node)
        print("Post spin")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
