#!/usr/bin/env python3
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import BuoyLocation
from rclpy.node import Node
from sensor_msgs.msg import Image


class BuoyDetectionNode(Node):
    def __init__(self):
        super().__init__("buoy_detection_node")

        self.subscription = self.create_subscription(Image, "/camera/camera/color/image_raw", self.depth_callback, 10)
        self.publisher = self.create_publisher(BuoyLocation, "buoy_location", 10)

        self.bridge = CvBridge()

    def depth_callback(self, color_msg):
        # color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="passthrough")

        # Placeholder buoy detection logic
        buoy_location = BuoyLocation()
        buoy_location.left = 100
        buoy_location.top = 150
        buoy_location.width = 50
        buoy_location.height = 50

        # Publish buoy location
        self.get_logger().info("Buoy Location: " + str(buoy_location))
        self.publisher.publish(buoy_location)


def main():
    rclpy.init()

    node = BuoyDetectionNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
