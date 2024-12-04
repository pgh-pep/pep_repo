#!/usr/bin/env python3
import rclpy
from cv_bridge import CvBridge
from interfaces.msg import BuoyLocation, DepthSlice
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import Image


class DepthMaskNode(Node):
    def __init__(self):
        super().__init__("depth_mask_node")

        # Subscribers for depth and buoy messages
        self.depth_subscriber = Subscriber(self, Image, "/camera/camera/depth/image_rect_raw")
        self.buoy_subscriber = Subscriber(self, BuoyLocation, "buoy_location")

        # Synchronizer to align depth and buoy messages
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_subscriber, self.buoy_subscriber], queue_size=10, slop=0.1, allow_headerless=True
        )
        self.sync.registerCallback(self.sync_callback)

        # Publisher for depth slice
        self.publisher = self.create_publisher(DepthSlice, "depth_slice", 10)

        self.bridge = CvBridge()

    def sync_callback(self, depth_msg, buoy_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

            # Clamps the buoy location to the image boundaries
            clamped_left = max(0, buoy_msg.left)
            clamped_right = min(depth_image.shape[1], buoy_msg.left + buoy_msg.width)
            clamped_top = max(0, buoy_msg.top)
            clamped_bottom = min(depth_image.shape[0], buoy_msg.top + buoy_msg.height)

            # Samples the middle pixel of the buoy location for the depth
            buoy_depth = float(depth_image[(clamped_left + clamped_right) // 2, (clamped_top + clamped_bottom) // 2])

            slice = [-1.0] * 1280
            # Populates the slice with the depth of the buoy
            for i in range(clamped_left, clamped_right):
                slice[i] = buoy_depth

            msg = DepthSlice()
            msg.depth_slice = slice
            self.publisher.publish(msg)
            self.get_logger().info(str(msg.depth_slice))
        except Exception as e:
            self.get_logger().error(f"Error processing synchronized messages: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthMaskNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
