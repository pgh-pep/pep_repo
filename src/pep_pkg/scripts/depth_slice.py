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

        # Subscribers for depth image and buoy location
        self.depth_subscriber = Subscriber(self, Image, "/camera/camera/depth/image_rect_raw")
        self.buoy_subscriber = Subscriber(self, BuoyLocation, "buoy_location")

        # Synchronizer to align depth and buoy messages
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_subscriber, self.buoy_subscriber], queue_size=10, slop=0.1, allow_headerless=True
        )
        self.sync.registerCallback(self.sync_callback)

        # Publisher for the processed depth slice
        self.publisher = self.create_publisher(DepthSlice, "depth_slice", 10)

        # CV Bridge for converting ROS Image to NumPy
        self.bridge = CvBridge()

    def sync_callback(self, depth_msg, buoy_msg):
        try:
            self.get_logger().info("Yeah")
            # Convert depth image to a NumPy array
            # depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")

            # Create a masked depth array initialized to -1
            # masked_depth = np.full(depth_image.shape, -1, dtype=np.float32)

            # Apply buoy mask based on the buoy location
            # left, top, width, height = buoy_msg.left, buoy_msg.top, buoy_msg.width, buoy_msg.height
            # masked_depth[top : top + height, left : left + width] = depth_image[top : top + height, left : left + width]

            # Publish the masked depth array
            # msg = DepthSlice()
            # msg.depth_array = masked_depth.flatten().tolist()
            # self.publisher.publish(msg)

            # print("Processing depth slice...", buoy_location)
            slice = [-1.0] * 1280

            # Populates the slice with the depth of the buoy, wherever it is
            left, right = max(0, buoy_msg.left), min(1280 - 1, buoy_msg.left + buoy_msg.width)
            for i in range(left, right):
                slice[i] = 1.0

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
