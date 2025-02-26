#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Empty
from interfaces.msg import DepthSlice, BoundingBox2DArray
from sensor_msgs.msg import Image


class DepthMaskNode(Node):
    def __init__(self):
        super().__init__("depth_mask_node")

        self.tracked_depth_frame = None
        self.most_recent_depth_frame = None

        # Publisher for depth slice
        self.publisher = self.create_publisher(DepthSlice, "depth_slice", 10)

        # Subscribers for depth and bounding boxes messages
        self.depth_subscriber = self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.depth_callback, 10)
        self.track_subscriber = self.create_subscription(Empty, "notify_track_most_recent_depth_frame", self.track_callback, 1)
        self.boxes_subscriber = self.create_subscription(BoundingBox2DArray, "object_bounding_boxes", self.bounding_boxes_callback, 0)

        self.bridge = CvBridge()

        self.get_logger().info("Depth Mask Node has been started")


    def depth_callback(self, depth_msg):
        self.most_recent_depth_frame = depth_msg


    def track_callback(self, msg):
        self.get_logger().info("Tracked depth frame!")
        self.tracked_depth_frame = self.most_recent_depth_frame


    def bounding_boxes_callback(self, boxes_msg):
        if self.tracked_depth_frame is None:
            self.get_logger().warn("Received bounding boxes, but no depth frame is being tracked")
            return

        depth_image = self.bridge.imgmsg_to_cv2(self.tracked_depth_frame, desired_encoding="passthrough")

        depth_slice = [-1] * depth_image.shape[1]
        for box in boxes_msg.boxes:
            x_middle = int((box.x1 + box.x2) / 2)
            y_middle = int((box.y1 + box.y2) / 2)
            depth = depth_image[y_middle, x_middle]
            for x in range(box.x1, box.x2):
                if depth_slice[x] == -1 or depth < depth_slice[x]:
                    depth_slice[x] = depth

        self.get_logger().info(str(depth_slice))


def main(args=None):
    rclpy.init(args=args)
    node = DepthMaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
