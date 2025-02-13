#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Empty
from interfaces.msg import BoundingBox2D, BoundingBox2DArray
from sensor_msgs.msg import Image
from ultralytics import YOLO


class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection_node")

        self.subscription = self.create_subscription(Image, "/camera/camera/color/image_raw", self.color_callback, 0)
        self.notify_publisher = self.create_publisher(Empty, "notify_track_most_recent_depth_frame", 0)
        self.bounding_box_publisher = self.create_publisher(BoundingBox2DArray, "object_bounding_boxes", 10)

        self.bridge = CvBridge()
        self.model = YOLO("data/model.pt")  # Load the YOLO model


    def color_callback(self, color_msg):
        # Notify the depth-slice node to keep track of the most recent depth frame
        self.notify_publisher.publish(Empty())

        color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding="passthrough")
        color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        boxes = self.model(color_image_rgb)[0].boxes.xyxy.tolist()  # Run the YOLO model on the image
        bounding_boxes = BoundingBox2DArray()
        bounding_boxes.header.stamp = self.get_clock().now().to_msg()
        bounding_boxes.header.frame_id = color_msg.header.frame_id

        self.get_logger().info("In color callback")
        for box in boxes:
            x1, y1, x2, y2 = int(box[0]), int(box[1]), int(box[2]), int(box[3])

            self.get_logger().info(f"Buoy detected at ({x1}, {y1}) with width {x2 - x1} and height {y2 - y1}")

            bounding_box = BoundingBox2D()
            bounding_box.x1 = x1
            bounding_box.y1 = y1
            bounding_box.x2 = x2
            bounding_box.y2 = y2

            bounding_boxes.boxes.append(bounding_box)
            cv2.rectangle(color_image_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow("Object Detection", color_image_rgb)
        cv2.waitKey(1)

        self.bounding_box_publisher.publish(bounding_boxes)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
