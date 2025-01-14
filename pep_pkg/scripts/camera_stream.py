#!/usr/bin/env python3
import cv2
import numpy as np

# from std_msgs.msg import String
import pyrealsense2
import rclpy
from rclpy.node import Node


class CameraSteam(Node):
    def __init__(self):
        super().__init__("camera_stream")
        self.rgb_stream_publisher = self.create_publisher(None, "camera_rgb_stream", 0)
        self.depth_stream_publisher = self.create_publisher(
            None,
            "depth_stream",
            0,
        )


def main(args=None):
    pipeline = pyrealsense2.pipeline()
    config = pyrealsense2.config()
    config.enable_stream(pyrealsense2.stream.color, 1280, 720, pyrealsense2.format.z16, 30)
    pipeline.start(config)

    rclpy.init(args=args)

    camera_stream = CameraSteam()

    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        cv2.imshow("RGB Frame", color_image)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    rclpy.spin(camera_stream)

    camera_stream.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
