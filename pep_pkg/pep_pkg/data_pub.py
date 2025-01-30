#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO


class DataPublisher(Node):
    def __init__(self):
        super().__init__("data_publisher")
        self.publisher_img = self.create_publisher(Image, "frame", 10)

        #Open cv bridge
        self.br_img = CvBridge()
        #yolo
        self.yolo = YOLO("yolo11n.pt")
        self.yolo.eval()

        #pipeline
        self.pipe = rs.pipeline()
        self.cfg  = rs.config()

        self.cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
        self.cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.pipe.start(self.cfg)
        timer_period = .033
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        
        frames = self.pipe.wait_for_frames()

        color_image = np.asanyarray(frames.get_color_frame().get_data())

        depth_image = np.array(frames.get_depth_frame().get_data(),dtype=np.uint16)

        detections = self.yolo(color_image)[0]

        for result in detections:
            box = result.boxes.xyxy.tolist()[0]
            x1 = int(box[0])
            y1 = int(box[1])
            x2 = int(box[2])
            y2 = int(box[3])
            y_dist = y2 - y1
            x_dist = x2 - x1
            # the bounding boxes kinda suck so we are looking at the general center of them to cut out most of the background
            depth_box = depth_image[(int(y1+y_dist*.40)):(int(y2-y_dist*.40)), (int(x1+x_dist*.40)):(int(x2-x_dist*.40))]

            obj_distance = str(np.median(depth_box))
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (250, 50, 230), 8)
            #center of the bounding box
            cv2.rectangle(color_image, (int((x1+x_dist*.40)), int((y1+y_dist*.40))), (int((x2-x_dist*.40)), int((y2-y_dist*.40))), (0, 230, 30), 3)

            cv2.putText(color_image, obj_distance + "mm", (x1+10, y1+5), cv2.FONT_HERSHEY_SIMPLEX, 1, (105, 255, 255), 3, cv2.LINE_AA)

        self.publisher_img.publish(self.br_img.cv2_to_imgmsg(color_image))


def main(args = None):
    rclpy.init(args = None)
    publisher = DataPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()