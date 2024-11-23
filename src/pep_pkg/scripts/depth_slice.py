#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import DepthSlice as DepthSliceMsg
from interfaces.msg import BuoyLocation as BuoyLocationMsg
from interfaces.srv import GetBuoyLocation

class DepthSlice(Node):
    def __init__(self):
        super().__init__('depth_slice')

        self.publisher = self.create_publisher(DepthSliceMsg, 'depth_slice', 10)
        self.subscriber = self.create_subscription(BuoyLocationMsg, 'process_depth_slice', self.process_depth_slice_callback, 10)

        self.buoy_detection_client = self.create_client(GetBuoyLocation, 'get_buoy_location')
        self.buoy_detection_client.wait_for_service()
        self.request = GetBuoyLocation.Request()


    def process_depth_slice_callback(self, buoy_location):
        print("Processing depth slice...", buoy_location)
        slice = [-1.0] * 1280

        # Populates the slice with the depth of the buoy, wherever it is
        left, right = max(0, buoy_location.left), min(1280 - 1, buoy_location.left + buoy_location.width)
        for i in range(left, right):
            slice[i] = 1.0

        msg = DepthSliceMsg()
        msg.depth_slice = slice
        self.publisher.publish(msg)


def main():
    rclpy.init()

    depth_slice = DepthSlice()
    print("Yeah we up")
    rclpy.spin(depth_slice)

    depth_slice.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
