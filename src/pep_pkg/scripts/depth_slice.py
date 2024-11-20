#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import DepthSlice
from interfaces.msg import BuoyLocation
from interfaces.srv import GetBuoyLocation

class DepthSlice(Node):
    def __init__(self):
        super().__init__('depth_slice')

        self.publisher = self.create_publisher(DepthSlice, 'depth_slice', 10)
        self.subscriber = self.create_subscription(BuoyLocation, 'process_depth_slice', self.process_depth_slice_callback, 10)

        self.buoy_detection_client = self.create_client(GetBuoyLocation, 'get_buoy_location')
        self.buoy_detection_client.wait_for_service()
        self.request = GetBuoyLocation.Request()


    def process_depth_slice_callback(self, msg):
        print("Processing depth slice...", msg)
        slice = [-1] * 1280

        # Populates the slice with the depth of the buoy, wherever it is
        left, right = max(0, msg.data.left), min(1280 - 1, msg.data.left + msg.data.width)
        for i in range(left, right):
            slice[i] = 1

        msg = DepthSlice()
        msg.data = slice
        self.publisher.publish(msg)
        print(slice)

        return slice


def main():
    rclpy.init()

    depth_slice = DepthSlice()

    depth_slice.get_depth_slice()

    rclpy.spin(depth_slice)

    depth_slice.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
