#!/usr/bin/env python3
import rclpy
from interfaces.msg import BuoyLocation as BuoyLocationMsg
from interfaces.srv import GetBuoyLocation
from rclpy.node import Node


class Main(Node):
    def __init__(self):
        super().__init__("main")

        self.process_depth_slice_publisher = self.create_publisher(BuoyLocationMsg, "process_depth_slice", 10)

        self.buoy_detection_client = self.create_client(GetBuoyLocation, "get_buoy_location")
        self.buoy_detection_client.wait_for_service()
        self.buoy_detection_request = GetBuoyLocation.Request()

    def foo(self):
        # Get the buoy's location
        buoy_location_future = self.buoy_detection_client.call_async(self.buoy_detection_request)
        rclpy.spin_until_future_complete(self, buoy_location_future)
        buoy_location_response = buoy_location_future.result()

        # Tell something to create the depth slice from the buoy location
        self.process_depth_slice_publisher.publish(buoy_location_response.location)


def main():
    rclpy.init()

    main = Main()
    main.foo()

    rclpy.spin(main)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
