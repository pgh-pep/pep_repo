#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.msg import BuoyLocation as BuoyLocationMsg
from interfaces.srv import GetBuoyLocation

class Main(Node):
    def __init__(self):
        super().__init__('main')

        self.process_depth_slice_publisher = self.create_publisher(BuoyLocationMsg, 'process_depth_slice', 10)

        self.buoy_detection_client = self.create_client(GetBuoyLocation, 'get_buoy_location')
        self.buoy_detection_client.wait_for_service()
        self.buoy_detection_request = GetBuoyLocation.Request()


    def foo(self):
        buoy_location_future = self.buoy_detection_client.call_async(self.buoy_detection_request)
        rclpy.spin_until_future_complete(self, buoy_location_future)
        buoy_location_response = buoy_location_future.result()

        # Convert the service response to a message, because ros hates me. Surely there's a better way to do this.
        buoy_location_message = BuoyLocationMsg()
        buoy_location_message.left = buoy_location_response.left
        buoy_location_message.top = buoy_location_response.top
        buoy_location_message.width = buoy_location_response.width
        buoy_location_message.height = buoy_location_response.height

        print("published")
        self.process_depth_slice_publisher.publish(buoy_location_message)


def main():
    rclpy.init()

    main = Main()
    main.foo()
    print("post foo")

    rclpy.spin(main)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
