#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from interfaces.srv import GetBuoyLocation

class BuoyDetector(Node):
    def __init__(self):
        super().__init__('buoy_detector')
        self.srv = self.create_service(GetBuoyLocation, 'get_buoy_location', self.get_buoy_location_callback)
        self.get_logger().info('Buoy detector service has been started...')

    def get_buoy_location_callback(self, request, response):
        response.left = 69
        response.top = 69
        response.width = 420
        response.height = 420

        self.get_logger().info('Incoming request')
        print(response)
        return response


def main():
    rclpy.init()

    buoy_detector = BuoyDetector()

    rclpy.spin(buoy_detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
