#!/usr/bin/env python3
import rclpy
from interfaces.srv import GetBuoyLocation
from rclpy.node import Node


class BuoyDetector(Node):
    def __init__(self):
        super().__init__("buoy_detector")
        self.srv = self.create_service(GetBuoyLocation, "get_buoy_location", self.get_buoy_location_callback)
        self.get_logger().info("Buoy detector service has been started...")

    def get_buoy_location_callback(self, request, response):
        response.location.left = 69
        response.location.top = 69
        response.location.width = 420
        response.location.height = 420

        self.get_logger().info("BuoyLocation: " + str(response.location))
        return response


def main():
    rclpy.init()

    buoy_detector = BuoyDetector()

    rclpy.spin(buoy_detector)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
