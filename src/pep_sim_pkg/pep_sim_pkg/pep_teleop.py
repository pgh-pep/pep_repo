#!/usr/bin/env python3
from threading import Lock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64, Float64MultiArray
from visualization_msgs.msg import Marker


class PEPTeleopNode(Node):
    def __init__(self):
        super().__init__("pep_teleop")

        self.outboard_servo_data = 0
        self.thruster_data = 0

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        self.outboard_servo_sub = self.create_subscription(Float64, '/pepbot/outboard_servo', self.outboard_servo_callback, 10)
        self.thruster_sub = self.create_subscription(Float64, '/pepbot/thrust', self.thruster_callback, 10)

        self.controller_pub = self.create_publisher(
            Float64MultiArray, '/velocity_trajectory_controller/commands', 10
        )

        self.outboard_lock = Lock()
        self.thruster_lock = Lock()

    def outboard_servo_callback(self, msg):

        data = msg.data
        self.outboard_servo_data = data
        self.get_logger().info(f"Outboard Angle: {self.outboard_servo_data}")
        # self._joy_msg_lock.acquire()
        # self._joy_msg = data
        # self._joy_msg_lock.release()

        # with self._joy_msg_lock:
        #     self._joy_msg = data

    def thruster_callback(self, msg):

        data = msg.data
        self.thruster_data = data
        self.get_logger().info(f"thrust: {self.thruster_data}")


    def joint_state_callback(self, msg):

        self.get_logger().info(f"Outboard Angle: {self.outboard_servo_data}")
        self.get_logger().info(f"Thrust: {self.thruster_data}")

        next_joint_state = Float64MultiArray()
        next_joint_state.data = [3*float(self.outboard_servo_data),.1*float(self.thruster_data)]

        # msg_out.header.stamp = self.get_clock().now().to_msg(

        # print(type(msg))


        self.controller_pub.publish(next_joint_state)



def main(args=None):
    rclpy.init(args=args)

    pep_teleop_node = PEPTeleopNode()

    rclpy.spin(pep_teleop_node)

    pep_teleop_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
