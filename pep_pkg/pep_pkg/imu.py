import rclpy
import serial
from rclpy.node import Node


class Imu(Node):
    def __init__(self):
        super().__init__("IMU")
        self.header_lsb = b'\x55'
        self.header_msb_32 = b'\xA0'
        self.header_msb_16 = b'\xA1'
        self.header_msb_8 = b'\xA2'
        self.check_error = True
        self.device = "/dev/ttyUSB0"
        print("Starting IMUs....")
        print("Hello")
        self.print_serial_output()

    def print_serial_output(self, speed):
    	
    	with serial.Serial(self.device, xonxoff=False,rtscts=True) as s:
    		while True:
    			value = s.read(s.in_waiting)
    			print(value)


    def low_pass_filter():
    	pass

    
def main():
    rclpy.init()
    node = Imu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
