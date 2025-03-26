import serial
import time
import rclpy
from rclpy.node import Node
from pep_interfaces.msg import RC
# from std_msgs.msg import Float32Array

# Reference: SBUS Arduino library: https://github.com/bolderflight/sbus/tree/main
# Designed for the Radiolink R8EF Reciever + Inverter (w/ the T8FB transmitter)

# TODO: timestamp stored with data if necessary


class SBUSReader:
    def __init__(self, port="/dev/ttyUSB0", baudrate=100000):
        self.port = port
        self.baudrate = baudrate
        self.serial_port = None
        self.currentPacket = [int]

    def init_connection(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1,
                # bytesize=serial.EIGHTBITS,
                # parity=serial.PARITY_EVEN,
                # stopbits=serial.STOPBITS_TWO,
            )

            # self.serial_port.dsrdtr = True

            time.sleep(2.5)

            print(f"Connected: {self.port} w/ {self.baudrate} baud.")

        except Exception as e:
            print(f"ERROR cannot open port: {e}")
            self.serial_port = None

    def close_connection(self):
        if self.serial_port:
            self.serial_port.close()
            print("Closing SBUS comms")

    def read_packet(self):
        try:
            current_data = self.serial_port.readline().decode().strip()

            if current_data:
                self.currentPacket = list(map(int, current_data.split()))

        except Exception:
            pass


class RC_Controller(SBUSReader):
    def __init__(self, port="/dev/ttyUSB0", baudrate=100000):
        SBUSReader.__init__(self, port, baudrate)
        self.raw_channels = {}
        self.channels = {}

        self.channel_labels = [
            "Joy_R_LR",
            "Joy_R_UD",
            "Joy_L_UD",
            "Joy_L_LR",
            "SW_B",
            "Vr_B",
            "SW_A",
            "Vr_A",
        ]

        for label in self.channel_labels:
            self.raw_channels[label] = None
            self.channels[label] = None

        # self.current_decoded_packet = {
        #     "channels": [],
        #     "frames_lost": 0,
        #     "failsafe_activated": 0,
        # }

    def label_SBUS(self):  # NOTE: Only 8 channels used
        # TODO: iterate through the list at the start to only decode when all values in, or just wait one second before decoding
        if self.currentPacket:
            try:
                for i in range(len(self.channel_labels)):
                    self.raw_channels[self.channel_labels[i]] = self.currentPacket[i]
            except Exception:
                pass

    def process_SBUS(self):
        # TODO: values used were manually chosen,
        # Improve by sampling first xx seconds and take average to find center value
        if self.raw_channels:
            self.channels["Joy_R_LR"] = (self.raw_channels["Joy_R_LR"] - 967) / 800
            self.channels["Joy_R_UD"] = (self.raw_channels["Joy_R_UD"] - 988) / -800

            # TODO: Joy_L_UD is not correctly scaled
            self.channels["Joy_L_UD"] = (self.raw_channels["Joy_L_UD"] - 987) / 800
            self.channels["Joy_L_LR"] = (self.raw_channels["Joy_L_LR"] - 1146) / 800
            self.channels["SW_B"] = {200: 1, 1000: 0, 1800: -1}.get(
                self.raw_channels["SW_B"], None
            )
            self.channels["SW_A"] = {200: 1, 1800: -1}.get(
                self.raw_channels["SW_A"], None
            )
            self.channels["Vr_B"] = (self.raw_channels["Vr_B"] - 200) / (1800 - 200)
            self.channels["Vr_A"] = (self.raw_channels["Vr_A"] - 200) / (1800 - 200)

            for i in range(len(self.channel_labels)):
                if self.channels[self.channel_labels[i]]:
                    if abs(self.channels[self.channel_labels[i]]) < 0.1:
                        self.channels[self.channel_labels[i]] = 0

    def get_data_stream(self):
        while True:
            self.read_packet()
            self.label_SBUS()
            self.process_SBUS()
            print(self.channels)


class RCPub(Node):

    def __init__(self):
        super().__init__("RCPublisher")
        self.publisher = self.create_publisher(RC, 'rc_channels', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # RC Values
        self.rc_controller = RC_Controller("/dev/ttyUSB0", 100000)
        self.rc_controller.init_connection()

    def timer_callback(self):
        msg = RC()

        self.rc_controller.read_packet()
        self.rc_controller.label_SBUS()
        self.rc_controller.process_SBUS()

        channels = self.rc_controller.channels

        msg.joy_r_lr = channels["Joy_R_LR"]
        msg.joy_r_ud = channels["Joy_R_UD"]
        msg.joy_l_ud = channels["Joy_L_UD"]
        msg.joy_l_lr = channels["Joy_L_LR"]
        msg.swb = channels["SW_B"]
        msg.vr_b = channels["Vr_B"]
        msg.swa = channels["SW_A"]
        msg.vr_a = channels["Vr_A"]

        self.publisher.publish(msg)

        # Log values
        logger = self.get_logger()

        logger.info("Publishing Joystick Right (Left Right) %lf", msg.joy_l_lr)
        logger.info("Publishing Joystick Right (Left Right) %lf", msg.joy_l_ud)
        logger.info("Publishing Joystick Right (Left Right) %lf", msg.joy_r_lr)
        logger.info("Publishing Joystick Right (Left Right) %lf", msg.joy_r_ud)
        logger.info("Publishing Joystick Right (Left Right) %lf", msg.swa)
        logger.info("Publishing Joystick Right (Left Right) %lf", msg.swb)
        logger.info("Publishing Joystick Right (Left Right) %lf", msg.vr_a)
        logger.info("Publishing Joystick Right (Left Right) %lf", msg.vr_b)


def main():
    rclpy.init()

    rc_pub = RCPub()

    rclpy.spin(rc_pub)

    rc_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
