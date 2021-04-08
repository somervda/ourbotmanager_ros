#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from smbus2 import SMBus
import time


class JoyNode(Node): 
    # RPi Channel 1
    channel = 1
    # ADS1115 address and registers
    address = 0x48
    reg_config = 0x01
    reg_conversion = 0x00

    def __init__(self):
        super().__init__("joy_node") 
        self.pub = self.create_publisher(Twist,"base/twist",10)
        # Fire up an asyncronous timer to check for messages from the ft232 on SERIAL_PORT
        self.joy_timer = self.create_timer(0.1 , self.process_joy) 
        self.bus = SMBus(self.channel)
        self.get_logger().info("joy_node has started")

    def process_joy(self):
        # Check the joystick hardware status,send a twist message when needed
        config = [0x83, 0xC3]
        # Start conversion
        self.bus.
        self.bus.write_i2c_block_data(self.address, self.reg_config,config)
        # Wait for conversion
        time.sleep(0.01)
        # Read 16-bit result
        result = self.bus.read_i2c_block_data(self.address, self.reg_conversion, 2)
        self.get_logger().info(f"AO value: {self.tc_to_int(result)}")


    def tc_to_int(self,tcIn):
        # Convert from 2-complement to an integer
        value = ((tcIn[0] & 0xFF) << 8) | (tcIn[1] & 0xFF)
        if value & 0x8000 != 0:
            value -= 1 << 16
        return value
        


def main(args=None):
    rclpy.init(args=args)
    node = JoyNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()