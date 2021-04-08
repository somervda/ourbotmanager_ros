#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from smbus2 import SMBus
import time
# Import the ADS1x15 module.
import Adafruit_ADS1x15


class JoyNode(Node): 
    
    GAIN = 1

    def __init__(self):
        super().__init__("joy_node") 
        try:
            self.adc = Adafruit_ADS1x15.ADS1115( busnum=1)
        except:
            self.get_logger().error("Unexpected error opening adc:", sys.exc_info()[0])
            rclpy.shutdown()
        self.pub = self.create_publisher(Twist,"base/twist",10)
        # Fire up an asyncronous timer to check for messages from the ft232 on SERIAL_PORT
        self.joy_timer = self.create_timer(1.0 , self.process_joy) 
        self.get_logger().info("joy_node has started")

    def process_joy(self):
        values = [0]*4
        for i in range(4):
            # Read the specified ADC channel using the previously set gain value.
            try:
                values[i] = self.adc.read_adc(i, gain=self.GAIN)
            except:
                self.get_logger().error("Error reading ADC")
            # Note you can also pass in an optional data_rate parameter that controls
            # the ADC conversion time (in samples/second). Each chip has a different
            # set of allowed data rate values, see datasheet Table 9 config register
            # DR bit values.
            #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
            # Each value will be a 12 or 16 bit signed integer value depending on the
            # ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
        self.get_logger().info("Values:" + str(values[0]) + ", " + str(values[1]))

def main(args=None):
    rclpy.init(args=args)
    node = JoyNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()