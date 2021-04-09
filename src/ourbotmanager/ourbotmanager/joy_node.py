#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from smbus2 import SMBus
import sys
import time
# Import the ADS1x15 module.
import Adafruit_ADS1x15


class JoyNode(Node): 
    

    exit = False

    def __init__(self):
        super().__init__("joy_node") 
        # Check we can open the ads1115
        try:
            self.adc = Adafruit_ADS1x15.ADS1115( busnum=1)
            self.pub = self.create_publisher(Twist,"base/twist",10)
            # Fire up an asyncronous timer to check for messages from the ft232 on SERIAL_PORT
            self.joy_timer = self.create_timer(0.1 , self.process_joy) 
            self.get_logger().info("joy_node has started")
        except:
            # Note: a permission error can be fixed with a "sudo chmod a+rw /dev/i2c-1"
            self.get_logger().error("joy_node  initialization:"  + str(sys.exc_info()[1]) )
            self.exit = True


    def process_joy(self):
        values = [0]*2
        for i in range(2):
            # Read the specified ADC channel 
            try:
                values[i] = self.adc.read_adc(i, gain=2/3, data_rate=128)
            except:
                self.get_logger().error("Error reading ADC:" + str(sys.exc_info()[0]))
            # My ads1115 needed a little sleep between measuerments to settle on a good value
            time.sleep(0.01)
        self.get_logger().info("Values:" + str(round(values[0] /265)) + ", " + str(round(values[1]/265)) )

def main(args=None):
    rclpy.init(args=args)
    node = JoyNode() 
    if node.exit == False:
        rclpy.spin(node)
    node.get_logger().info("joy_node ending")
    rclpy.shutdown()


if __name__ == "__main__":
    main()