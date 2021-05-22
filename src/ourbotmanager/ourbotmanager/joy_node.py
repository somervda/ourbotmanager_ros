#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from smbus2 import SMBus
import sys
import time
# Import the ADS1x15 module ..
import Adafruit_ADS1x15
from gpiozero import LED


class JoyNode(Node): 
    
    rosRunLED = LED(13)
    exit = False
    # Range of values to treat as a zero value for the joystick
    ZERO_RANGE_MIN = -10
    ZERO_RANGE_MAX = +10
    # Use polatity to change if values increase or decrease going left to right or up to down
    # Useful so you arn't dependent on how you wired up the joystick potentiometers
    X_RANGE_POLARITY = -1
    Y_RANGE_POLARITY = +1
    # Granularity is the number of distinct values between -100 and +100 to support
    # 20 will give you 10 distict positive and 10 negitive values
    GRANULARITY = 10
    # Keep track of last joystick values
    last_value_x = 0
    last_value_y = 0
    # Scaling factors for converting joystick values to twist magnitudes
    TWIST_LINEAR_SCALING = 0.02  # joystick 100 = twist 2ms velocity
    TWIST_ANGULAR_SCALING = 0.01  # joystick 100 = twist 1 radians/sec velocity

    
    # Republsh every n number of process_joy cycles to make sure base got last value (Might remove this)
    PUBLISH_CYCLES = 10
    republish_counter = PUBLISH_CYCLES
    

    def __init__(self):
        super().__init__("joy_node") 
        # Check we can open the ads1115
        try:
            self.adc = Adafruit_ADS1x15.ADS1115( busnum=1)
            self.pub = self.create_publisher(Twist,"base/twist",10)
            # Fire up an asyncronous timer to check for messages from the ft232 on SERIAL_PORT
            self.joy_timer = self.create_timer(0.1 , self.process_joy) 
            self.rosRunLED.on()
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
        # Convert to a value bettween 0 and 200 and number of distict values set by the granularity
        value_x = (round(((round(values[0]/132) - 100) * self.X_RANGE_POLARITY) / self.GRANULARITY)) * self.GRANULARITY
        value_y = (round(((round(values[1]/132) - 100) * self.Y_RANGE_POLARITY) / self.GRANULARITY)) * self.GRANULARITY
        if value_x <= self.ZERO_RANGE_MAX and value_x >= self.ZERO_RANGE_MIN :
            value_x = 0
        if value_y <= self.ZERO_RANGE_MAX and value_y >= self.ZERO_RANGE_MIN :
            value_y = 0
        self.get_logger().info("Values:" + str(value_x) + ", " + str(value_y) )
        # Only output a twist message when the joystick values change,
        # output only the larger of the linear or angular values (Not both at the same time)
        # If nothing published for N seconds then send the last value again.
        doPublish = False
        msg = Twist()
        self.republish_counter -= 1
        if abs(value_x) > abs(value_y): 
            # Deal with x
            msg.angular.z = value_x * self.TWIST_ANGULAR_SCALING
            if self.last_value_x != value_x:
                # Change in x value
                doPublish = True
        elif abs(value_x) < abs(value_y):
            # Deal with y
            msg.linear.x = value_y * self.TWIST_LINEAR_SCALING
            if self.last_value_y != value_y:
                # Change in y value
                doPublish = True
        else :
            # Deal with joystick back to zero values
            msg.linear.x = 0.0
            msg.angular.y = 0.0
            if self.last_value_y != 0 or self.last_value_x != 0:
                doPublish = True
        if doPublish or self.republish_counter == 0:
            self.republish_counter = self.PUBLISH_CYCLES
            self.pub.publish(msg)
        self.last_value_x = value_x
        self.last_value_y = value_y
             

def main(args=None):
    rclpy.init(args=args)
    node = JoyNode() 
    if node.exit == False:
        rclpy.spin(node)
    node.get_logger().info("joy_node ending")
    node.rosRunLED.off()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
