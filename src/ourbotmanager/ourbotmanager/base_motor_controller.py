#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose,Point,Quaternion
import serial
import os
import math

class BaseMotorController(Node):
    SERIAL_PORT = '/dev/ttyUSB0'

    PWD_MIN = 10 # Minumum PWD supported by base
    LINEAR_TO_PWD = 100 # if linear.x = 1 m/s then set speed (pwd) to 100
    ANGULAR_TO_PWD = 90 # if angular.z = 1 rad/s then set speed (pwd) to 100

    # Keep a running plot of location and orentation of the base
    # as feedback is returned from the base using meters and radians
    plot_x = 0
    plot_y = 0
    plot_angle = 0

    # Conversion factors for converting the base_feedback info that is in motor encoder ticks
    # to meters and radians used by the plot message
    TICKS_TO_METERS = 0.001
    TICKS_TO_RADIANS = 0.01

    base_feedback = ""
    
    def __init__(self):
        super().__init__("base_motor_controller") 
        if not os.path.exists(self.SERIAL_PORT):
            self.get_logger().error("Serial Port not found:" + self.SERIAL_PORT + " base_motor_controller not started")
            rclpy.shutdown()
        self.ser = serial.Serial(self.SERIAL_PORT,9600,timeout=0)  # open serial port without blocking
        self.send_serial("") # Send to clear out any noise in the serial buffer 
        self.pub = self.create_publisher(Pose,"base/pose",10)
        self.twist_subscriber = self.create_subscription(Twist,"base/cmd_vel",self.send_cmd_vel,10)
        # Fire up an asyncronous timer to check for messages from the ft232 on SERIAL_PORT
        self.read_timer = self.create_timer(0.05 , self.read_serial) 
        self.get_logger().info("base_motor_controller has started")

    def send_cmd_vel(self,msg):
        self.get_logger().info("Twist: Linear velocity: %f Angular velocity: %f" % (msg.linear.x, msg.angular.z))
        if msg.linear.x != 0 :
            # We are making a linear movement
            direction = 1
            # Set pwd and adjust to limits
            if msg.linear.x < 0:
                direction = -1
            pwd = int(msg.linear.x * self.LINEAR_TO_PWD)
            if abs(pwd) < self.PWD_MIN :
                pwd = self.PWD_MIN * direction
            if abs(pwd) > 100 :
                pwd = 100 * direction
            self.get_logger().info("sending serial:" + "RV" +  "%+4d" % pwd)
            self.send_serial("RV" +  "%+4d" % pwd)
        elif msg.angular.z != 0 :
            # We are making a turn
            direction = 1
            # Set pwd and adjust to limits
            if msg.angular.x < 0 :
                direction = -1
            pwd = int(msg.angular.z * self.ANGULAR_TO_PWD)
            if abs(pwd) < self.PWD_MIN :
                pwd = self.PWD_MIN * direction
            if abs(pwd) > 100 :
                pwd = 100 * direction
            self.get_logger().info("sending serial:" + "TV" +  "%+4d" % pwd)
            self.send_serial("TV" +  "%+4d" % pwd)
        else:
            self.get_logger().info("sending serial:" + "SP")
            self.send_serial("SP")

    def send_serial(self,send):
        self.ser.write((send + "\n").encode())

    def read_serial(self):
        readChr = self.ser.read(1).decode('utf-8')
        while len(readChr) > 0 :
            if readChr == "\n":
                self.get_logger().info(self.base_feedback)
                self.process_plot()
                self.base_feedback = ""
            else:
                self.base_feedback += readChr
            readChr = self.ser.read(1).decode('utf-8')
    
    def process_plot(self):
        # Calculate location and angle of the base from the base feedback 
        # and publish as a Plot message
        if len(self.base_feedback) != 7:
            self.get_logger().error("Misformed base_feedback message:" + self.base_feedback)
        else:
            baseMode = self.base_feedback[0:1]
            if baseMode not in ["R","T"]:
                self.get_logger().error("Misformed base_feedback baseMode :" + baseMode)
            else:
                ticks_string = self.base_feedback[1:7]
                if not ticks_string.isnumeric:
                    self.get_logger().error("Misformed base_feedback ticks value :" + ticks_string)
                else:
                    ticks = int(ticks_string)
                    if baseMode == "T":
                        self.plot_angle += (self.TICKS_TO_RADIANS * ticks)
                        self.publish_Plot()
                    else:
                        # Calculate plot_x and plot_y co-ordinants based on current location, plot_angle and distance moved
                        self.plot_x += (self.TICKS_TO_METERS * ticks) * math.cos(self.plot_angle)
                        self.plot_y += (self.TICKS_TO_METERS * ticks) * math.sin(self.plot_angle)
                        self.publish_Plot()
                        
    def publish_Plot(self):
        self.get_logger().info("plot_x:" + str(self.plot_x) + " plot_y:" + str(self.plot_y) + " plot_angle:" + str(self.plot_angle))
        

def main(args=None):
    rclpy.init(args=args)
    node = BaseMotorController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()