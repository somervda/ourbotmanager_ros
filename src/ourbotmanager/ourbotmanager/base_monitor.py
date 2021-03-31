#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose,Point,Quaternion
import serial
import os


class BaseMonitor(Node): 
    SERIAL_PORT = '/dev/ttyUSB0'
    motionInfo = ""

    def __init__(self):
        super().__init__("base_monitor") 
        self.pub = self.create_publisher(Pose,"base/pose",10)
        if not os.path.exists(self.SERIAL_PORT):
            self.get_logger().error("Serial Port not found:" + self.SERIAL_PORT + " base_motor_controller not started")
            rclpy.shutdown()
        self.ser = serial.Serial(self.SERIAL_PORT,9600, timeout=0)  # open serial port in non-blocking mode
        # Fire up an asyncronous timer to check for messages from the ft232 on SERIAL_PORT
        self.read_timer = self.create_timer(0.05 , self.read_serial) 
        self.get_logger().info("base_motor_controller has started")        

    def read_serial(self):
        readChr = self.ser.read(1).decode('utf-8')
        while len(readChr) > 0 :
            if readChr == "\n":
                self.get_logger().info(self.motionInfo)
                self.motionInfo = ""
            else:
                self.motionInfo += readChr
            readChr = self.ser.read(1).decode('utf-8')

def main(args=None):
    rclpy.init(args=args)
    node = BaseMonitor() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()