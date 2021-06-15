#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose,Point,Quaternion
# import numpy as np.
import serial
import os
import math

class BaseMotorController(Node):
    VERSION = '1.005'
    SERIAL_PORT = '/dev/ttyUSB0'

    # Update based on actual base specs.
    PWD_MIN = 10 # Minumum PWD supported by base
    LINEAR_TO_PWD = 50 # if linear.x = 2 m/s then set speed (pwd) to 100
    ANGULAR_TO_PWD = 90 # if angular.z = 1 rad/s then set speed (pwd) to 90

    # Keep a track of location and orentation of the base
    # as feedback is returned from the base using meters and radians
    pose_x = 0.0
    pose_y = 0.0
    pose_angle = 0.0

    # Keep track of last serial message type
    lastSerialMessageType = "RB"

    # Conversion factors for converting the base_feedback info that is in motor encoder ticks
    # to meters and radians used by the pose message. Update based on actual base specs.
    TICKS_TO_METERS = 0.00058345
    TICKS_TO_RADIANS = 0.0047276

    # Information recieved from the base about movement : Type & magnitude
    base_movement_info = ""
    
    def __init__(self):
        super().__init__("base_motor_controller") 
        if not os.path.exists(self.SERIAL_PORT):
            self.get_logger().error("Serial Port not found:" + self.SERIAL_PORT + " base_motor_controller not started")
            rclpy.shutdown()
        self.ser = serial.Serial(self.SERIAL_PORT,9600,timeout=0)  # open serial port without blocking
        self.send_serial("") # Send to clear out any noise in the serial buffer 
        self.pub = self.create_publisher(Pose,"base/pose",10)
        self.twist_subscriber = self.create_subscription(Twist,"base/twist",self.send_twist,10)
        # Fire up an asyncronous timer to check for messages from the ft232 on SERIAL_PORT
        self.serial_read_timer = self.create_timer(0.1 , self.read_serial) 
        # Reboot the uController for the new session
        self.send_serial("RB")
        self.lastSerialMessageType = "RB"
        self.get_logger().info("base_motor_controller has started: " +  self.VERSION)

    def send_twist(self,msg):
        # self.get_logger().info("Twist: Linear velocity: %f Angular velocity: %f" % (msg.linear.x, msg.angular.z))
        if msg.linear.x != 0 :
            # We are making a linear movement
            direction = 1
            if self.lastSerialMessageType == "TV":
                self.send_serial("SP")
                self.lastSerialMessageType = "SP"
            # Set pwd and adjust to limits
            if msg.linear.x < 0:
                direction = -1
            pwd = int(msg.linear.x * self.LINEAR_TO_PWD)
            if abs(pwd) < self.PWD_MIN :
                pwd = self.PWD_MIN * direction
            if abs(pwd) > 100 :
                pwd = 100 * direction
            # self.get_logger().info("sending serial:" + "RV" +  "%+4d" % pwd)
            self.send_serial("RV" +  "%+4d" % pwd)
            self.lastSerialMessageType = "RV"
        elif msg.angular.z != 0 :
            # We are making a turn
            direction = 1
            if self.lastSerialMessageType == "RV":
                self.send_serial("SP")
                self.lastSerialMessageType = "SP"
            # Set pwd and adjust to limits
            if msg.angular.x < 0 :
                direction = -1
            pwd = int(msg.angular.z * self.ANGULAR_TO_PWD)
            if abs(pwd) < self.PWD_MIN :
                pwd = self.PWD_MIN * direction
            if abs(pwd) > 100 :
                pwd = 100 * direction
            # self.get_logger().info("sending serial:" + "TV" +  "%+4d" % pwd)
            self.send_serial("TV" +  "%+4d" % pwd)
            self.lastSerialMessageType = "TV"
        else:
            # self.get_logger().info("sending serial:" + "SP")
            self.send_serial("SP")
            self.lastSerialMessageType = "SP"

    def send_serial(self,send):
        self.ser.write((send + "\n").encode())

    def read_serial(self):
        readChr = self.ser.read(1).decode()
        while len(readChr) > 0 :
            if readChr == "\n":
                # Once newline is seen, process the movement info recieved from the base
                self.get_logger().info(self.base_movement_info)
                self.process_pose()
                self.base_movement_info = ""
            elif readChr == "\x00":
                # Usually see one of these in the queue after the uController is reset
                # Ignore these characters
                pass
            else:
                # Build up base_movement_info character by character
                self.base_movement_info += readChr
            readChr = self.ser.read(1).decode()
    
    def process_pose(self):
        # Calculate location and angle of the base from the base movement_info i.e R+00020 
        # and publish as a Plot message
        if len(self.base_movement_info) != 7:
            self.get_logger().error("Misformed base_feedback message:" + self.base_movement_info)
        else:
            baseMode = self.base_movement_info[0:1]
            if baseMode not in ["R","T"]:
                self.get_logger().error("Misformed base_feedback baseMode :" + baseMode)
            else:
                ticks_string = self.base_movement_info[1:7]
                if not ticks_string.isnumeric:
                    self.get_logger().error("Misformed base_feedback ticks value :" + ticks_string)
                else:
                    ticks = int(ticks_string)
                    if baseMode == "T":
                        self.pose_angle += (self.TICKS_TO_RADIANS * ticks)
                        # Clean up radians so not more than 360 degrees (2pi) is reported.
                        self.pose_angle %= (2 * math.pi)
                        self.publish_Pose()
                    else:
                        # Calculate pose_x and pose_y co-ordinants based on current location, pose_angle and distance moved
                        self.pose_x += (self.TICKS_TO_METERS * ticks) * math.cos(self.pose_angle)
                        self.pose_y += (self.TICKS_TO_METERS * ticks) * math.sin(self.pose_angle)
                        self.get_logger().info("cos:" + str(math.cos(self.pose_angle)) + " sin:" + str(math.sin(self.pose_angle))) 
                        self.publish_Pose()
                        
    def publish_Pose(self):
        self.get_logger().info("pose_x:" + str(self.pose_x) + " pose_y:" + str(self.pose_y) + " pose_angle:" + str(self.pose_angle))
        # Convert plot_angle to a quaternion for a pose message see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        # plot_angle is equivalent to yaw.
        pose_msg = Pose()
        pose_msg.position.x = self.pose_x
        pose_msg.position.y = self.pose_y
        pose_msg.position.z = 0.0
        my_quaternion = self.euler_to_quaternion(self.pose_angle,0.0,0.0)
        pose_msg.orientation.x = my_quaternion[0]
        pose_msg.orientation.y = my_quaternion[1]
        pose_msg.orientation.z = my_quaternion[2]
        pose_msg.orientation.w = my_quaternion[3]
        self.pub.publish(pose_msg)

    def euler_to_quaternion(self,yaw, pitch, roll):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = BaseMotorController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
