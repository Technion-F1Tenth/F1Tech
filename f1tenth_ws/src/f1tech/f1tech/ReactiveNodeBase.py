#!/usr/bin/env python3
import math
#import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
#SLOW_MODE = True
#MAX_SPEED = 6.2
#MIN_SPEED = 1.0

class ReactiveNodeBase(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('ReactiveNodeBase')
        
        ## Topics
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        ## Subs & Pubs
        self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        
        ## Parameters
        self.current_steering_angle = 0.0 # variable

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message """
        proc_ranges = self.preprocess_lidar(data.ranges)
        self.prepare_and_send_drive_msg()

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = ranges
        return proc_ranges
    
    def prepare_and_send_drive_msg(self):
        ackermann_message = AckermannDriveStamped()
        ackermann_message.drive.steering_angle = self.current_steering_angle
        ackermann_message.drive.speed = self.get_velocity(self.current_steering_angle)
        self.drive_publisher.publish(ackermann_message)

    def get_velocity(self, steering_angle):
        velocity = 0.
        return velocity