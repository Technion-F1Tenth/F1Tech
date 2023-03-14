#!/usr/bin/env python
from __future__ import print_function
import sys
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
servo_offset = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 1
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = "/vesc/ackermann_cmd_mux/input/navigation"

        self.lidar_sub_ = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=10)
        self.drive_pub_ = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

        #PID CONTROL PARAMS
        self.kp = 1
        self.kd = 0.2
        self.ki = 0.5
        self.integral = 0
        self.prev_error = 0
        self.error = 0

        self.time_current = 0
        self.time_previous = 0
        self.following_distance = 1.3#requested distance to wall
        self.lookahead_param = 0.5 #how much our car is projected to move forward between servo commands

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        angle_min = -2.3499999046325684
        angle_max = 2.3499999046325684
        angle_increment = 0.004351851996034384

        # angle_rad = np.deg2rad(angle)
        angle_rad = angle

        index = int(np.floor((angle_rad-angle_min)/angle_increment))
        try:
            return data[index]

        except IndexError as e:
            print("length of dists = ", len(data), " and you're asking for index ", index, " !")

        return None

    def pid_control(self, error, velocity):
        angle = -(self.kp*self.error + self.ki*self.integral + self.kd*(self.error-self.prev_error)/(self.time_current-self.time_previous))
       
        if angle > np.deg2rad(45):
            angle = np.deg2rad(45)
        elif angle < -np.deg2rad(45):
            angle = -np.deg2rad(45)
        
        print("angle = ", np.rad2deg(angle))
        if abs(np.rad2deg(angle)) < 10:
            velocity = 1.
        elif (abs(np.rad2deg(angle)) > 10 and abs(np.rad2deg(angle)) < 30):
            velocity = 1.
        else:
            velocity = 0.7
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub_.publish(drive_msg)

    def followLeft(self, range_data, leftDist):
        #Follow left wall as per the algorithm 
        angle_b = np.deg2rad(90 - 0)
        angle_a = np.deg2rad(90 - 45)

        while (not np.isfinite(self.getRange(range_data, angle_a))) or (angle_a == angle_b):
            angle_a += 0.01

        while (not np.isfinite(self.getRange(range_data, angle_b))) or (angle_a == angle_b):
            angle_b += 0.01

        theta = -(angle_a-angle_b)
        range_a = self.getRange(range_data, angle_a)
        range_b = self.getRange(range_data, angle_b)

        num = range_a*np.cos(theta) - range_b
        dem = range_a*np.sin(theta)

        alpha = np.arctan2(num,dem)
        D_t = range_b*np.cos(alpha)
        
        L = self.lookahead_param
        D_t_plus_1 = D_t + L*np.sin(alpha)

        error = self.following_distance - D_t_plus_1
        self.prev_error = self.error
        self.error = error
        self.time_previous = self.time_current
        self.time_current = time.time()
        time_diff = self.time_current - self.time_previous
        self.integral = self.prev_error*(time_diff)
        return 0.0 

    def lidar_callback(self, msg):
        error = self.followLeft(msg.ranges, self.following_distance)
        velocity = VELOCITY
        self.pid_control(error, velocity)

def main():
    rospy.init_node("wf_node", anonymous=True)
    wf = WallFollow()
    rospy.spin()

if __name__=='__main__':
    print("Wall-Following Mode Initialized...")
    main()