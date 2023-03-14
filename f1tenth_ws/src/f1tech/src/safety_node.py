#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
#from math import isfinite, cos, sin

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        """
        One publisher should publish to the /brake topic with a AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a Bool message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        """
        # self.speed = Odometry().twist.twist.linear
        self.speed = 0
        self.TTC_thresh = 0.001 #0.3 #1 second TTC threshold

        self.init_publishers()
        self.init_subscribers()

    def init_publishers(self):
        # self._brake_publisher = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 10)
        # self._brake_publisher2 = rospy.Publisher("/nav", AckermannDriveStamped, queue_size = 10)
        # self._brake_publisher3 = rospy.Publisher("/vesc/commands/motor/speed", Float64, queue_size = 10)
        # self._brake_publisher4 = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/active", AckermannDriveStamped, queue_size = 10)
        # self._brake_publisher2 = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/default", AckermannDriveStamped, queue_size = 10)
        # self._brake_publisher6 = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/active", AckermannDriveStamped, queue_size = 10)
        self._brake_publisher = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size = 10)
        # self._brake_publisher8 = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 10)
        #self._brake_bool_publisher = rospy.Publisher("/brake_bool", Bool, queue_size = 1)

    def init_subscribers(self):
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size = 10)
        rospy.Subscriber("/vesc/odom", Odometry, self.odom_callback, queue_size = 10)

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        # self.speed = odom_msg.twist.twist.linear
        self.speed = odom_msg.twist.twist.linear.x


        vel = AckermannDriveStamped()
        vel.drive.speed = float(3)
        # print("publishing")
        #self._brake_publisher2.publish(vel)

    def scan_callback(self, scan_msg):
        
        TTC_threshold = 1
        min_TTC = 50
        # vel_x = self.speed.x
        # vel_y = self.speed.y
        vel_x = self.speed
        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        increment = scan_msg.angle_increment
        ranges = np.array(scan_msg.ranges)

        for i in range(len(ranges)):
            #check not nan and not inf
            if np.isfinite(ranges[i]):
                local_dist = ranges[i]
                local_angle = angle_min + increment*i
                # local_derivative = vel_x*np.cos(local_angle) + vel_y*np.sin(local_angle)
                local_derivative = vel_x*np.cos(local_angle)

                # try:
                #     local_TTC = local_dist/local_derivative
                # except ZeroDivisionError:
                #     local_TTC = 0 #min_TTC

                if local_derivative == 0:
                    local_TTC = 0 #min_TTC
                    # print("zerro error hanfled")

                else:
                    local_TTC = local_dist/local_derivative
                
                if (local_derivative > 0) and (local_TTC < min_TTC):
                    min_TTC = local_TTC
            # print("min_TTC = ", min_TTC)
            if min_TTC <= TTC_threshold:
                vel = AckermannDriveStamped()
                vel.drive.speed = float(0)
                # self.drive_pub_.publish(vel)
                # while not rospy.is_shutdown():
                self._brake_publisher.publish(vel)
                print("EMERGENCY BRAKING PROCEDURE INITIATED")

                
        # TODO: calculate TTC
        #first, grab ranges and generate angle vector
	    #print("speed = ", self.speed)
        # if self.speed != 0:
        #     # delay = (rospy.Time.now()- scan_msg.header.stamp).to_sec()
        #     ranges = np.array(scan_msg.ranges)
        #     # ranges = range[np.isfinite(ranges)]
        #     min_angle = scan_msg.angle_min
        #     max_angle = scan_msg.angle_max
        #     increment = scan_msg.angle_increment
        #     angle_vec = np.arange(min_angle, max_angle, increment)
        #     dr = np.cos(angle_vec)*self.speed
        #     denom = np.where(dr < 0, 0.01, dr)
        #     TTC = ranges/denom
        #     rospy.loginfo("Rangemin {}, TTC {}".format(np.min(ranges), np.min(TTC)))
        #     #print("TTC = ", min(TTC))

        #     #Publish brake bool and brake message in emergency
        #     if np.any(TTC < self.TTC_thresh):
        #         ack_msg = AckermannDriveStamped()
        #         ack_msg.drive.speed = 0.0
        #         brake_bool = Bool()
        #         brake_bool.data = True
        #         self._brake_publisher.publish(ack_msg)
        #         #self._brake_bool_publisher.publish(brake_bool)
        #         print("AEB Activated!")
        # else:
        #     brake_bool = Bool()
        #     brake_bool.data = False
        #     #self._brake_bool_publisher.publish(brake_bool)

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()

if __name__ == '__main__':
    print("AEB-Mode Enabled")
    main()