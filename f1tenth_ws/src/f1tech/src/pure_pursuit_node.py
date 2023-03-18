#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

import math, os, csv, tf
import numpy as np
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from visualization_msgs.msg import Marker, MarkerArray

# vehicle physical parameters
WHEELBASE_LEN = 0.325

# lookahead parameters
ANG_LOOKAHEAD_DIST = 3

LOOKAHEAD_DISTANCE = 0.6# 1.20
KP = 0.3

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        self.trajectory_name = 'straight_blanchin'
        self.plan = self.construct_path()
        
        drive_topic = '/vesc/ackermann_cmd_mux/input/navigation'
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)

        pf_odom_topic = '/pf/pose/odom'
        self.pf_odom_sub = rospy.Subscriber(pf_odom_topic, Odometry, self.odom_callback, queue_size=10)

        wp_viz_topic = '/wp_viz'
        self.nearest_wp_pub = rospy.Publisher(wp_viz_topic+'/nearest', Marker, queue_size = 10)
        self.all_wp_pub = rospy.Publisher(wp_viz_topic+'/all', MarkerArray, queue_size = 10)

        #pf_inferred_pose_topic = '/pf/viz/inferred_pose'
        #self.pf_pose_sub = rospy.Subscriber(pf_inferred_pose_topic, PoseStamped, self.lidar_callback, queue_size=10)

    def odom_callback(self, odom_msg):
        """ Callback function for the subcriber of the localization-inferred odometry """
        # Show all waypoints
        markerArray =self.create_waypoint_marker_array()
        self.all_wp_pub.publish(markerArray)
        
        # Find nearest waypoint
        curr_x = odom_msg.pose.pose.position.x
        curr_y = odom_msg.pose.pose.position.y
        nearest_waypoint_idx = self.find_nearest_point(curr_x, curr_y)

        lookahead_waypoint_idx = nearest_waypoint_idx
        while math.sqrt(math.pow(self.plan[lookahead_waypoint_idx][0]-curr_x,2) + math.pow(self.plan[lookahead_waypoint_idx][1]-curr_y,2)) < LOOKAHEAD_DISTANCE:
            lookahead_waypoint_idx += 1
            if (lookahead_waypoint_idx > len(self.plan) - 1):
                lookahead_waypoint_idx = 0
        print(lookahead_waypoint_idx, nearest_waypoint_idx)
        
        # Choose next waypoint to pursue (lookahead)
        plan_size = len(self.plan)
        #lookahead_waypoint_idx = (nearest_waypoint_idx + ANG_LOOKAHEAD_DIST) % plan_size
        marker = self.create_waypoint_marker(lookahead_waypoint_idx, nearest_wp=True)
        self.nearest_wp_pub.publish(marker)

        # Prepare the drive command for pursuing that waypoint
        drive_msg = AckermannDriveStamped()

        heading = tf.transformations.euler_from_quaternion((odom_msg.pose.pose.orientation.x,
                                                        odom_msg.pose.pose.orientation.y,
                                                        odom_msg.pose.pose.orientation.z,
                                                        odom_msg.pose.pose.orientation.w))[2]

        goal_x = self.plan[lookahead_waypoint_idx][0] # need to implement lookahead
        goal_y = self.plan[lookahead_waypoint_idx][1] # need to implement lookahead
        eucl_d = math.sqrt(math.pow(goal_x - curr_x, 2) + math.pow(goal_y - curr_y, 2))

        lookahead_angle = math.atan2(goal_y - curr_y, goal_x - curr_x)
        print(lookahead_angle)
        del_y = eucl_d * math.sin(lookahead_angle - heading)

        #curvature = math.degrees(2.0*(abs(goal_x) - abs(curr_x))/(math.pow(eucl_d, 2)))
        #curvature = math.degrees(2.0*del_y/(math.pow(eucl_d, 2)))
        curvature = 2.0*del_y/(math.pow(eucl_d, 2))
        print(curvature)
        #steering_angle = math.atan(curvature * WHEELBASE_LEN)
        steering_angle = KP * curvature
        while (steering_angle > np.pi/2) or (steering_angle < -np.pi/2):
            if steering_angle > np.pi/2:
                steering_angle -= np.pi
            elif steering_angle < -np.pi/2:
                steering_angle += np.pi
        print(steering_angle)

        """
        theta = math.atan2(ang_goal_y - curr_y, ang_goal_x - curr_x)

        proj_x = eucl_d * math.cos(heading) + curr_x
        proj_y = eucl_d * math.sin(heading) + curr_y

        proj_eucl_shift = math.sqrt(math.pow(proj_x - ang_goal_x, 2) + math.pow(proj_y - ang_goal_y, 2))

        angle_error = math.acos(1 - (math.pow(proj_eucl_shift, 2)/(2 * math.pow(eucl_d, 2))))
        angle_error = math.degrees(angle_error)

        goal_sector = (ang_goal_x - curr_x)*(proj_y - curr_y) - (ang_goal_y - curr_y)*(proj_x - curr_x)

        if goal_sector > 0:
            goal_sector = GOAL_RIGHT
        elif goal_sector < 0:
            goal_sector = GOAL_LEFT
        else:
            goal_sector = GOAL_ON_LINE

        if goal_sector == GOAL_ON_LINE:
            msg = MSG_A.format(round(eucl_d, 2))
        else:
            msg = MSG_B.format(round(eucl_d, 2), round(angle_error, 2), goal_sector)

        # full P-control for angle and speed

        if angle_error > ANGLE_RANGE_A:
            angle_error = ANGLE_RANGE_A

        drive_msg.drive.steering_angle = angle_error/ANGLE_RANGE_A
        # drive_msg.drive.speed = 1.0 - (angle_error/ANGLE_RANGE_A) # * (SPEED_TURN_MIN - SPEED_TURN_MAX)
        # drive_msg.drive.speed = drive_msg.drive.speed + SPEED_TURN_MAX

        if goal_sector == GOAL_RIGHT:
            drive_msg.drive.steering_angle = -1.0 * drive_msg.drive.steering_angle

        # velocity control node

        vel_eucl_d = math.sqrt(math.pow(vel_goal_x - curr_x, 2) + math.pow(vel_goal_y - curr_y, 2))

        if use_ackermann_model == 'true':
            drive_msg.drive.speed = (vel_eucl_d/(MAX_VEL_GOAL_DIST - WHEELBASE_LEN)) * SPEED_TURN_MIN
        else:
            drive_msg.drive.speed = (vel_eucl_d/MAX_VEL_GOAL_DIST) * SPEED_TURN_MIN

        if adaptive_lookahead == 'true':
            if lookahead_state == 'brake':
                drive_msg.drive.speed = SCALE_VEL_BRAKE * drive_msg.drive.speed
            elif lookahead_state == 'caution':
                drive_msg.drive.speed = SCALE_VEL_CAUTION * drive_msg.drive.speed
            else:
                drive_msg.drive.speed = SCALE_VEL_UNRESTRICTED * drive_msg.drive.speed
        else:
            drive_msg.drive.speed = SCALE_VEL_NO_ADAPTIVE_LOOKAHEAD * drive_msg.drive.speed

        if drive_msg.drive.speed < SPEED_TURN_MAX:
            drive_msg.drive.speed = SPEED_TURN_MAX
        if drive_msg.drive.speed > SPEED_TURN_MIN:
            drive_msg.drive.speed = SPEED_TURN_MIN
        """

        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        #drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        sa_deg = np.rad2deg(steering_angle)
        print("Steering Angle: {:.3f} [deg]".format(sa_deg))
        drive_msg.drive.speed = self.get_velocity(steering_angle)
        self.drive_pub.publish(drive_msg)

    def create_waypoint_marker(self, waypoint_idx, nearest_wp=False):
        """Given the index of the nearest waypoint, publishes the necessary Marker data to the 'wp_viz' topic for RViZ visualization"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = 2 # sphere
        marker.action = 0 # add the marker
        marker.pose.position.x = self.plan[waypoint_idx][0]
        marker.pose.position.y = self.plan[waypoint_idx][1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        
        # different color/size for the whole waypoint array and the nearest waypoint
        if nearest_wp:
            marker.scale.x *= 2
            marker.scale.y *= 2
            marker.scale.z *= 2
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        return marker

    def create_waypoint_marker_array(self):
        markerArray = MarkerArray()
        for i in range(len(self.plan)):
            marker = self.create_waypoint_marker(i)
            markerArray.markers.append(marker)
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        return markerArray

    def get_velocity(self, steering_angle):
        """ Given the desired steering angle, returns the appropriate velocity to publish to the car """
        slow = True
        if slow == True:
            return 0.8
        if abs(steering_angle) < np.deg2rad(10):
            velocity = 1.2 #2.6 #2.5
        elif abs(steering_angle) < np.deg2rad(20):
            velocity = 1.0 #2.2
        else:
            velocity = 0.8 #1.4
        return velocity

    def find_nearest_point(self, curr_x, curr_y):
        """ Given the current XY position of the car, returns the index of the nearest waypoint """
        ranges = []
        for index in range(0, len(self.plan)):
            eucl_x = math.pow(curr_x - self.plan[index][0], 2)
            eucl_y = math.pow(curr_y - self.plan[index][1], 2)
            eucl_d = math.sqrt(eucl_x + eucl_y)
            ranges.append(eucl_d)
        return (ranges.index(min(ranges)))

    def construct_path(self):
        """ Reads waypoint data from the .csv file, inserts it into an array called 'plan' """
        file_path = os.path.expanduser('~/f1tenth_ws/logs/{}.csv'.format(self.trajectory_name))
        plan = []
        with open(file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter = ',')
            for waypoint in csv_reader:
                plan.append(waypoint)
        for index in range(0, len(plan)):
            for point in range(0, len(plan[index])):
                plan[index][point] = float(plan[index][point])
        return plan

def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()

if __name__ == '__main__':
    print("Pure Pursuit Mode Initialized...")
    main()