#!/usr/bin/env python
import numpy as np
from numpy import linalg as LA
import math, time

import rospy, tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
#from tf import transform_listener

# For occupancy grid, lets try it
#import range_libc
#from nav_msgs.srv import GetMap
#
#import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker#, MarkerArray

# TODO: import as you need
from RRTTree import RRTTree
from RRTMotionPlanner import RRTMotionPlanner
from rrt_pp import PurePursuit

class RRT(object):
    def __init__(self):
        pf_topic = '/pf/pose/odom'
        scan_topic = '/scan'
        drive_topic = '/vesc/ackermann_cmd_mux/input/navigation'
        wp_viz_topic = '/wp_viz'

        self.occ_grid = None
        self.rotMatrix = None
        #rospy.Subscriber(pf_topic, PoseStamped, self.pf_callback)
        self.occ_pub = rospy.Publisher('/occ_grid', OccupancyGrid, queue_size = 10)
        self.width = 40
        self.height = 40
        self.resolution = 0.05
        #self.footprint = 0.3 # square size of the car footprint [m]
        #self.car_pose = tf.TransformListener() # listener of transforms between the car_base_link and the world frame
        #self.rate = 5.0 # Map update rate (defaulted to 5 Hz)

        #self.curr_pos = [0,0]
        #self.curr_orientation = [0,0,0,1]
        self.curr_pos = None
        self.curr_orientation = None
        self.drive_msg = None

        """
        self.curr_pos = None
        self.curr_orientation = None
        
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, callback=self.set_initial_position)
        rospy.Subscriber('/pose', PoseWithCovarianceStamped, callback=self.set_initial_position)
        
        print("Waiting for an initial position...")
        while self.curr_pos is None:
            continue
        """
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)

        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        #self.pf_odom_sub = 
        rospy.Subscriber(pf_topic, Odometry, self.pf_callback, queue_size=10)

        self.nearest_wp_pub = rospy.Publisher(wp_viz_topic+'/nearest', Marker, queue_size = 10)
        self.goal_wp_pub = rospy.Publisher(wp_viz_topic+'/goal', Marker, queue_size = 10)
        self.plan_pub = rospy.Publisher(wp_viz_topic+'/plan', Marker, queue_size = 10)
    
    #def set_initial_position(self, msg):
    #    initial_pose = msg.pose.pose
    #    if self.curr_pos is None or self.curr_orientation is None:
    #        self.curr_pos = [initial_pose.position.x, initial_pose.position.y]
    #        self.curr_orientation = [initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z, initial_pose.orientation.w]

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:
        
        """
        # Create a local occupancy grid for the car, where the car's current location is at (x,y)=(0, WIDTH/2)
        grid = np.ndarray((self.width, self.height), buffer=np.zeros((self.width, self.height), dtype=np.int), dtype=np.int)
        grid.fill(int(-1))

        angle_min = scan_msg.angle_min
        angle_max = scan_msg.angle_max
        #print(angle_min, angle_max)
        dist = list(scan_msg.ranges)
        #dist.reverse() # laser frame
        angles = np.linspace(angle_min, angle_max, num = len(dist)) # laser frame

        #t = self.car_pose.getLatestCommonTime('/base_link', '/map')
        #position, quaternion = self.car_pose.lookupTransform('/map', '/base_link', t)
        
        #t = self.car_pose.getLatestCommonTime('/base_link', '/laser')
        #lidar_position, _ = self.car_pose.lookupTransform('/base_link', '/laser', t)
        
        #position = self.curr_pos
        #quaternion = self.curr_orientation
        #self.set_free_cells(grid, position, int(self.footprint//self.resolution))
        #print(self.curr_pos)

        #euler = tf.transformations.euler_from_quaternion(self.curr_orientation)
        #self.rotMatrix = np.array([[np.cos(euler[2]), -np.sin(euler[2])], [np.sin(euler[2]), np.cos(euler[2])]])
        if self.rotMatrix is None:
            return

        new_origin = np.dot(self.rotMatrix, np.array([0, - self.height // 2 * self.resolution])) + [self.curr_pos[0], self.curr_pos[1]]

        self.set_obstacle(grid, angles, dist)
        map_msg = OccupancyGrid()
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = rospy.Time.now()
        map_msg.info.resolution = self.resolution
        map_msg.info.width = self.width
        map_msg.info.height = self.height
        #map_msg.info.origin.position.x = self.curr_pos[0]# - self.width // 2 * self.resolution
        #map_msg.info.origin.position.y = self.curr_pos[1] - self.height // 2 * self.resolution

        #new_origin = [self.curr_pos[0] - (-self.height // 2 * self.resolution)*np.sin(euler[2]), self.curr_pos[1] + (-self.height // 2 * self.resolution)*np.cos(euler[2])]
        #print(new_origin)

        #print(np.dot(rotMatrix, np.array([0, + self.height // 2 * self.resolution])))
        #temp = np.dot(rotMatrix, np.array([0, + self.height // 2 * self.resolution])) + [self.curr_pos[0], self.curr_pos[1]]
        #print(temp)
        map_msg.info.origin.position.x = new_origin[0]
        map_msg.info.origin.position.y = new_origin[1]

        map_msg.info.origin.orientation.x = self.curr_orientation[0]
        map_msg.info.origin.orientation.y = self.curr_orientation[1]
        map_msg.info.origin.orientation.z = self.curr_orientation[2]
        map_msg.info.origin.orientation.w = self.curr_orientation[3]
        map_msg.data = range(self.width*self.height)
        for i in range(self.width*self.height):
            map_msg.data[i] = grid.flat[i]  
        self.occ_grid = map_msg
        self.occ_pub.publish(map_msg)

        if self.drive_msg is not None:
            self.drive_pub.publish(self.drive_msg)

    def set_obstacle(self, grid, angles, dist, lidar_position=0.265):
        for d in range(len(dist)):
            x = dist[d]*np.cos(angles[d]) # x coordinate in lidar frame
            y = dist[d]*np.sin(angles[d]) # y coordinate in lidar frame
            x -= lidar_position # x coordinate in base link frame
            #print(x,y)
            #print(x,y)
            #y -= lidar_position[1] # y coordinate in base link frame
            #if abs(y) < (self.width*self.resolution // 2) and abs(x) < (self.height*self.resolution): # obstacle is within the grid
            if abs(y) < (self.width*self.resolution / 2) and abs(x) < (self.height*self.resolution): #/ 2
                #print('yay')
                #obstacle = np.dot(self.rotMatrix, np.array([0, - self.height // 2])) + [y//self.resolution, x//self.resolution]
                #obstacle = np.dot(self.rotMatrix, np.array([0, - self.height // 2])) + [y//self.resolution, x//self.resolution]
                
                #obstacle = [x//self.resolution- self.height // 2, y//self.resolution]
                obstacle = [y//self.resolution- self.height // 2, x//self.resolution]
                #print(int(obstacle[0]),int(obstacle[1]))
                #obstacle = [y//self.resolution - self.height//2, x//self.resolution]
                if int(obstacle[0]) < self.height and int(obstacle[1]) < self.width:
                    grid[int(obstacle[0]), int(obstacle[1])] = int(100)
                
                if int(obstacle[0]+1) < self.height  and int(obstacle[1]) < self.width:
                    if grid[int(obstacle[0]+1), int(obstacle[1])] < int(1):
                        grid[int(obstacle[0]+1), int(obstacle[1])] = int(50)
                if int(obstacle[0]) < self.height and int(obstacle[1]+1) < self.width:
                    if  grid[int(obstacle[0]), int(obstacle[1]+1)] < int(1):
                        grid[int(obstacle[0]), int(obstacle[1]+1)] = int(50)
                if obstacle[0]-1 > 0 and obstacle[0]-1 < self.height and int(obstacle[1]) < self.width:
                    if  grid[int(obstacle[0]-1), int(obstacle[1])] < int(1):
                        grid[int(obstacle[0]-1), int(obstacle[1])] = int(50)
                if int(obstacle[0]) < self.height and obstacle[1]-1 > 0 and obstacle[1]-1 < self.width:
                    if  grid[int(obstacle[0]), int(obstacle[1]-1)] < int(1):
                        grid[int(obstacle[0]), int(obstacle[1]-1)] = int(50)

                t = 0.1; i = 1
                while t*i <= dist[d]: # create free cells
                    x = (dist[d]-t*i)*np.cos(angles[d]) # x coordinate in lidar frame
                    y = (dist[d]-t*i)*np.sin(angles[d]) # y coordinate in lidar frame
                    x -= lidar_position # x coordinate in base link frame
                    #y -= lidar_position[1] # y coordinate in base link frame
                    #possible_free_cell = np.dot(self.rotMatrix, np.array([0, - self.height // 2])) + [y//self.resolution, x//self.resolution]
                    possible_free_cell = [y//self.resolution - self.height//2, x//self.resolution]
                    if int(possible_free_cell[0]) < self.height and int(possible_free_cell[1]) < self.width:
                        if grid[int(possible_free_cell[0]), int(possible_free_cell[1])] < int(1):
                            grid[int(possible_free_cell[0]), int(possible_free_cell[1])] = int(0)
                    i += 1
            else: # obstacle beyond the grid
                t = 0.1; i = 1
                while t*i <= dist[d]: # create free cells
                    x = min(dist[d]-t*i - lidar_position, self.resolution*self.width/2)*np.cos(angles[d]) # x coordinate in lidar frame
                    y = min(dist[d]-t*i, self.resolution*self.width/2)*np.sin(angles[d]) # y coordinate in lidar frame
                    possible_free_cell = [y//self.resolution - self.height//2, x//self.resolution]
                    #possible_free_cell = np.dot(self.rotMatrix, np.array([0, - self.height // 2])) + [y//self.resolution, x//self.resolution]
                    if int(possible_free_cell[0]) < self.height and int(possible_free_cell[1]) < self.width:
                        if grid[int(possible_free_cell[0]), int(possible_free_cell[1])] < int(1):
                            grid[int(possible_free_cell[0]), int(possible_free_cell[1])] = int(0)
                    i += 1
        return
    
    def pf_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        curr_x = pose_msg.pose.pose.position.x # global frame
        curr_y = pose_msg.pose.pose.position.y # global frame
        self.curr_pos = [curr_x, curr_y] # global frame
        self.curr_orientation = [pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w]
        #heading = tf.transformations.euler_from_quaternion((pose_msg.pose.pose.orientation.x,
                                                        #pose_msg.pose.pose.orientation.y,
                                                        #pose_msg.pose.pose.orientation.z,
                                                        #pose_msg.pose.pose.orientation.w))[2]

        euler = tf.transformations.euler_from_quaternion(self.curr_orientation)
        heading = euler[2]
        self.rotMatrix = np.array([[np.cos(heading), -np.sin(heading)], [np.sin(heading), np.cos(heading)]])

        if self.occ_grid is None:
            return

        #goal_loc = [0,0] #self.create_goal_point()
        mp = RRTMotionPlanner(self.occ_grid, self.curr_pos, self.rotMatrix) #, goal_loc)
        plan = mp.plan()

        plan_marker = self.create_plan_marker(plan)
        #self.plan_pub.publish(plan_marker)
        #print(plan)
        #print("wee")
        PP = PurePursuit(plan)
        #goal_marker = PP.create_waypoint_marker(goal_loc[0], goal_loc[1])
        self.drive_msg, marker = PP.pursue(curr_x, curr_y, heading)
        
        #self.goal_wp_pub.publish(goal_marker)
        #self.nearest_wp_pub.publish(marker)
        #print('woo')
        self.drive_pub.publish(self.drive_msg)
        """
        #drive_msg = AckermannDriveStamped()
        #drive_msg.drive.steering_angle = float(steering_angle)
        #sa_deg = np.rad2deg(steering_angle)
        #print("Steering Angle: {:.3f} [deg]".format(sa_deg))
        #if time.time() - self.start_time < 5:
        #    drive_msg.drive.speed = 0.1
        #else:
        #    drive_msg.drive.speed = 0.0
        self.drive_pub.publish(drive_msg)
    
    def create_goal_point(self):
        # We want to create a goal point in the car's reference frame
        #car_loc = (0,0)
        r = 0.5 # lookahead distance
        goal_loc = None
        max_steering_angle = np.pi/3
        while goal_loc is None:
            theta = np.random.uniform(low=-max_steering_angle, high=max_steering_angle)
            x = r*np.cos(theta)
            y = r*np.sin(theta)
            if self.collision_free((x,y)):
                goal_loc = (x,y)
        return goal_loc
    
    #def collision_free(self, pos):
        # Given a point in the car's reference frame, check whether or not it is collision-free
    #    return False
    """
    
    def create_plan_marker(self, plan):
        """ Given the position of a sample, publishes the necessary Marker data to the 'wp_viz' topic for RViZ visualization """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.action = 0 # add the marker
        marker.type = 5 # line_list

        for p in range(len(plan)-1):
            pt1 = Point()
            pt1.x = plan[p][0]
            pt1.y = plan[p][1]
            marker.points.append(pt1)

            pt2 = Point()
            pt2.x = plan[p+1][0]
            pt2.y = plan[p+1][1]
            marker.points.append(pt2)

        #
        #marker.pose.position.x = pos[0]
        #marker.pose.position.y = pos[1]
        #marker.pose.position.z = 0
        #marker.pose.orientation.x = 0.0
        #marker.pose.orientation.y = 0.0
        #marker.pose.orientation.z = 0.0
        #marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        #marker.scale.y = 0.1
        #marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0    
        return marker

    """"
    def sample(self):
        # This method should randomly sample the free space, and returns a viable point
        # Returns: (x, y) (float float): a tuple representing the sampled point

        x = np.random.uniform(low=x_low, high=x_high)
        y = np.random.uniform(low=y_low, high=y_high)
        return (x, y)

    def nearest(self, tree, sampled_point):
        # This method should return the nearest node on the tree to the sampled point
        # Args:
        #    tree ([]): the current RRT tree
        #    sampled_point (tuple of (float, float)): point sampled in free space
        # Returns:
        #    nearest_node (int): index of neareset node on the tree

        nearest_node = 0
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        # This method should return a point in the viable set such that it is closer 
        # to the nearest_node than sampled_point is.

        # Args:
        #    nearest_node (Node): nearest node on the tree to the sampled point
        #    sampled_point (tuple of (float, float)): sampled point
        # Returns:
        #    new_node (Node): new node created from steering

        new_node = None
        return new_node

    def check_collision(self, nearest_node, new_node):
        #This method should return whether the path between nearest and new_node is collision free.

        #Args:
        #    nearest (Node): nearest node on the tree
        #    new_node (Node): new node from steering
        # Returns:
        #    collision (bool): whether the path between the two nodes are in collision
        #                      with the occupancy grid
        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        # This method should return whether the latest added node is close enough to the goal.

        # Args:
        #    latest_added_node (Node): latest added node on the tree
        #    goal_x (double): x coordinate of the current goal
        #    goal_y (double): y coordinate of the current goal
        # Returns:
        #    close_enough (bool): true if node is close enoughg to the goal

        return False

    def find_path(self, tree, latest_added_node):
        # This method returns a path as a list of Nodes connecting the starting point to
        # the goal once the latest added node is close enough to the goal

        # Args:
        #    tree ([]): current tree as a list of Nodes
        #    latest_added_node (Node): latest added node in the tree
        # Returns:
        #    path ([]): valid path as a list of Nodes
        path = []
        return path
    
    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        # This method should return the cost of a node

        # Args:
        #    node (Node): the current node the cost is calculated for
        # Returns:
        #    cost (float): the cost value of the node
        return 0

    def line_cost(self, n1, n2):
        # This method should return the cost of the straight line between n1 and n2

        # Args:
        #    n1 (Node): node at one end of the straight line
        #    n2 (Node): node at the other end of the straint line
        # Returns:
        #    cost (float): the cost value of the line

        return 0

    def near(self, tree, node):
        # This method should return the neighborhood of nodes around the given node

        # Args:
        #    tree ([]): current tree as a list of Nodes
        #    node (Node): current node we're finding neighbors for
        # Returns:
        #    neighborhood ([]): neighborhood of nodes as a list of Nodes
        neighborhood = []
        return neighborhood
    """

def main():
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.spin()

if __name__ == '__main__':
    main()