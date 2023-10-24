#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
SLOW_MODE = True
MAX_SPEED = 6.2
MIN_SPEED = 1.0

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
        self.bubble_radius = 1.5 # fixed
        self.threshold = 2.5 # fixed
        self.max_range  = 5. # fixed
        self.current_steering_angle = 0.0 # variable
        self.integral = 0.0

    def preprocess_lidar(self, ranges, lidar_min, lidar_max, angle_increment):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """

        # Note: lidar_min/max is constant, why is it getting passed each time?

        min_angle = -90 * (math.pi/180) 
        max_angle = 90 * (math.pi/180) 
        min_idx = int(math.floor((min_angle - lidar_min)/angle_increment))
        max_idx = int(math.floor((max_angle - lidar_min)/angle_increment))
        
        proc_ranges = np.array(ranges[min_idx:max_idx])
        
        proc_ranges[proc_ranges == np.inf] = self.threshold 
        proc_ranges[proc_ranges > self.max_range] = self.max_range 
        proc_ranges[proc_ranges < self.threshold] = 0 

        return (proc_ranges, min_idx, max_idx)
        
    """
    def remove_corners(self, ranges):
        indicies_to_zero = []
        i = 0
        corners_removed = False
        while i < len(ranges) - 1:
            if np.abs(ranges[i] - ranges[i+1]) > MIN_THRESHOLD_FOR_CORNER:
                delta_theta = np.tan(CAR_WIDTH / (2 * ranges[i]))
                if ranges[i] > delta_theta:
                    min_idx = np.argmin(np.abs(ranges - (ranges[i] - delta_theta)))
                else:
                    min_idx = 0
                if ranges[i] < ranges[-1] - delta_theta:
                    max_idx =  np.argmin(np.abs(ranges - (ranges[i] + delta_theta)))
                else:
                    max_idx = len(ranges) - 1
                # print(f"len of ranges[min_idx : max_idx] = {ranges[min_idx : max_idx]}")
                # print(f"len of np.zeros(max_idx-min_idx) = {np.zeros(max_idx-min_idx)}")
                #print(f"man_idx = {max_idx}, min_idx = {min_idx}")
                #ranges[min_idx : max_idx] = np.zeros(max_idx-min_idx)
                indicies_to_zero.append([min_idx, max_idx])
                
                i+= max_idx - min_idx
                corners_removed = True
            if corners_removed:
                i += max_idx - min_idx
            else:
                i += 1  
        for indices in indicies_to_zero:
            min_idx = indices[0]
            max_idx = indices[1]
            ranges[min_idx:max_idx] = np.zeros(max_idx-min_idx)
        return ranges
    """

    def find_max_gap(self, free_space_ranges):
        """ 
        Return the start index & end index of the max gap in free_space_ranges
        A n-length gap is a n-length series of consecutive non-zero elements in the range
        """
        current_start = None
        current_length = 0
        max_start = None
        max_length = 0
        
        for i, val in enumerate(free_space_ranges):
            if val != 0:
                if current_start is None: # Starts new gap count
                    current_start = i
                current_length += 1
            else: # Ends gap
                if current_start is not None and current_length > max_length: # largest gap until now
                    max_start = current_start
                    max_length = current_length
                current_start = None
                current_length = 0
        
        if current_start is not None and current_length > max_length: # At the end if the gap hasn't ended and is largest, end it.
            max_start = current_start
            max_length = current_length
        
        if max_start is None:
            return None, None # VERY BAD BUT JUST IN CASE. SHOULD THROW AN ERROR
        else:
            return max_start, max_start + max_length - 1
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        # return (start_i+end_i)/2
        """Find the optimal point within the identified gap for smooth transition"""
        

        return (start_i+end_i)/2
    
    def get_velocity(self, steering_angle, adaptive=True):
        if SLOW_MODE:
            return MIN_SPEED
        #if adaptive:
        #    velocity = max(MAX_SPEED - abs(np.rad2deg(steering_angle))/50, 0.8) # Velocity varies smoothly with steering angle
            # print('Velocity: ' + str(velocity))
        #    return velocity
        
        if abs(steering_angle) < np.deg2rad(10):
            velocity = max(MAX_SPEED - abs(np.rad2deg(steering_angle))/50, 0.8)
            #velocity = 2.3
        #if abs(steering_angle) < np.deg2rad(10):
        #    velocity = 2.2
        elif abs(steering_angle) < np.deg2rad(15):
            velocity = 2.0
        elif abs(steering_angle) < np.deg2rad(20):
            velocity = 1.7
        elif abs(steering_angle) < np.deg2rad(30):
            velocity = 1.5
        elif abs(steering_angle) < np.deg2rad(40):
            velocity = 1.4
        else:
            velocity = 1.2
        

        #velocity = max(MAX_SPEED-(2/(1+np.exp(-5*np.abs(steering_angle)/25))-1)*(MAX_SPEED-MIN_SPEED), MIN_SPEED)#-\frac{4}{\left(d\right)^{2}},v_{n}\right)

        return velocity
         
        
        
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        (proc_ranges, min_idx, max_idx) = self.preprocess_lidar(ranges, data.angle_min, data.angle_max, data.angle_increment)
        #Find closest point to LiDAR
        closest_index = np.argmin(proc_ranges)
        print("closest idx", closest_index)
        #Eliminate all points inside 'bubble' (set them to zero) 
        start_index_bubble = int(max(closest_index-self.bubble_radius, 0))
        end_index_bubble = int(min(closest_index+self.bubble_radius, len(proc_ranges)))
        print("indexs:",start_index_bubble, end_index_bubble)
        # proc_ranges[start_index_bubble:end_index_bubble] = 0
        #Find max length gap 
        (gap_start, gap_end) = self.find_max_gap(proc_ranges)
        #Find the best point in the gap
        print("gap endpoints", gap_start, gap_end)
        best_point = self.find_best_point(gap_start,gap_end, proc_ranges) 
        true_index =  best_point + min_idx
        print("bp", best_point)
        #Publish Drive message 
        self.current_steeering_angle = true_index * data.angle_increment + data.angle_min
        '''
        desired_angle = true_index * data.angle_increment + data.angle_min
        K_P = 0.7
        K_I = 0.2
        error = desired_angle-self.current_steeering_angle
        self.integral+=error
        self.current_steeering_angle = error*K_P + self.current_steeering_angle + self.integral * K_I
        '''
        print(self.current_steeering_angle)
        ackermann_message = AckermannDriveStamped()
        ackermann_message.drive.steering_angle = self.current_steeering_angle
        self.current_steeering_angle = self.current_steeering_angle
        ackermann_message.drive.speed = self.get_velocity(self.current_steeering_angle)
        self.drive_pub.publish(ackermann_message)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()