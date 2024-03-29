#!/usr/bin/env python

import rclpy, time
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

MAX_SPEED = 2.0

SLOW_MODE = False

class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')
        #relevant attributes for the algorithms:
        self.threshold = 5. #check this
        self.min_gap = 3 #change this tf?
        self.safety_radius = 1. #1.5 #[m] #what is the car width?
        self.min_distance = 1
        self.gap_check_distance = 1.

        self.angle_min = None
        self.angle_max = None
        self.angle_increment = None
        self.angles = None
        
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/vesc/ackermann_cmd_mux/input/navigation'
        safety_topic = '/vesc/low_level/ackermann_cmd_mux/input/safety'

        self.scan_sub_ = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback,10)
        #self.safety_sub_ = self.create_subscription(safety_topic, AckermannDriveStamped, self.safety_callback, queue_size=10)
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def safety_callback(self, msg):
        print("REACHED")
        # vel = AckermannDriveStamped()
        # vel.drive.speed = float(0)
        self.drive_pub_.publish(msg)
        rclpy.signal_shutdown("Safety Stop")
        

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        min_range = min(ranges)
        min_index = ranges.index(min_range)
        proc_ranges = np.array(ranges)
        proc_ranges[proc_ranges > self.threshold] = self.threshold
        for item in proc_ranges:
            if item == 'nan' or item == np.inf:
                item = self.threshold

        # proc_ranges = np.nan_to_num(proc_ranges, posinf=self.threshold) # nan=self.threshold,
        
        delta_theta = abs(np.arctan2(self.safety_radius/2, min_range))

        current_angle_sum = 0
        current_index = min_index
        while (current_angle_sum < delta_theta) and (current_index < len(self.angles)):
            proc_ranges[current_index] = 0
            current_index += 1
            current_angle_sum += self.angle_increment
        current_angle_sum = 0
        current_index = min_index
        while (current_angle_sum > delta_theta) and (current_index >= 0):
            proc_ranges[current_index] = 0
            current_index -= 1
            current_angle_sum -= self.angle_increment

        # print(f"processed ranges = \n{proc_ranges}")
        return proc_ranges

    def valid_range(self, proc_ranges, index):
        # return (proc_ranges[index] > 0) # and (proc_ranges[index] < np.inf)
        return (proc_ranges[index] > self.min_distance)

    def bigger_ranges(self, ranges1, ranges2):
        # print(f"comparing: {ranges1} to {ranges2}")
        return ranges1 if (ranges1[1]-ranges1[0]) > (ranges2[1]-ranges2[0]) else ranges2

    def find_max_gap(self, proc_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges"""
        # ninety_index = np.argmin(np.abs(self.angles - np.deg2rad(90)))
        # minus_ninety_index = np.argmin(np.abs(self.angles + np.deg2rad(90)))
        # print(f"max of current ranges = {max(proc_ranges)}")
        current_index = 0
        current_range = [0,0]
        max_range = [0,0]

        for current_index in range(len(proc_ranges)):
            # #check if there is currently no indices in range
            # if len(current_range) == 0 and self.valid_range(proc_ranges, current_index):
            #     current_range.append(current_index)
            if self.valid_range(proc_ranges, current_index):
                current_range[1] = current_index
                
            else:
                if current_range == self.bigger_ranges(max_range, current_range):
                    max_range = current_range
                current_range = [current_index,current_index]
                        
        length = max_range[1] - max_range[0]   
        # print(f"max of max_gap = {max(max_range)}") 
        if length > self.min_gap:
            return max_range
        else:
            print("max gap too smalls")
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = 0.
            drive_msg.drive.speed = 0.
            self.drive_pub_.publish(drive_msg)
            # rclpy.sleep(2)
            # quit()
            # exit()
            rclpy.signal_shutdown("max gap too small")
        return
    
    def find_best_point(self, start_i, end_i, ranges, method='furthest'):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        if method == 'furthest':
            # Find furthest point in max_gap -- closest to centre of max_gap

            #Initialize
            index_furthest_point = int((start_i+end_i)/2)
            current_index = int((start_i+end_i)/2)
            current_furthest_point = ranges[current_index]

            #First, increasing from middle
            while current_index <= end_i:
                if ranges[current_index] > current_furthest_point:
                    current_furthest_point = ranges[current_index]
                    index_furthest_point = current_index
                current_index += 1

            #Next Decreasing from the middle
            while current_index >= start_i:
                if ranges[current_index] > current_furthest_point:
                    current_furthest_point = ranges[current_index]
                    index_furthest_point = current_index
                current_index -= 1

            return index_furthest_point
        
        elif method == 'middle':
            middle_index = int((start_i+end_i)/2)
            return middle_index

    def get_velocity(self, steering_angle, adaptive=True):
        if SLOW_MODE:
            return 0.8
        if adaptive:
            velocity = max(MAX_SPEED - abs(np.rad2deg(steering_angle))/50, 0.8) # Velocity varies smoothly with steering angle
            # print('Velocity: ' + str(velocity))
            return velocity
        if abs(steering_angle) < np.deg2rad(5):
            velocity = 2.3
        if abs(steering_angle) < np.deg2rad(10):
            velocity = 2.2
        elif abs(steering_angle) < np.deg2rad(15):
            velocity = 2.1
        elif abs(steering_angle) < np.deg2rad(20):
            velocity = 2.0
        elif abs(steering_angle) < np.deg2rad(30):
            velocity = 1.8
        elif abs(steering_angle) < np.deg2rad(40):
            velocity = 1.6
        else:
            velocity = 1.4
        return velocity

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        if self.angle_min is None:
            self.angle_min = data.angle_min

        if self.angle_max is None:
            self.angle_max = data.angle_max

        if self.angle_increment is None:
            self.angle_increment = data.angle_increment

        if self.angles is None:
            self.angles = np.linspace(self.angle_min, self.angle_max, num = len(data.ranges))
        
        # TODO:
        #Find closest point to LiDAR

        #Eliminate all points inside 'bubble' (set them to zero) 

        #Find max length gap 

        #Find the best point in the gap 

        #Publish Drive message

        ranges = list(data.ranges)
        ranges.reverse()
        processed_ranges = self.preprocess_lidar(ranges)

        max_gap_indices = self.find_max_gap(processed_ranges)
        steering_angle = 0.

        #First check if we are making a turn
        # if self.cornering_left(processed_ranges):
        #     time_init = time.time()
        #     while time.time() - time_init < 1:
        #         continue
        #     steering_angle = -(np.deg2rad(-90))

        # elif self.cornering_right(processed_ranges):
        #     time_init = time.time()
        #     while time.time() - time_init < 1:
        #         continue
        #     steering_angle = -(np.deg2rad(90))

        if max_gap_indices is not None:
            # best_idx = self.find_best_point(*max_gap_indices, processed_ranges, method='middle')
            best_idx = self.find_best_point(max_gap_indices[0], max_gap_indices[-1], processed_ranges, method='middle')
            steering_angle = -self.angles[best_idx]
        # else:
        #     print("Getting NONE for max_gap")
        #     drive_msg = AckermannDriveStamped()
        #     drive_msg.drive.steering_angle = 0.
        #     drive_msg.drive.speed = 0.
        #     self.drive_pub_.publish(drive_msg)
        #     rclpy.sleep(2)
        
        ## print("steering angle = ", -steering_angle) - uncomment to see the steering angle
        
        # print(f"max gap = {max_gap_indices}")
        # print(f"length of ranges = {len(ranges)}")

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(steering_angle)
        drive_msg.drive.speed = self.get_velocity(steering_angle)
        self.drive_pub_.publish(drive_msg)

    def cornering_left(self, ranges):
        #check if there is a drastic drop between range values at angles around -90
        angle_closer_left_idx = np.argmax(self.angles < np.deg2rad(-96))
        angle_further_left_idx = np.argmin(self.angles > np.deg2rad(-85))

        if ranges[angle_further_left_idx] - ranges[angle_closer_left_idx] > self.gap_check_distance:
            print("TURNING LEFT")
            return True
        else:
            return False

    def cornering_right(self, ranges):
        #check if there is a drastic drop between range values at angles around -90
        angle_closer_right_idx = np.argmin(self.angles < np.deg2rad(85))
        angle_further_right_idx = np.argmax(self.angles > np.deg2rad(96))

        if ranges[angle_further_right_idx] - ranges[angle_closer_right_idx] > self.gap_check_distance:
            print("TURNING RIGHT")
            return True
        else:
            return False

def main():
    rclpy.init()#"reactive_node")#, anonymous=True)
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

if __name__ == '__main__':
    print('NEW UPDATE NEWNEW')
    print("Follow-The-Gap Mode Initialized...")
    main()