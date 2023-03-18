import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from RRTTree import RRTTree

MAX_SAMPLE_RADIUS = 2.0
GOAL_DISTANCE = 0.5
MAX_STEERING_ANGLE = np.pi/3

class RRTMotionPlanner(object):
    def __init__(self, occupancy_grid, curr_pos, rotMatrix, goal_pos=None, goal_prob=0.05): #planning_env, ext_mode, goal_prob):
        self.tree = RRTTree() #self.planning_env
        self.curr_pos = curr_pos
        self.tree.add_vertex(self.curr_pos, parent=None)

        self.occ_grid = occupancy_grid
        self.goal_pos = goal_pos
        self.goal_prob = goal_prob

        self.res = self.occ_grid.info.resolution
        self.height = self.occ_grid.info.height
        self.width = self.occ_grid.info.width
        self.grid = np.reshape(self.occ_grid.data,(self.width,self.height))

        self.rotMatrix = rotMatrix

        self.samples_pub = rospy.Publisher('/sample_viz', MarkerArray, queue_size = 10)

        # set environment and search tree
        #self.planning_env = planning_env
        
        # set search params
        #self.ext_mode = ext_mode
        self.ext_mode = 'E2'
    
        # set step size for extensions (custom)
        #self.step_size = planning_env.step_size

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''

        markerArray = MarkerArray()
        
        goal_added = False; num_iter = 0
        while not goal_added:
            num_iter += 1
            goal = False

            # Sampling step
            #p = np.random.uniform() # goal biasing
            #if p < self.goal_prob:
            #    pos = self.goal_pos
            #    goal = True
            #else:
            pos = self.sample()

            # Get nearest vertex to the sample
            nearest_vert = self.tree.get_nearest_pos(pos)
            nearest_vert_idx = nearest_vert[0]

            if self.ext_mode == 'E2':
                pos = self.extend(nearest_vert[1], pos)
                if not self.collision_free_pos(pos, self.occ_grid):
                    continue
                #print(pos)

            if np.linalg.norm(np.subtract(pos,self.curr_pos),2) > GOAL_DISTANCE:# and num_iter > 10:# and np.linalg.norm(pos,2) < MAX_SAMPLE_RADIUS:
                goal = True   
                #print("yp")
            sample_marker = self.create_sample_marker(pos, goal)
            #if num_iter < 1000:
            markerArray.markers.append(sample_marker)
            self.create_waypoint_marker_array(markerArray)
            
            # Verify that the sample is in free space
            #if not self.collision_free_pos(pos, self.occ_grid):
            #    continue

            # Partial extensions, if enabled
            #if self.ext_mode == 'E2':
            #    pos = self.extend(nearest_vert[1], pos) # config = x_new
                #if not env.config_validity_checker(config):
                #    continue
            
            # Check obstacle-collision for potential edge
            if self.edge_validity_checker(pos, nearest_vert[1], self.occ_grid):
                pos_idx = self.tree.add_vertex(pos, nearest_vert)
                #cost = self.tree.compute_distance(config, nearest_vert[1])
                self.tree.add_edge(nearest_vert_idx, pos_idx)#, cost)
                if goal:# and self.ext_mode == 'E1':
                    goal_added = True
            else:
                goal_added = False

        # Record the plan
        plan = []
        plan.append(pos)
        child_idx = pos_idx
        parent_pos = nearest_vert[1]
        while self.tree.edges[child_idx]:
            plan.append(parent_pos)
            child_idx = self.tree.get_idx_for_pos(parent_pos)
            parent_idx = self.tree.edges[child_idx] # new parent
            parent_pos = self.tree.vertices[parent_idx].pos
        plan.append(parent_pos)
        plan = plan[::-1]
        #print(plan)
        return plan #np.array(plan)

    """
    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 2.3 - DONE
        
        return self.tree.get_vertex_for_config(plan[-1]).cost
    """

    def sample(self):
        """ This method should randomly sample the free space, and returns a viable point """
        pos = None
        
        while pos is None:
            r = np.random.uniform(low=0, high=MAX_SAMPLE_RADIUS)
            theta = np.random.uniform(low=-MAX_STEERING_ANGLE, high=MAX_STEERING_ANGLE)
            #theta = np.random.uniform(low=-np.pi/2, high=np.pi/2)
            x = r*np.cos(theta)
            y = r*np.sin(theta)
            #print(x,y)
            if abs(y) < (self.width*self.res / 2) and abs(x) < (self.height*self.res):
                point = [y//self.res - self.height//2, x//self.res]
                #point = np.dot(self.rotMatrix, np.array([0, - self.height // 2])) + [y//self.res, x//self.res]
                if int(point[0]) < self.height and int(point[1]) < self.width:
                    if self.grid[int(point[0]), int(point[1])] == int(0):
                    #print('yay')
                        #print(x,y)
                        pos = np.dot(self.rotMatrix, np.array([x, y])) + [self.curr_pos[0], self.curr_pos[1]]
                        #pos = np.dot(self.rotMatrix, np.array([y, x])) + [self.curr_pos[0], self.curr_pos[1]]
                        #pos = np.dot(self.rotMatrix, np.array([y, x])) + [self.curr_pos[0], self.curr_pos[1]]
                        #pos = (x,y)
        return pos

    def collision_free_pos(self, position, occupancy_grid):
        '''
        Checks if a given point is in free space, given the car's local occupancy grid.
        @param position The XY position of the point in the car's reference frame.
        @param occupancy_grid The local occupancy grid in the car's reference frame.
        '''
        [x,y] = np.dot(np.transpose(self.rotMatrix), np.subtract(np.array(position), self.curr_pos))
        point = [y//self.res - self.height//2, x//self.res]
        #print(point)
        #print(int(point[0]),int(point[1]))
        if abs(int(point[0])) < self.height / 2 and abs(int(point[1])) < self.width:
            #print("yay")
            if self.grid[int(point[0]), int(point[1])] == int(0):
                #print("huh")
                return True
        return False # True if collision free, else False
    
    def edge_validity_checker(self, position1, position2, occupancy_grid):
        '''
        Checks if the straight-line path between two given points is in free space (non-collisional), given the car's local occupancy grid.
        @param position1 The XY position of the first point in the car's reference frame.
        @param position2 The XY position of the second point in the car's reference frame.
        @param occupancy_grid The local occupancy grid in the car's reference frame.
        '''
        return True # True if collision free, else False

    
    def extend(self, near_pos, rand_pos):
        '''
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        '''
        # TODO: Task 2.3 - DONE

        #goal = False
        #goal_config = self.planning_env.goal
        #if np.allclose(rand_config, goal_config):
        #    goal = True

        vec = np.subtract(rand_pos, near_pos) 
        vec_mag = np.linalg.norm(vec,2)
        unit_vec = vec / vec_mag

        # Projection of the step size in the direction of the neighbor
        new_vec = 0.1 * vec_mag * unit_vec #self.step_size * unit_vec
        new_pos = near_pos + new_vec # New sample point

        # Check if this overshoots the goal, if yes return the goal
        #goal_added = False
        #if goal and vec_mag < self.step_size:
        #    new_config = goal_config
        #    goal_added = True # Tells the motion planner to terminate

        return new_pos #, goal_added
    
    def create_sample_marker(self, pos, goal=False):
        # Given the position of a sample, publishes the necessary Marker data to the 'wp_viz' topic for RViZ visualization
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = 0
        marker.type = 2 # sphere
        marker.action = 0 # add the marker
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        if goal:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        return marker

    def create_waypoint_marker_array(self, markerArray):
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        self.samples_pub.publish(markerArray)
        return