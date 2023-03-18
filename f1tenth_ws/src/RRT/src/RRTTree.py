import operator
import numpy as np

class RRTTree(object):
    def __init__(self):#, planning_env):#, task="mp"):
        #self.planning_env = planning_env
        #self.task = task
        self.vertices = {}
        self.edges = {}

    def get_root_id(self):
        '''
        Returns the ID of the root in the tree.
        '''
        return 0

    def add_vertex(self, pos, parent):
        '''
        Add a state to the tree.
        @param config Configuration to add to the tree.
        '''
        vid = len(self.vertices)
        self.vertices[vid] = Node(pos=pos, parent=parent)
        return vid

    def add_edge(self, sid, eid):#, edge_cost=0):
        '''
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        '''
        self.edges[eid] = sid
        #self.vertices[eid].set_cost(cost=self.vertices[sid].cost + edge_cost)

    def if_goal_exists(self, pos):
        '''
        Check if goal exists.
        @param config Configuration to check if exists.
        '''
        goal_idx = self.get_idx_for_pos(pos=pos)
        if goal_idx is not None:
            return True
        return False

    def get_vertex_for_pos(self, pos):
        '''
        Search for the vertex with the given config and return it if exists
        @param config Configuration to check if exists.
        '''
        v_idx = self.get_idx_for_config(pos=pos)
        if v_idx is not None:
            return self.vertices[v_idx]
        return None

    def get_idx_for_pos(self, pos):
        '''
        Search for the vertex with the given config and return the index if exists
        @param config Configuration to check if exists.
        '''
        #valid_idxs = [v_idx for v_idx, v in self.vertices.items() if (v.pos == pos).all()]
        valid_idxs = [v_idx for v_idx, v in self.vertices.items() if (v.pos[0] == pos[0] and v.pos[1] == pos[1])]
        if len(valid_idxs) > 0:
            return valid_idxs[0]
        return None

    def get_nearest_pos(self, pos):
        '''
        Find the nearest vertex for the given config and returns its state index and configuration
        @param config Sampled configuration.
        '''
        # compute distances from all vertices
        dists = []
        for _, vertex in self.vertices.items():
            dists.append(self.compute_distance(pos, vertex.pos))

        # retrieve the id of the nearest vertex
        vid, _ = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid].pos

    def compute_distance(self, pos1, pos2):
        return np.linalg.norm(np.subtract(pos1,pos2), 2)

class Node(object):
    def __init__(self, pos, parent=None):#, is_root=False):
        self.pos = pos
        #self.x = pos[0] # car's reference frame
        #self.y = pos[1] # car's reference frame
        self.parent = parent
        #self.cost = None # only used in RRT*
        #self.is_root = is_root

#class RRTVertex(object):
#
#    def __init__(self, config):#, cost=0, inspected_points=None):
#
#        self.config = config
#        self.cost = cost
#        self.inspected_points = inspected_points

#    def set_cost(self, cost):
#        '''
#        Set the cost of the vertex.
#        '''
#        self.cost = cost