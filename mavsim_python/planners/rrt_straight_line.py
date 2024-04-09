# rrt straight line path planner for mavsim_python
import numpy as np
from message_types.msg_waypoints import MsgWaypoints

class RRTStraightLine:
    def __init__(self):
        self.segment_length = 300 # standard length of path segments

    def update(self, start_pose, end_pose, Va, world_map, radius):
        tree = MsgWaypoints()
        #tree.type = 'straight_line'
        tree.type = 'fillet'

        ###### TODO ######
        # add the start pose to the tree
        tree.add(start_pose, Va, cost=0, parent=0, connect_to_goal=0)
        
        found_connecting_node = False
        while not found_connecting_node:
            # extend tree toward end_pose
            self.extend_tree(tree, end_pose, Va, world_map)
            print("tree.num_waypoints: ", tree.num_waypoints)

            # check to see if start_pose connects directly to end_pose
            if distance(tree.ned[-1], end_pose) < self.segment_length:
                if not collision(tree.ned[-1], end_pose, world_map):
                    tree.connect_to_goal[-1] = 1
                    tree.add(end_pose, Va, np.inf, np.inf, np.inf, np.inf)
                    found_connecting_node = True
        
        # find path with minimum cost to end_node
        waypoints_not_smooth = find_minimum_path(tree, end_pose)
        waypoints = smooth_path(waypoints_not_smooth, world_map)
        self.waypoints_not_smoothed = waypoints_not_smooth
        self.tree = tree
        return waypoints

    def extend_tree(self, tree, end_pose, Va, world_map):
        # extend tree by randomly selecting pose and extending tree toward that pose
        
        ###### TODO ######
        while True:
            rand_pose = random_pose(world_map, end_pose[-1,0]).reshape((3,1))
            # create new pose self.segment_length away from last pose in the tree
            if distance(tree.ned[:, -1], rand_pose) > self.segment_length:
                vector_to_rand_pose = rand_pose - tree.ned[:, -1]
                rand_pose = tree.ned[:, -1] + self.segment_length * vector_to_rand_pose / distance(tree.ned[:, -1], rand_pose)

            if not collision(tree.ned[:, -1].reshape((3,1)), rand_pose, world_map):
                tree.add(rand_pose, Va, cost=distance(tree.ned[:, -1], end_pose), parent=tree.num_waypoints-1, connect_to_goal=0)
                return
        
    def process_app(self):
        self.planner_viewer.process_app()

def smooth_path(waypoints, world_map):

    ##### TODO ####
    # construct smooth waypoint path
    smooth_waypoints = MsgWaypoints()
    smooth = [0]  # add the first waypoint
    j = 1
    i = 0

    while j < len(waypoints):
        w_s = smooth[i]
        w_plus = waypoints[j]

        if not collision(smooth[-1], waypoints[j], world_map):
            smooth.append(waypoints[j])
        j += 1

    return smooth_waypoints

def find_minimum_path(tree, end_pose):
    # find the lowest cost path to the end node
    # find nodes that connect to end_node
    connecting_nodes = []
    for i in range(tree.num_waypoints):
        if tree.connect_to_goal.item(i) == 1:
            connecting_nodes.append(i)
    # find minimum cost last node
    idx = np.argmin(tree.cost[connecting_nodes])
    # construct lowest cost path order
    path = [connecting_nodes[idx]]  # last node that connects to end node
    parent_node = tree.parent.item(connecting_nodes[idx])
    while parent_node >= 1:
        path.insert(0, int(parent_node))
        parent_node = tree.parent.item(int(parent_node))
    path.insert(0, 0)
    # construct waypoint path
    waypoints = MsgWaypoints()
    for i in path:
        waypoints.add(column(tree.ned, i),
                      tree.airspeed.item(i),
                      np.inf,
                      np.inf,
                      np.inf,
                      np.inf)
    waypoints.add(end_pose,
                  tree.airspeed[-1],
                  np.inf,
                  np.inf,
                  np.inf,
                  np.inf)
    waypoints.type = tree.type
    return waypoints

def random_pose(world_map, pd):
    # generate a random pose
    pn = world_map.city_width * np.random.rand()
    pe = world_map.city_width * np.random.rand()
    pose = np.array([pn, pe, pd])
    return pose

def distance(start_pose, end_pose):
    # compute distance between start and end pose
    d = np.linalg.norm(start_pose - end_pose)
    return d

def collision(start_pose, end_pose, world_map):
    # check to see of path from start_pose to end_pose colliding with map
    collision_flag = False
    points = points_along_path(start_pose, end_pose, 100)
    for i in range(points.shape[1]):
        if height_above_ground(world_map, column(points, i)) <= 0:
            collision_flag = True
    return collision_flag

def height_above_ground(world_map, point):
    # find the altitude of point above ground level
    point_height = -point.item(2)
    tmp = np.abs(point.item(0)-world_map.building_north)
    d_n = np.min(tmp)
    idx_n = np.argmin(tmp)
    tmp = np.abs(point.item(1)-world_map.building_east)
    d_e = np.min(tmp)
    idx_e = np.argmin(tmp)
    if (d_n<world_map.building_width) and (d_e<world_map.building_width):
        map_height = world_map.building_height[idx_n, idx_e]
    else:
        map_height = 0
    h_agl = point_height - map_height
    return h_agl

def points_along_path(start_pose, end_pose, N):
    # returns points along path separated by Del
    points = start_pose
    q = (end_pose - start_pose)
    L = np.linalg.norm(q)
    q = q / L
    w = start_pose
    for i in range(1, N):
        w = w + (L / N) * q
        points = np.append(points, w, axis=1)
    return points

def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col