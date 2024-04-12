# rrt dubins path planner for mavsim_python
import numpy as np
from message_types.msg_waypoints import MsgWaypoints
from planners.dubins_parameters import DubinsParameters


class RRTDubins:
    def __init__(self):
        self.segment_length = 450  # standard length of path segments
        self.dubins_path = DubinsParameters()

    def update(self, start_pose, end_pose, Va, world_map, radius):
        self.segment_length = 4 * radius
        tree = MsgWaypoints()
        tree.type = 'dubins'   

        ##### TODO #####
        # add the start pose to the tree
        tree.add(start_pose[:3], Va, course=start_pose[-1], cost=0, parent=0, connect_to_goal=0)
        
        while True:
            # extend tree toward end_pose
            self.extendTree(tree, end_pose, Va, world_map, radius)

            # check to see if start_pose connects directly to end_pose
            if distance(tree.ned[:,-1].reshape((3,1)), end_pose) < self.segment_length:
                if not self.collision(tree.ned[:,-1].reshape((3,1)), end_pose, world_map, radius):
                    tree.connect_to_goal[-1] = 1
                    break

        # find path with minimum cost to end_node
        print(f"tree.num_waypoints: {tree.num_waypoints}")
        self.waypoints_not_smoothed = findMinimumPath(tree, end_pose)

        print(f"waypoints_not_smoothed: {self.waypoints_not_smoothed.num_waypoints}")
        waypoints = self.smoothPath(self.waypoints_not_smoothed, world_map, radius)
        self.tree = tree
        return waypoints

    def extendTree(self, tree, end_pose, Va, world_map, radius):
        # extend tree by randomly selecting pose and extending tree toward that pose
        
        ###### TODO ######
        count = 0
        while True:
            count += 1
            rand_pose = random_pose(world_map, end_pose[2,0]).reshape((3,1))

            # find node closest to rand_pose
            closest_node = 0
            min_distance = np.inf
            for i in range(tree.num_waypoints):
                d = distance(tree.ned[:, i].reshape((3,1)), rand_pose)
                if d < min_distance:
                    min_distance = d
                    closest_node = i

            self.dubins_path.update(tree.ned[:,closest_node].reshape((3,1)), tree.course[closest_node],
                                    rand_pose, 0, radius)

            # create new pose that is segment_length away from closest_node towards the random pose
            if min_distance > self.segment_length:
                vector_to_rand_pose = rand_pose - tree.ned[:, closest_node].reshape((3,1))
                rand_pose = tree.ned[:, closest_node].reshape((3,1)) + self.segment_length * vector_to_rand_pose / min_distance

            # check to see if path from closest_node to rand_pose is collision free
            if not self.collision(tree.ned[:, closest_node].reshape((3,1)), rand_pose, world_map, radius):
                tree.add(rand_pose, Va,
                         course=np.arctan2(rand_pose[1]-tree.ned[1, closest_node], rand_pose[0]-tree.ned[0, closest_node]),
                         cost=distance(rand_pose.reshape((3,1)), end_pose), 
                         parent=closest_node, connect_to_goal=0)
                return

    def collision(self, start_pose, end_pose, world_map, radius):        
        # check to see of path from start_pose to end_pose colliding with map
        collision_flag = False
        points = points_along_path(start_pose[:3], end_pose[:3], 100)
        for i in range(points.shape[1]):
            if heightAboveGround(world_map, column(points, i)) <= 0:
                collision_flag = True
        return collision_flag

    def process_app(self):
        self.planner_viewer.process_app()

    def smoothPath(self, waypoints, world_map, radius):
        # construct smooth waypoint path
        smooth_waypoints = MsgWaypoints()
        smooth_waypoints.type = "dubins"
        smooth_waypoints.add(waypoints.ned[:, 0].reshape((3,1)), waypoints.airspeed[0],
                                course=0, cost=0, parent=0, connect_to_goal=0)
        next_node_pointer_j = 1
        curr_node_pointer_i = 0

        while next_node_pointer_j < waypoints.num_waypoints-1:
            w_s = waypoints.ned[:, curr_node_pointer_i].reshape((3,1))
            w_plus = waypoints.ned[:, next_node_pointer_j+1].reshape((3,1))

            if self.collision(w_s, w_plus, world_map, radius):
                w = waypoints.ned[:, next_node_pointer_j].reshape((3,1))
                angle_between_waypoints = np.arctan2(w[1]-w_s[1], w[0]-w_s[0])
                smooth_waypoints.add(w, waypoints.airspeed[next_node_pointer_j], 
                                    course=angle_between_waypoints,
                                        cost=distance(w_s, w), 
                                        parent=curr_node_pointer_i,
                                        connect_to_goal=0)
                curr_node_pointer_i = next_node_pointer_j

            next_node_pointer_j += 1

        smooth_waypoints.add(waypoints.ned[:, -1].reshape((3,1)), waypoints.airspeed[-1],
                                course=0, cost=0, parent=0, connect_to_goal=1)
        return smooth_waypoints

def findMinimumPath(tree, end_pose):
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
    waypoints.type = tree.type
    for i in path:
        waypoints.add(column(tree.ned, i),
                      tree.airspeed.item(i),
                      course=np.arctan2(tree.ned[1, i]-tree.ned[1, int(tree.parent.item(i))], tree.ned[0, i]-tree.ned[0, int(tree.parent.item(i))]),
                      cost=tree.cost.item(i),
                      parent=tree.parent.item(i),
                      connect_to_goal=tree.connect_to_goal.item(i))
    waypoints.add(end_pose[0:3],
                  tree.airspeed[-1],
                  end_pose.item(3),
                  np.inf,
                  np.inf,
                  np.inf)
    waypoints.type = tree.type
    return waypoints

def distance(start_pose, end_pose):
    # compute distance between start and end pose
    d = np.linalg.norm(start_pose[0:3] - end_pose[0:3])
    return d

def heightAboveGround(world_map, point):
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

def randomPose(world_map, pd):
    # generate a random pose
    pn = world_map.city_width * np.random.rand()
    pe = world_map.city_width * np.random.rand()
    chi = 0
    pose = np.array([[pn], [pe], [pd], [chi]])
    return pose

def mod(x):
    # force x to be between 0 and 2*pi
    while x < 0:
        x += 2*np.pi
    while x > 2*np.pi:
        x -= 2*np.pi
    return x

def column(A, i):
    # extracts the ith column of A and return column vector
    tmp = A[:, i]
    col = tmp.reshape(A.shape[0], 1)
    return col

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

def random_pose(world_map, pd):
    # generate a random pose
    pn = world_map.city_width * np.random.rand()
    pe = world_map.city_width * np.random.rand()
    pose = np.array([pn, pe, pd])
    return pose

