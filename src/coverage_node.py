#!/usr/bin/python

import sys, rospy
import math
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
import tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist, Quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import LaserScan

sys.setrecursionlimit(10 ** 9)
num_of_rows = num_of_colms = source_x = source_y = map_resolution = 0
current_robot_row = current_robot_col = 0


robot_size = 0.35
arr = arr_from_map = [0, 0]
goal = MoveBaseGoal()
command_pub = grid_4d = None

DOWN = "Down"
LEFT = "Left"
RIGHT = "Right"
UP = "Up"


class Hemilton(object):
    def __init__(self, vertices, starting_point, dfs):
        self.vertices = vertices
        self.visited_vertices = []
        self.visited_vertices.append(starting_point.coordinate)
        self.dest = starting_point
        self.path = []
        self.dfs = dfs

    def is_goal(self, node):
        return self.dest.coordinate == node.coordinate

    def get_new_node(self, dup_neighbours):
        new_node = None
        if len(dup_neighbours):
            if len(dup_neighbours) == 1:
                new_node = dup_neighbours[0]
            else:
                first_neighbour = dup_neighbours[0]
                first_index = self.dfs[first_neighbour.source_in_fourd_grid]
                second_neighbour = dup_neighbours[1]
                second_neighbour = self.dfs[second_neighbour.source_in_fourd_grid]
                if first_index > second_neighbour:
                    new_node = second_neighbour
                else:
                    new_node = first_neighbour
        return new_node

    def find_cycle(self, came_from, cur_node, first_time):
        self.visited_vertices.append(cur_node.coordinate)

        # if (not is_first_time) and self.is_goal(cur_node):
        #     return
        if not first_time:
            remove_source_node(cur_node, came_from)
        num_of_neighbours = len(cur_node.get_neighbours())
        front_neighbour = False
        new_node = None
        if num_of_neighbours == 1:
            new_node = cur_node.get_neighbours()[0]
        else:
            dup_neighbours = cur_node.get_neighbours()
            for neighbour in cur_node.get_neighbours():
                if neighbour.coordinate in self.visited_vertices:
                    dup_neighbours.remove(neighbour)
                elif neighbour.source_in_fourd_grid.coordinate == cur_node.source_in_fourd_grid.coordinate:
                    new_node = neighbour
                    break

                current_index = self.dfs[cur_node.source_in_fourd_grid]
                neighbour_index = self.dfs[neighbour.source_in_fourd_grid]

                if current_index + 1 == neighbour_index:
                    new_node = neighbour
                    front_neighbour = True

                if (neighbour_index == current_index - 1) and not front_neighbour:
                    new_node = neighbour

        if not new_node:
            new_node = self.get_new_node(dup_neighbours)
        if new_node:
            self.find_cycle(cur_node, new_node, False)


class Point(object):
    def __init__(self, x, y):
        self.is_occupied = False
        self.x = x
        self.y = y

    def set_is_occupied(self, bool):
        # Set the point to be occupied.
        self.is_occupied = bool

    def __iter__(self, other):
        return self.x == other.x and self.y == other.y


class Node(object):
    def __init__(self, coordinate):
        self.coordinate = coordinate
        self.up_neighbour = None
        self.down_neighbour = None
        self.right_neighbour = None
        self.left_neighbour = None
        self.adj = []
        self.children = []

    def set_next_node(self, next_node, direction):
        # THe function sets the following node.
        if direction == UP:
            if not next_node.coordinate.is_occupied:
                self.up_neighbour = next_node
        elif direction == DOWN:
            if not next_node.coordinate.is_occupied:
                self.down_neighbour = next_node
        elif direction == RIGHT:
            if not next_node.coordinate.is_occupied:
                self.right_neighbour = next_node
        elif direction == LEFT:
            if not next_node.coordinate.is_occupied:
                self.left_neighbour = next_node

    def set_neighbours(self):
        # The function sets the neighbours of the node.
        self.adj.append([self.right_neighbour, RIGHT])
        self.adj.append([self.up_neighbour, UP])
        self.adj.append([self.left_neighbour, LEFT])
        self.adj.append([self.down_neighbour, DOWN])

    def get_neighbours(self):
        # The function return the neighbours of the node.
        list_of_adj = []
        for adj in self.adj:
            if adj[0]:
                list_of_adj.append(adj)
        return list_of_adj

    def add_child(self, child):
        # THe function appends a child to the children list.
        self.children.append(child)


class Dnode(object):
    def __init__(self, coordinate):
        self.coordinate = coordinate
        self.neighbours = {}
        self.up = None
        self.down = None
        self.right = None
        self.left = None
        self.source_in_fourd_grid = None
        self.dic = {}

    def set_dict(self):
        # Sets the Dnode dict.
        if self.right:
            self.dic[self.right] = RIGHT
        if self.up:
            self.dic[self.up] = UP
        if self.left:
            self.dic[self.left] = LEFT
        if self.down:
            self.dic[self.down] = DOWN

    def get_neighbours(self):
        # REturns the neighbours of this Dnode.
        list_of_adj = []
        if self.right:
            list_of_adj.append(self.right)
        if self.up:
            list_of_adj.append(self.up)
        if self.left:
            list_of_adj.append(self.left)
        if self.down:
            list_of_adj.append(self.down)
        return list_of_adj

    def set_source(self, source_node):
        self.source_in_fourd_grid = source_node

    def set_occupied(self):
        # Set if node is occupied.
        self.coordinate.set_is_occupied(True)

    def set_next_node(self, next_node, direction):
        # Set's the precedding node of this node.
        if direction == UP:
            if not next_node.coordinate.is_occupied:
                self.up = next_node
        elif direction == DOWN:
            if not next_node.coordinate.is_occupied:
                self.down = next_node
        elif direction == RIGHT:
            if not next_node.coordinate.is_occupied:
                self.right = next_node
        elif direction == LEFT:
            if not next_node.coordinate.is_occupied:
                self.left = next_node


class Graph(object):
    def __init__(self, rev_grid, type):
        # self.vertices_num = num_of_vertices
        self.vertices = []
        self.came_from = {}
        self.dfs_list = []
        self.dfs_dict = {}
        for i, row in enumerate(rev_grid):
            row_nodes = []
            for j, node in enumerate(row):
                curr_point = Point(i, j)
                if rev_grid[i][j]:
                    curr_point.set_is_occupied(True)
                if type == "4d":
                    row_nodes.append(Node(curr_point))
                else:
                    row_nodes.append(Dnode(curr_point))
            self.vertices.append(row_nodes)

    def set_neighbours(self):
        # The function sets the node's neighbours.
        last_row = len(self.vertices) - 1
        last_col = len(self.vertices[0]) - 1
        for i, row in enumerate(self.vertices):
            for j, node in enumerate(row):
                if not node.coordinate.is_occupied:
                    if i != 0:
                        node.set_next_node(self.vertices[i - 1][j], UP)
                    if i != last_row:
                        node.set_next_node(self.vertices[i + 1][j], DOWN)
                    if j != last_col:
                        node.set_next_node(self.vertices[i][j + 1], RIGHT)
                    if j != 0:
                        node.set_next_node(self.vertices[i][j - 1], LEFT)

    # dfs function
    def DFS(self, first_node):
        # set a list of adj for each node
        for row in self.vertices:
            for vertex in row:
                if not vertex.coordinate.is_occupied:
                    vertex.set_neighbours()
        self.create_dfs_dict(first_node, (None, None))

    #
    def create_dfs_dict(self, node, came_from):
        # Create's the DFS dict.
        x, y = came_from
        self.came_from[node] = [x, y]
        self.dfs_list.append(node)
        self.dfs_dict[node] = len(self.dfs_list) - 1
        for adj_node in node.get_neighbours():
            if adj_node[0] not in self.came_from:
                self.create_dfs_dict(adj_node[0], [node, adj_node[1]])

    def match_4d_to_d(self, d_graph):
        # Set the coorisponding node from 4dgraph to d_graph.
        for i in range(len(self.vertices)):
            k = double(i)
            for j in range(len(self.vertices[0])):
                l = double(j)
                cell_4d = self.vertices[i][j]
                children = []
                children.append(d_graph.vertices[k][l])
                children.append(d_graph.vertices[k][l + 1])
                children.append(d_graph.vertices[k + 1][l])
                children.append(d_graph.vertices[k + 1][l + 1])
                for child in children:
                    cell_4d.add_child(child)
                    child.set_source(cell_4d)
                    if cell_4d.coordinate.is_occupied:
                        child.set_occupied()


def double(x):
    # DOubles the x's value.
    return x * 2


def pp(x):
    # The function immetate ++ functionality.
    return x + 1


def remove_source_node(curr_node, source_node):
    # The function removes the source node of a node.
    came_from = source_node.dic[curr_node]
    if came_from == DOWN:
        curr_node.up = None
    elif came_from == UP:
        curr_node.down = None
    elif came_from == LEFT:
        curr_node.right = None
    elif came_from == RIGHT:
        curr_node.left = None


def get_prev_children(prev_node):
    # Returns the previous node children.
    prev_children = []
    prev_children.append(prev_node.children[0])
    prev_children.append(prev_node.children[1])
    prev_children.append(prev_node.children[2])
    prev_children.append(prev_node.children[3])
    return prev_children


def get_curr_children(curr_node):
    # Returns the current node children.
    curr_children = []
    curr_children.append(curr_node.children[0])
    curr_children.append(curr_node.children[1])
    curr_children.append(curr_node.children[2])
    curr_children.append(curr_node.children[3])
    return curr_children


def delete_up(prev_children, curr_children):
    # Set's the up nodes vals to None.
    curr_children[2].right = None
    curr_children[3].left = None
    prev_children[0].right = None
    prev_children[1].left = None


def delete_right(prev_children, curr_children):
    # Set's the right nodes vals to None.
    curr_children[0].down = None
    curr_children[2].up = None
    prev_children[1].down = None
    prev_children[3].up = None


def delete_left(prev_children, curr_children):
    # Set's the left nodes vals to None.
    curr_children[1].down = None
    curr_children[3].up = None
    prev_children[0].down = None
    prev_children[2].up = None


def delete_down(prev_children, curr_children):
    # Set's the down nodes vals to None.
    curr_children[0].right = None
    curr_children[1].left = None
    prev_children[2].right = None
    prev_children[3].left = None


def remove_direction_by_dfs(graph_4d):
    # graph_d will remove directions according to spanning tree.
    route = graph_4d.dfs_list
    came_from_dict = graph_4d.came_from
    # skip first node (there's no came_from)
    for curr_node in route[1:]:
        prev_node = came_from_dict[curr_node][0]
        origin_direction = came_from_dict[curr_node][1]
        prev_childs = get_prev_children(prev_node)
        curr_childs = get_curr_children(curr_node)
        if origin_direction == UP:
            delete_up(prev_childs, curr_childs)
        elif origin_direction == RIGHT:
            delete_right(prev_childs, curr_childs)
        elif origin_direction == DOWN:
            delete_down(prev_childs, curr_childs)
        elif origin_direction == LEFT:
            delete_left(prev_childs, curr_childs)



def get_robot_position():
    # The function returns the robots position from the map by listening to the "base_footprint" service.
    listener = tf.TransformListener()
    listener.waitForTransform("/map", "/base_footprint", rospy.Time(0), rospy.Duration(5))
    try:
        (transformation, _) = listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        x, y, _ = transformation
        return x, y
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr("Service failed: %s" % e)


def convert_point_to_world():
    # The function converts the point in the dgrid into the real world coordinates.
    global source_y, map_resolution, num_of_rows, current_robot_col, current_robot_row
    curr_x, curr_y = get_robot_position()
    rospy.loginfo("Robot location: " + str(curr_x) + ", " + str(curr_y))
    source_y = source_y + map_resolution * num_of_rows
    x = math.fabs(curr_x - source_x)
    current_robot_col = int(math.floor(x / map_resolution))
    y = math.fabs(curr_y - source_y)
    current_robot_row = int(math.floor(y / map_resolution))
    if current_robot_row < 0 or current_robot_row >= num_of_rows or current_robot_col < 0 \
            or current_robot_col >= num_of_colms:
        rospy.logerr("Robot is out of limits")
    else:
        print("Robot row: " + str(current_robot_row) + " col: " + str(current_robot_col))


def position_valid(row, col, num_of_rows, num_of_colms):
    # Checks if the position is valid.
    return 0 <= row < num_of_rows and 0 <= col < num_of_colms


def vertex_in_list(vertices_list, vertex):
    # REturn if vertex is in the given list.
    for vert in vertices_list:
        if vert.x == vertex.x and vert.y == vertex.y:
            return True
    return False


def num_of_free_cells(grid_4d, verteces_visited_list, travel_list, start_pos):
    # Return the number of free cells.
    directions = [[0, 1], [1, 0], [0, -1], [-1, 0], [1, 1], [1, -1], [-1, -1], [-1, 1]]
    num_of_free_cells = 1
    temp_row = int(start_pos.x)
    temp_col = int(start_pos.y)
    travel_list.append(start_pos)
    verteces_visited_list.insert(0, start_pos)
    for i in range(len(directions)):
        temp_node = Point(temp_row + directions[i][0], temp_col + directions[i][1])
        if position_valid(temp_node.x, temp_node.y, len(grid_4d), len(grid_4d[0])) and not vertex_in_list(verteces_visited_list, temp_node)\
                and grid_4d[temp_node.x][temp_node.y] is False:
            verteces_visited_list.insert(0, temp_node)
            num_of_free_cells = num_of_free_cells + num_of_free_cells(grid_4d, verteces_visited_list, travel_list, temp_node)
            travel_list.append(start_pos)
    return num_of_free_cells


def drive_path(coordinates_list):
    # Gives the robot destenation from the path to drive.
    global map_resolution, source_x, source_y, robot_size
    division_factor = robot_size / map_resolution
    dest_msg = MoveBaseGoal()
    c = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    c.wait_for_server(rospy.Duration(5))
    # Remove first coordinate from path. (shouldn't go there, it's already there)
    del coordinates_list[0]
    vertexes_visited = []
    for coordinate in coordinates_list:
        row = int(coordinate.y)
        col = int(coordinate.x)
        x = row * math.ceil(division_factor)
        x = source_x + x * map_resolution + robot_size
        y = col * math.ceil(division_factor)
        y = source_y - y * map_resolution - robot_size
        dest_msg.target_pose.header.frame_id = 'map'
        dest_msg.target_pose.header.stamp = rospy.Time.now()
        dest_msg.target_pose.pose.position.x = x
        dest_msg.target_pose.pose.position.y = y
        ang = (dest_msg.target_pose.pose.position.y / dest_msg.target_pose.pose.position.x)
        quaternion = quaternion_from_euler(0, 0, ang * (math.pi / 180))
        q_msg = Quaternion(*quaternion)
        dest_msg.target_pose.pose.orientation = q_msg
        rospy.loginfo("Robot's new location: x = %f, y = %f, theta = %f" % (
            dest_msg.target_pose.pose.position.x, dest_msg.target_pose.pose.position.y, ang))
        c.send_goal(dest_msg)
        c.wait_for_result()
        if c.get_state() == actionlib.GoalStatus.SUCCEEDED:
            if not vertex_in_list(vertexes_visited, coordinate):
                vertexes_visited.insert(0, coordinate)
    return len(vertexes_visited)


def get_value(src, row, col, res):
    # GEt's the value from original map.
    res = int(res)
    dgrid_row = int(row * res)
    dgrid_col = int(col * res)
    for i in range(res):
        res_row = dgrid_row + i
        for k in range(res):
            return src[res_row][dgrid_col + k]
    return False


def transform_robot_location(source, resolution):
    # Get the robot's location.
    global current_robot_row, current_robot_col
    r = 8
    directions = [[-1, -1], [-1, 0], [-1, 1], [0, 1], [1, 1], [1, 0], [1, -1], [0, -1]]
    vertex_queue = []
    finished_list = []
    current_pos = Point(current_robot_row / resolution, current_robot_col / resolution)
    vertex_queue.append(current_pos)
    while vertex_queue:
        current_pos = vertex_queue.pop(0)
        temp_row = int(current_pos.x)
        temp_col = int(current_pos.y)
        if temp_col < 0 or temp_col >= len(source[0]) or temp_row < 0 or temp_row >= len(source):
            rospy.logerr("Trans_err")
            exit(-1)
        if not source[temp_row][temp_col]:
            current_robot_row = temp_row
            current_robot_col = temp_col
            finished_list = []
            vertex_queue = []
            return
        else:
            finished_list.insert(0, current_pos)
        for i in range(r):
            current_pos = Point(temp_row + directions[i][0], temp_col + directions[i][1])
            if position_valid(current_pos.x, current_pos.y, len(source), len(source[0])) and not vertex_in_list(
                    finished_list, current_pos) and not vertex_in_list(vertex_queue, current_pos):
                vertex_queue.append(current_pos)
    exit(-1)


def set_grid(src, dest, res):
    # Set the grid from src node to dest node by resolution of map.
    rows = int(len(src) / res)
    cols = int(len(src[0]) / res)
    for i in range(rows):
        dest[i] = []
        for j in range(cols):
            dest[i].append(get_value(src, i, j, res))
    transform_robot_location(dest, res)


def get_map_data():
    global num_of_rows, num_of_colms
    global source_x, source_y
    global map_resolution, grid
    try:
        # Get the map information and extract the robots start position and map resolution.
        get_static_map = rospy.ServiceProxy('static_map', GetMap)
        map_info = get_static_map()
        rospy.loginfo("Received new location: %d X %d map @ %.3f m/px" % (
            map_info.map.info.width, map_info.map.info.height, map_info.map.info.resolution))
        start_position = map_info.map.info.origin.position
        source_x = start_position.x
        source_y = start_position.y
        num_of_rows = map_info.map.info.height
        num_of_colms = map_info.map.info.width
        map_resolution = map_info.map.info.resolution
        make_occupancy_grid(map_info.map)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)


def init_node():
    # Initialize the node and wait for the map service in order to get it information.
    rospy.init_node("coverage_node", argv=sys.argv)
    rospy.wait_for_service('static_map')


def make_dgrid(grid, division_factor):
    # Create dgrid.
    dgrid = convert_grid_size(division_factor, grid)
    dgrid = dgrid[:-1]
    # Reverse dgrid.
    for i in range(len(dgrid)):
        dgrid[i] = dgrid[i][:-1]
    return dgrid


def set_dict_for_graph(graph):
    # Set the dict for each node.
    for row in graph.vertices:
        for node in row:
            node.set_dict()


def create_graphs(dgrid, grid_4d):
    # The function creates graph from dgrid to 4dgrid.
    graph_d = make_graph(dgrid, "d")
    graph_4d = make_graph(grid_4d, "4d");
    graph_4d.match_4d_to_d(graph_d)
    graph_d.set_neighbours()
    graph_4d.DFS(graph_4d.vertices[0][0])
    remove_direction_by_dfs(graph_4d)
    return graph_d, graph_4d


def write_path_to_file(stc_path):
    # The function writes the path into file.
    file_path = "Coverage_path.txt"
    file = open(file_path, "w")
    for coordinate in stc_path:
        file.write("row: " + str(coordinate.x) + " col: " + str(coordinate.y))
        file.write("\n")
    file.close()


def write_grid_to_file(grid):
    # The function writes the path into file.
    file_path = "new_grid.txt"
    file = open(file_path, "w")
    for row in reversed(grid):
        for cell in row:
            if cell == 1:
                file.write("1")
            else:
                file.write("0")
        file.write("\n")
    file.close()


def make_graph(grid, type):
    # The function creates a reversed graph by type.
    grid.reverse()
    g = Graph(grid, type)
    if type == "4d":
        g.set_neighbours()
    return g


def convert_grid_size(block_size, old_grid):
    # Convert the size of the grid by block_size.
    num_of_cols = int(math.trunc(len(old_grid[0]) / block_size))
    num_of_rows = int(math.trunc(len(old_grid) / block_size))

    cols_grid = make_empty_grid(len(old_grid), num_of_cols, False)
    resized_grid = make_empty_grid(num_of_rows, num_of_cols, False)

    i = 0
    for k in range(len(old_grid)):
        occupied = False
        j = 0
        for l in range(len(old_grid[0])):
            if old_grid[k][l]:
                occupied = True
            if (l % block_size) + 1 == block_size:
                cols_grid[i][j] = occupied
                occupied = False
                j = pp(j)
        i = pp(i)

    j = 0
    for l in range(len(cols_grid[0])):
        i = 0
        occupied = False
        for k in range(len(cols_grid)):
            if cols_grid[k][l]:
                occupied = True
            if (k % block_size) + 1 == block_size:
                resized_grid[i][j] = occupied
                occupied = False
                i = pp(i)
        j = pp(j)
    return resized_grid


def make_occupancy_grid(my_map):
    # Set the occupancies in the grid.
    global map_res, arr_from_map, grid
    global num_of_rows, num_of_colms
    map_res = my_map.info.resolution
    num_of_rows = my_map.info.height
    num_of_colms = my_map.info.width
    arr_from_map[0] = my_map.info.origin.position.x
    arr_from_map[1] = my_map.info.origin.position.y

    # creating the occupancy grid
    grid = make_empty_grid(num_of_rows, num_of_colms, None)
    for i in range(num_of_rows):
        for j in range(num_of_colms):
            if my_map.data[i * num_of_colms + j] == 0:
                grid[i][j] = False
            else:
                grid[i][j] = True
    grid = flip_horizontally(grid)


def make_empty_grid(row, col, val):
    # Create an empty 2D grid.
    empty_grid = []
    for i in range(row):
        r = []
        for j in range(col):
            r.append(val)
        empty_grid.append(r)
    return empty_grid


def flip_horizontally(grid):
    # Flip the grid horizontally.
    flipped_grid = []
    for i in range(len(grid)):
        flipped_grid.append(grid[len(grid) - i - 1])
    return flipped_grid


def main():
    global num_of_rows, num_of_colms
    global source_x, source_y
    global map_resolution, grid
    four_d = 2
    robot_diameter = 0.35
    init_node()
    get_map_data()
    convert_point_to_world()
    division_factor = int(math.ceil(robot_diameter / map_resolution))
    dgrid = make_dgrid(grid, division_factor)
    transform_robot_location(dgrid, division_factor)

    grid_4d = convert_grid_size(four_d, dgrid)
    graph_d, graph_4d = create_graphs(dgrid, grid_4d)

    set_dict_for_graph(graph_d)

    hemiltonian_graph = Hemilton(graph_d.vertices, graph_d.vertices[current_robot_row][current_robot_col],
                              graph_4d.dfs_dict)
    hemiltonian_graph.find_cycle(None, graph_d.vertices[current_robot_row][current_robot_col], True)

    write_path_to_file(hemiltonian_graph.visited_vertices)
    write_grid_to_file(dgrid)

    drive_path(hemiltonian_graph.visited_vertices)  # Needed to run the robot.


if __name__ == '__main__':
    main()
