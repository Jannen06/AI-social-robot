import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance
import networkx as nx


from home_layout import generate_house_layout
grid, rooms = generate_house_layout(
    file_name='/home/jannen/Documents/MAS2025/AI/Assignments/home.txt')

file_path = ...


# Read the Layout from the txt file
def read_layout(file_path='home.txt'):
    file_path = file_path
    home = []
    with open(file_path, 'r') as file:

        for line in file:
            row = line.strip().split(' ')
            home.append(row)
    home = np.array(home)
    return home


# print(rooms.items())
# Slice the rooms


rooms_dict = {}

for room_type, (row_slice, col_slice) in rooms.items():
    slices = []

    for y in range(row_slice.start, row_slice.stop):
        for x in range(col_slice.start, col_slice.stop):

            slices.append((y, x))
    # Assign the slices of matrix to each key room_type as dictionary in rooms_slice
    rooms_dict[room_type] = slices

# print(rooms_dict)

# Retrieving the room name by the tuple


def get_room_of_position(position=()):
    tuple = position
    for room, position_r in rooms_dict.items():
        if tuple in position_r:
            return room
    return None


def topology_graph(rooms_dict, grid):
    # rooms_dict = rooms_dict
    # grid = grid
    graph_edges = set()
    robot_room = ""

    for x in range(grid.shape[0]):
        for y in range(grid.shape[1]):

            if grid[x][y] == 'd':
                adjacent_rooms = set()  # Moved inside to reset for each door
                for d_from_x, d_from_y in [(0, -2), (0, 2), (-2, 0), (2, 0)]:
                    m_x = x + d_from_x
                    m_y = y + d_from_y
                    room = get_room_of_position(position=(m_x, m_y))
                    if room:
                        adjacent_rooms.add(room)

                if len(adjacent_rooms) >= 2:
                    adj_list = list(adjacent_rooms)
                    for i in range(len(adj_list)):
                        for j in range(i + 1, len(adj_list)):
                            graph_edges.add(
                                tuple(sorted((adj_list[i], adj_list[j]))))

            if grid[x][y] == 'r':
                # Moved inside to check for robot position.
                # robot_position = (i, j)
                robot_room = set()

                for d_from_x, d_from_y in [(0, -2), (0, 2), (-2, 0), (2, 0)]:
                    m_x = x + d_from_x
                    m_y = y + d_from_y
                    robot_position = get_room_of_position(
                        position=(m_x, m_y))
                    if robot_position:
                        robot_room = robot_position
                        # robot_edge.add(tuple(robot_position))

    return graph_edges, robot_room


##################################
# USAGE
file_path = "/home/jannen/Documents/MAS2025/AI/Assignments/home.txt"
grid = read_layout(file_path)
print(rooms.items())


##########################################
##########################################
G = nx.Graph()

graph_edges, robot_room = topology_graph(rooms_dict, grid)
G.add_node("r")
G.add_nodes_from(rooms_dict.keys())
G.add_edges_from(graph_edges)
G.add_edge("r", robot_room)

plt.figure(figsize=(8, 6))
nx.draw(G, with_labels=True, node_size=2000,
        node_color="lightblue", font_size=10)
plt.title("Topological Map of House Rooms")
plt.show()
