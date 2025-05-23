import numpy as np
import math
import matplotlib.pyplot as plt
import heapq

file_path = 'home.txt'

home = []
with open(file_path, 'r') as file:
    for line in file:
        row = line.strip().split(' ')
        home.append(row)

initial_grid = np.array(home)


def get_heuristic(x, y, goal):
    return abs(x - goal[0]) + abs(y - goal[1])


def is_valid_move(grid, row, col):
    rows, cols = grid.shape
    return 0 <= row < rows - 1 and 0 <= col < cols - 1 and \
        grid[row][col] != 'W' and grid[row + 1][col] != 'W' and \
        grid[row][col + 1] != 'W' and grid[row + 1][col + 1] != 'W'


def locate_neighbours(grid, current_pos, blocked_cells=None):
    r, c = current_pos
    neighbours = []
    movements = [(0, 1), (0, -1), (1, 0), (-1, 0)]

    for dr, dc in movements:
        new_r, new_c = r + dr, c + dc
        if is_valid_move(grid, new_r, new_c):
            neighbours.append((new_r, new_c))
    return neighbours


def implement_a_star(grid, start, goal, blocked_cells=None):
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    f_score = {start: get_heuristic(*start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor in locate_neighbours(grid, current, blocked_cells):
            tentative_g_score = g_score[current] + 1

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + \
                    get_heuristic(*neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None


def locate_persons_goals_robot(grid):
    persons = {}
    goals = {}
    robot = None
    count = 0
    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i][j].isdigit():
                persons[int(grid[i][j])] = (i, j)
            if grid[i][j] == 'H':
                goals[str(grid[i][j]) + str(count)] = (i, j)
                count += 1
            if grid[i][j] == 'r':
                robot = (i, j)  # Top-left of the robot
    return goals, persons, robot


def update_robot_position(grid, old_pos, new_pos):
    # Clear old robot position
    for i in range(2):
        for j in range(2):
            grid[old_pos[0] + i][old_pos[1] + j] = '.'

    # Set new robot position
    for i in range(2):
        for j in range(2):
            grid[new_pos[0] + i][new_pos[1] + j] = 'r'
    return grid


def update_grid(grid, replace_coordinate, replace_char):
    grid[replace_coordinate[0]][replace_coordinate[1]] = replace_char
    return grid


def visualize_char_grid(char_array):
    rows, cols = np.array(char_array).shape
    fig, ax = plt.subplots()
    ax.axis('off')
    font = {'family': 'monospace', 'size': 10}

    for i in range(rows):
        for j in range(cols):
            ax.text(j, -i, char_array[i][j],
                    ha='center', va='center', fontdict=font)

    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(-rows + 0.5, 0.5)
    plt.title("Character Array Display")
    plt.tight_layout()
    plt.show()


goals, persons, robot_pos = locate_persons_goals_robot(initial_grid)

person_to_goal_dict = {}

while persons:
    updated_grid = initial_grid.copy()

    # Block cells occupied by the robot
    blocked_cells = set()
    for i in range(2):
        for j in range(2):
            blocked_cells.add((robot_pos[0] + i, robot_pos[1] + j))

    closest_person = None
    min_dist = float('inf')

    for person, person_loc in persons.items():
        dist = get_heuristic(*robot_pos, person_loc)
        if dist < min_dist:
            min_dist = dist
            closest_person = person

    if closest_person is not None:
        person_loc = persons[closest_person]
        path_to_person = implement_a_star(
            updated_grid, robot_pos, person_loc, blocked_cells)

        if path_to_person:
            print(f"Path to person {closest_person}: {path_to_person}")

            # Move robot to the person
            for next_pos in path_to_person:
                updated_grid = update_robot_position(
                    updated_grid, robot_pos, next_pos)
                # visualize_char_grid(updated_grid)
                robot_pos = next_pos

            initial_grid = updated_grid.copy()

            # Clear blocked cells after robot moves
            blocked_cells = set()
            for i in range(2):
                for j in range(2):
                    blocked_cells.add((robot_pos[0] + i, robot_pos[1] + j))

            # Find path to goal
            min_path_length = float('inf')
            chosen_goal = None

            for goal, goal_loc in goals.items():
                path_to_goal = implement_a_star(
                    updated_grid, robot_pos, goal_loc, blocked_cells)
                if path_to_goal:
                    path_length = len(path_to_goal)
                    if path_length < min_path_length:
                        min_path_length = path_length
                        chosen_goal = goal

            if chosen_goal:
                print(
                    f"Chosen goal for person {closest_person}: {chosen_goal} at {goals[chosen_goal]}")

                # Move robot with person to the goal
                goal_path = implement_a_star(
                    updated_grid, robot_pos, goals[chosen_goal], blocked_cells)
                if goal_path:
                    for next_pos in goal_path:
                        updated_grid = update_robot_position(
                            updated_grid, robot_pos, next_pos)
                        # visualize_char_grid(updated_grid)
                        robot_pos = next_pos
                    initial_grid = updated_grid.copy()

                person_to_goal_dict[closest_person] = chosen_goal
                del persons[closest_person]
                del goals[chosen_goal]
            else:
                print(
                    f"No path found from robot to any available goal for person {closest_person}.")
                break
        else:
            print(f"No path found to person {closest_person}.")
            break
    else:
        print("No persons left to process.")
        break

print("\nAll possible goals have been assigned (or no more paths found).")
print("Person to Goal Assignments:", person_to_goal_dict)
