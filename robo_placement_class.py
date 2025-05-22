import numpy as np
import heapq
import string

class A_star_people_placement:
    def __init__(self, filepath):
    # Init function to handle tasks:
    # Read and process the text file containing the grid
    # define the obstacles here into a list
    # find and store the positions of people, robot and the chairs

        self.grid = self.read_and_load_grid(filepath)
        self.row, self.column = self.grid.shape
        self.robot_pos = self.get_robot_pos()
        self.people_pos = self.get_people_pos()
        self.chair_pos = self.get_chair_pos()
        self.static_obstacles = {'W', 'A', 'F', 'k', 'G', 'T', 'H', 'S', 'c', 'b'}

    def read_and_load_grid(self, filepath):
        # Read and store the grid as a numpy array
        # Process as the data as: read each line as a list of characters 
        # and return the list as a numpy array
        with open(filepath, 'r') as r:
            lines = r.read().splitlines()
        return np.array([line.split() for line in lines])

    def get_robot_pos(self):
        # Use a nested for loop to process the grid
        # Look for the character 'r' to find the grid in which robot is present
        for i in range(self.row - 1):
            for j in range(self.column - 1):
                if self.grid[i][j] == 'r' and self.grid[i][j + 1] == 'r' and \
                   self.grid[i + 1][j] == 'r' and self.grid[i + 1][j + 1] == 'r':
                    return (i, j)

    def get_people_pos(self):
        # Here it is similar to previous function
        # iterate over the grid and look for numbers to detect the people
        # Store the people as a dictionary with number as key and position co-ordinates as the value
        people = {}
        for i in range(self.row):
            for j in range(self.column):
                if self.grid[i][j] in string.digits:
                    people[self.grid[i][j]] = (i, j)
        return people

    def get_chair_pos(self):
        # Again we do the same but store the position of chairs as a list.
        chairs = {}
        count = 1
        for i in range(self.row):
            for j in range(self.column):
                if self.grid[i][j] == 'H':
                    label = f'H{count}'
                    chairs[label] = (i, j)
                    count += 1
        return chairs
        
    def check_if_position_valid(self, row, column, blocked):
        # First make sure the robot is operating in the valid region of the region
        if row < 0 or column < 0 or row + 1 >= self.row or column + 1 >= self.column:
            return False
        # Now just make sure the 2x2 grid is open and robot will not encounter blocked cells
        for rw in range(2):
            for cl in range(2):
                sub_grid = self.grid[row + rw][column + cl]
                if sub_grid in blocked or (row + rw, column + cl) in blocked:
                    return False
        return True

    def get_next_cell(self, pos, blocked):
        # here we check in all 4 directions if the cell is vacant or not.
        # Just returns the top left corner position of the robot as the first element of the list of tuples.
        row, column = pos
        directions = [(1, 0), (-1, 0), (0, -1), (0, 1)]
        neighbour = []
        for rw, cl in directions:
            new_rw = row + rw
            new_cl = column + cl
            if self.check_if_position_valid(new_rw, new_cl, blocked):
                neighbour.append((new_rw, new_cl))
        return neighbour

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def apply_astar(self, start, goal, blocked):
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start))
        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, cost, current = heapq.heappop(open_set)
            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_next_cell(current, blocked):
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, tentative_g, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def get_adjacent_valid_positions(self, chair, blocked):
        r, c = chair
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # up, down, left, right
        positions = []
        for dr, dc in directions:
            top_left = (r + dr, c + dc)
        if self.check_if_position_valid(top_left[0], top_left[1], blocked):
            positions.append(top_left)
        return positions


    def find_best_assignments(self):
        robot = self.robot_pos
        assignments = {}
        temp_people = self.people_pos.copy()
        temp_chairs = self.chair_pos.copy()

        while temp_people:
            best_person = None
            best_chair = None
            best_path_len = float('inf')

            for pid, p_pos in temp_people.items():
                blocked = self.static_obstacles.union(
                    {p for p in string.digits if p != pid})
                blocked_positions = {loc for pid2, loc in temp_people.items() if pid2 != pid}
                path_to_person = self.apply_astar(robot, p_pos, blocked.union(blocked_positions))


                if not path_to_person:
                    continue

                for label, chair in temp_chairs.items():
                    blocked = self.static_obstacles.union(set(temp_people.values())) - {'H'}
                    adjacent_positions = self.get_adjacent_valid_positions(chair, blocked)
                    for adj_pos in adjacent_positions:
                        path_to_chair = self.apply_astar(p_pos, adj_pos, blocked)
                        if path_to_chair:
                            total_len = len(path_to_person) + len(path_to_chair)
                            if total_len < best_path_len:
                                best_path_len = total_len
                                best_person = pid
                                best_chair = label

            if not best_person or not best_chair:
                print("No more valid assignments can be made.")
                break

            assignments[best_person] = best_chair
            del temp_people[best_person]
            del temp_chairs[best_chair]
            robot = self.people_pos[best_person]  # Update for next round

        return assignments
    
    def seat_people_a_star(self, assignment_dict):
        robot = self.robot_pos
        total_cost = 0

        for pid, chair_label in assignment_dict.items():
            person_pos = self.people_pos[pid]
            chair_pos = self.chair_pos[chair_label]

            blocked = self.static_obstacles.union({p for p in string.digits if p != pid})
            other_people = {v for k, v in self.people_pos.items() if k != pid}
            path_to_person = self.apply_astar(robot, person_pos, blocked.union(other_people))

            if not path_to_person:
                print(f"Failed to reach person {pid}")
                continue

            robot = path_to_person[-1]
            print(f"For person {pid} and seated in chair {chair_label} best path is : {path_to_person}")
            total_cost += len(path_to_person)

            # Find path to adjacent chair position
            blocked_chair = self.static_obstacles.union(set(self.people_pos.values())) - {'H'}
            best_chair_path = None
            min_len = float('inf')

            for adj in self.get_adjacent_valid_positions(chair_pos, blocked_chair):
                path = self.apply_astar(robot, adj, blocked_chair)
                if path and len(path) < min_len:
                    best_chair_path = path
                    min_len = len(path)

            if not best_chair_path:
                print(f"Failed to deliver person {pid} to chair {chair_label}")
                continue

            robot = best_chair_path[-1]
            total_cost += len(best_chair_path)

            print(f"Delivered person {pid} to {chair_label} with total cost: {len(path_to_person) + len(best_chair_path)}")

        print(f"\nTotal operation cost: {total_cost}")
        return total_cost






if __name__ == '__main__':
    planner = A_star_people_placement('home.txt')
    assignments = planner.find_best_assignments()
    total_cost = planner.seat_people_a_star(assignments)
    print("Total cost:", total_cost)

        




