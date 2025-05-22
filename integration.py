# Simplified Integration of Seating CSP + A* + Topological Map

# --- Constraint Solver (Mocked Output for Simplicity) ---
from collections import deque
seating_assignment = {
    'P1': 's1', 'P2': 's2', 'P3': 's3', 'P4': 's4',
    'P5': 's5', 'P6': 's6', 'P7': 's7', 'P8': 's8'
}  # Normally from SeatingCSP().solve()[0]

# --- Topological Map (Example Graph) ---
topological_map = {
    'robot_start': ['n1'],
    'n1': ['robot_start', 'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7', 'P8'],
    'P1': ['n1'], 'P2': ['n1'], 'P3': ['n1'], 'P4': ['n1'],
    'P5': ['n1'], 'P6': ['n1'], 'P7': ['n1'], 'P8': ['n1'],
    's1': ['P1'], 's2': ['P2'], 's3': ['P3'], 's4': ['P4'],
    's5': ['P5'], 's6': ['P6'], 's7': ['P7'], 's8': ['P8']
}

# --- A* Search (Simple BFS as Placeholder) ---


def a_star(graph, start, goal):
    queue = deque([(start, [start], 0)])
    visited = set()

    while queue:
        current, path, cost = queue.popleft()
        if current == goal:
            return path, cost

        visited.add(current)
        for neighbor in graph.get(current, []):
            if neighbor not in visited:
                queue.append((neighbor, path + [neighbor], cost + 1))

    return None, float('inf')  # If no path found


# --- Integration Logic ---
total_cost = 0

for guest, assigned_seat in seating_assignment.items():
    guest_node = guest
    seat_node = assigned_seat

    path_to_guest, cost1 = a_star(topological_map, 'robot_start', guest_node)
    path_to_seat, cost2 = a_star(topological_map, guest_node, seat_node)

    total_cost += cost1 + cost2

    print(f"\nGuiding {guest}:")
    print(f"  Path to guest: {path_to_guest} (Cost: {cost1})")
    print(f"  Path to seat: {path_to_seat} (Cost: {cost2})")
    print(f"  Assigned seat: {assigned_seat}")

print(f"\nTotal cost of all paths: {total_cost}")

# --- Final Seating Output ---
print("\nFinal Seating Arrangement:")
for person, seat in seating_assignment.items():
    print(f"{person} -> {seat}")
