import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import random 

class PotentialFieldPathPlanner:
    def __init__(self, obstacles, start, goal, attractive_gain, repulsive_gain):
        self.obstacles = obstacles
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.attractive_gain = attractive_gain
        self.repulsive_gain = repulsive_gain
        self.step_size = 0.1  # Example step size
        self.max_iterations = 10000  # Example max iterations

    def attractive_force(self, position):
        direction = self.goal - position
        magnitude = np.linalg.norm(direction)
        return self.attractive_gain * (direction / magnitude if magnitude != 0 else np.zeros(3))

    def repulsive_force(self, position):
        force = np.zeros(3)
        for (ox, oy, oz, dx, dy, dz) in self.obstacles:
            if position[2] < oz + dz:  # Check if the agent's height is below the height of the obstacle
                adjusted_obstacle_center = np.array([ox + dx / 2, oy + dy / 2, position[2]])
                direction = position - adjusted_obstacle_center
                distance = np.linalg.norm(direction)
                safety_radius = np.sqrt((dx / 2) ** 2 + (dy / 2) ** 2)
                magnitude = self.repulsive_gain * 1 / (distance - safety_radius) ** 2
                force += (direction / distance) * magnitude
        return force

    def potential_field(self, position):
        return self.attractive_force(position) + self.repulsive_force(position)

    def advance_state(self, current_state):
        force = self.potential_field(current_state)
        next_state = current_state + self.step_size * force
        return next_state

    def is_within_safety_radius(self, position):
        for (ox, oy, oz, dx, dy, dz) in self.obstacles:
            obstacle_height = oz + dz
            if position[2] <= obstacle_height:  # Check if the agent's height is below or at the height of the obstacle
                obstacle_center_xy = np.array([ox + dx / 2, oy + dy / 2])
                position_xy = np.array([position[0], position[1]])
                distance_xy = np.linalg.norm(position_xy - obstacle_center_xy)
                safety_radius = np.sqrt((dx / 2) ** 2 + (dy / 2) ** 2)
                if distance_xy <= safety_radius:
                    return True
        return False

    def plan_path(self):
        path = [self.start]
        current_state = np.array(self.start)
        iterations = 0
        while np.linalg.norm(self.goal - current_state) > self.step_size and iterations < self.max_iterations:
            if self.is_within_safety_radius(current_state):
                print("Error: Agent came within the safety radius of an obstacle.")
                return path, float('inf')  # Return path up to collision and infinite cost
            current_state = self.advance_state(current_state)
            path.append(current_state)
            iterations += 1
        if self.is_within_safety_radius(current_state):
            print("Error: Agent came within the safety radius of an obstacle.")
            return path, float('inf')  # Return path up to collision and infinite cost
        
        # Only append the goal if the agent reached it within the step size
        if np.linalg.norm(self.goal - current_state) <= self.step_size:
            path.append(self.goal)
        cost = self.calculate_cost(path)
        return path, cost

    def calculate_cost(self, path):
        
        # If the final path point is not within the step size of the goal, return an infinite cost
        if np.linalg.norm(self.goal - path[-1]) > self.step_size:
            return float('inf')

        total_distance = 0
        for point in path:
            point_xy = np.array([point[0], point[1]])
            min_distance = float('inf')
            for (ox, oy, oz, dx, dy, dz) in self.obstacles:
                if point[2] < oz + dz:  # Only consider obstacles above the agent's height
                    obstacle_center_xy = np.array([ox + dx / 2, oy + dy / 2])
                    distance_xy = np.linalg.norm(point_xy - obstacle_center_xy)
                    safety_radius = np.sqrt((dx / 2) ** 2 + (dy / 2) ** 2)
                    adjusted_distance = distance_xy - safety_radius
                    if adjusted_distance < min_distance:
                        min_distance = adjusted_distance
            total_distance += min_distance ** 2

        mse = total_distance / len(path)
        return 1 / mse if mse != 0 else float('inf')  # Return the inverse of the mean square distance

def visualize_path(obstacles, path, start, goal, filename=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot obstacles
    for (ox, oy, oz, dx, dy, dz) in obstacles:
        ax.bar3d(ox, oy, 0, dx, dy, dz, color='r', alpha=0.5)  # Ensure bottom face is on Z=0

    # Plot start and goal
    ax.scatter(start[0], start[1], start[2], color='g', s=100, label='Start')
    ax.scatter(goal[0], goal[1], goal[2], color='b', s=100, label='Goal')

    # Plot the path
    if path:  # Only plot if path is not empty
        path = np.array(path)
        line, = ax.plot([], [], [], marker='o')

        def update(num):
            line.set_data(path[:num, 0], path[:num, 1])
            line.set_3d_properties(path[:num, 2])
            return line,

        ani = FuncAnimation(fig, update, frames=len(path), interval=10, blit=True)

        if filename:
            ani.save(filename, writer='imagemagick')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

def find_best_gains(obstacles, start, goals, attractive_gains, repulsive_gains):
    best_gains = None
    best_fitness = float('inf')

    total_combinations = len(attractive_gains) * len(repulsive_gains)
    combination_count = 0

    for attractive_gain in attractive_gains:
        for repulsive_gain in repulsive_gains:
            combination_count += 1
            print(f"Testing combination {combination_count}/{total_combinations}: Attractive Gain = {attractive_gain}, Repulsive Gain = {repulsive_gain}")
            
            costs = []
            for goal in goals:
                print(f"  Testing goal {goal}")
                planner = PotentialFieldPathPlanner(obstacles, start, goal, attractive_gain, repulsive_gain)
                path, cost = planner.plan_path()
                costs.append(cost)
                print(f"    Cost for goal {goal}: {cost}")
            
            average_cost = np.mean(costs)
            print(f"  Average cost for combination {combination_count}: {average_cost}")
            
            if average_cost < best_fitness:
                best_fitness = average_cost
                best_gains = (attractive_gain, repulsive_gain)
                print(f"  New best gains found: Attractive Gain = {best_gains[0]}, Repulsive Gain = {best_gains[1]} with Average Cost = {best_fitness}")
    
    return best_gains, best_fitness

# Example usage
obstacles = [
    (2, 2, 0, 1, 1, 6),
    (5, 2, 0, 1, 1, 6),
    (1, 5, 0, 1, 1, 6),
    (5, 5, 0, 1, 1, 6),
    (2, 7, 0, 1, 1, 6),
    (7, 3, 0, 1, 1, 6),
    (4, 1, 0, 1, 1, 6),
    (1, 1, 0, 1, 1, 6),
    (4, 7, 0, 1, 1, 6),
    (7, 7, 0, 1, 1, 6)
]

start = (2, 4, 0)
goals = [(8, 6, 3), (1, 8, 0), (3, 1, 0)]

attractive_gains = np.linspace(1.0, 2.0, 5)
repulsive_gains = np.linspace(0.1, 2.0, 5)

best_gains, best_fitness = find_best_gains(obstacles, start, goals, attractive_gains, repulsive_gains)
print(f"Best gains: Attractive Gain = {best_gains[0]}, Repulsive Gain = {best_gains[1]}")
print(f"Best MSE Cost: {best_fitness}")


# Visualize the path for the best gains and each goal
for goal in goals:
    planner = PotentialFieldPathPlanner(obstacles, start, goal, best_gains[0], best_gains[1])
    path, cost = planner.plan_path()
    visualize_path(obstacles, path, start, goal, f'potential_field_{goal}.gif')