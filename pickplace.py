import pybullet as p
import pybullet_data
import time
import numpy as np
import heapq
import matplotlib.pyplot as plt

# Initialize PyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load plane and KUKA robotic arm
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])  # KUKA iiwa model

# Load object (cube) at a specific position (starting position of the object)
object_position = [0.5, 0, 0.3]  # Initial position of the object
cube_id = p.loadURDF("cube.urdf", object_position, globalScaling=0.5)  # Smaller cube

# Define the goal position for placing the object
goal_position = [1.4, 4.7, 2.3]

# Define the end effector index (last joint of KUKA arm)
end_effector_index = 6  # Adjust as per your robot's end effector joint index

# Define the grid size and obstacles with finer resolution
grid_size = (100, 100)  # Larger grid for more search space
grid_resolution = 0.1  # 10 cm per grid cell for improved pathfinding

# Helper for grid conversion
def world_to_grid(world_pos):
    return int(world_pos[0] / grid_resolution), int(world_pos[1] / grid_resolution)

def grid_to_world(grid_pos):
    return grid_pos[0] * grid_resolution, grid_pos[1] * grid_resolution, 0.3

# A* Algorithm with Obstacle Avoidance
def a_star(start, goal):
    start = world_to_grid(start)
    goal = world_to_grid(goal)
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + distance(current, neighbor)

            if neighbor in obstacles:
                continue  # Skip obstacles

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    print("No path found. Consider revising the obstacle layout or increasing grid resolution.")
    return None  # No path found

def heuristic(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def distance(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return [grid_to_world(cell) for cell in path]

def get_neighbors(cell):
    x, y = cell
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1),
                  (-1, -1), (1, 1), (-1, 1), (1, -1)]
    neighbors = []
    for dx, dy in directions:
        neighbor = (x + dx, y + dy)
        if (0 <= neighbor[0] < grid_size[0] and
            0 <= neighbor[1] < grid_size[1]):
            neighbors.append(neighbor)
    return neighbors

# Create obstacles with an additional one added
def create_obstacles_simple(grid_size):
    obstacles = []
    # Add a few simple obstacles that won't block the goal position
    obstacles += [(10, 20), (11, 20), (12, 20)]
    obstacles += [(25, 30), (26, 30), (27, 30)]
    obstacles += [(15, 25)]  # Added new obstacle here
    return obstacles

# Initialize obstacles with the new one included
obstacles = create_obstacles_simple(grid_size)


# Function to add obstacles to PyBullet simulation
def add_obstacles_to_simulation(obstacles):
    obstacle_ids = []
    for (x, y) in obstacles:
        # Convert grid position to world coordinates
        world_position = grid_to_world((x, y))
        # Add an obstacle (a cube) at the calculated world position
        cube_id = p.loadURDF("cube.urdf", world_position, globalScaling=0.2)
        obstacle_ids.append(cube_id)
    return obstacle_ids

# Add obstacles to the PyBullet simulation
obstacle_ids = add_obstacles_to_simulation(obstacles)

# Function to visualize the grid and obstacles (for debugging)
def visualize_grid():
    grid = np.zeros(grid_size)
    for (x, y) in obstacles:
        if 0 <= x < grid_size[0] and 0 <= y < grid_size[1]:
            grid[y][x] = 1  # Mark obstacles

    start_grid = world_to_grid(object_position)
    goal_grid = world_to_grid(goal_position)

    grid[start_grid[1]][start_grid[0]] = 0.5  # Start position
    grid[goal_grid[1]][goal_grid[0]] = 0.75  # Goal position

    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap='gray')
    plt.title('Grid Map: Obstacles (White), Start (Gray), Goal (Light Gray)')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.show()

# Step 1: Visualize the grid and obstacles for debugging
visualize_grid()

# Step 2: Place the object at a specific position (already placed in the environment)
p.resetBasePositionAndOrientation(cube_id, object_position, [0, 0, 0, 1])

# Step 3: Perform A* path planning to the goal with obstacle avoidance
def plan_path_with_retries(start, goal, retries=3):
    for attempt in range(retries):
        print(f"Attempt {attempt + 1}/{retries} to find path...")
        path = a_star(start, goal)
        if path:
            return path
        else:
            print("Path not found. Retrying...")
            # Optionally, modify obstacles or grid resolution here before retrying
            # obstacles.append((np.random.randint(grid_size[0]), np.random.randint(grid_size[1])))  # Example obstacle change
            time.sleep(1)  # Add a slight delay between retries
    print("Failed to find a path after multiple attempts.")
    return None  # Return None if no path is found after retries

# Plan path with retries
path_to_goal = plan_path_with_retries(object_position, goal_position)

# Optional: Visualize the found path
if path_to_goal:
    path_grid = [world_to_grid(pos) for pos in path_to_goal]
    grid = np.zeros(grid_size)
    for (x, y) in obstacles:
        if 0 <= x < grid_size[0] and 0 <= y < grid_size[1]:
            grid[y][x] = 1  # Obstacles
    for (x, y) in path_grid:
        grid[y][x] = 0.5  # Path

    start_grid = world_to_grid(object_position)
    goal_grid = world_to_grid(goal_position)
    grid[start_grid[1]][start_grid[0]] = 0.75  # Start
    grid[goal_grid[1]][goal_grid[0]] = 0.25  # Goal

    plt.figure(figsize=(8, 8))
    plt.imshow(grid, cmap='gray')
    plt.title('Path Visualization: Obstacles (Black), Path (Gray), Start (Light Gray), Goal (Darker Gray)')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.show()

# Step 4: Move robot along the found path, holding the object and placing it at the goal
def move_base_and_arm_with_object(robot_id, path, end_effector_goal, move_object=False):
    if path is None:
        print("No path found. Please check the environment or revise the grid.")
        return  # Exit if no path found

    for base_position in path:
        # Move the base to the new position
        p.resetBasePositionAndOrientation(robot_id, base_position, [0, 0, 0, 1])

        # Calculate joint angles to position the end effector at the goal relative to the base
        joint_angles = p.calculateInverseKinematics(robot_id, end_effector_index, end_effector_goal)
        
        # Apply joint control for the calculated angles
        for i, joint_angle in enumerate(joint_angles):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_angle)
        
        # If move_object is True, update the object position to match the end effector's position (holding it)
        if move_object:
            end_effector_position = p.getLinkState(robot_id, end_effector_index)[0]
            p.resetBasePositionAndOrientation(cube_id, end_effector_position, [0, 0, 0, 1])

        time.sleep(1)  # Pause to simulate movement

    # Once the robot reaches the goal, place the object
    p.resetBasePositionAndOrientation(cube_id, end_effector_goal, [0, 0, 0, 1])
    print("Object placed at goal.")

# Move the robot and object to the goal
move_base_and_arm_with_object(robot_id, path_to_goal, goal_position, move_object=True)

# Disconnect from PyBullet
#p.disconnect()
