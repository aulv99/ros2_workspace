import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os

# we store trajectory information here
FILE_PATH = os.path.expanduser("~/trajectory.txt")

# defining the obstacles of the turtlebot3 world simulation
OBSTACLE_RADIUS = 0.2
OBSTACLES = [
    (-1, 1),  (0, 1),  (1, 1),
    (-1, 0),  (0, 0),  (1, 0),
    (-1, -1), (0, -1), (1, -1)
]

def main():
    # reading data
    x_list = []
    y_list = []
    
    try:
        with open(FILE_PATH, 'r') as f:
            for line in f:
                parts = line.split()
                if len(parts) == 2:
                    x_list.append(float(parts[0]))
                    y_list.append(float(parts[1]))
    except FileNotFoundError:
        print(f"Error: Could not find {FILE_PATH}")
        return

    # setup the Plot
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # drawing grid roughly the size of turtlebot3 world
    plt.grid(True, which='both', linestyle='--', alpha=0.5)
    plt.xlim(-2.75, 2.75)
    plt.ylim(-2.75, 2.75)
    
    # drawing the obstacles
    for (ox, oy) in OBSTACLES:
        # creating a circle path
        circle = patches.Circle((ox, oy), OBSTACLE_RADIUS, linewidth=1, edgecolor='r', facecolor='#FFDDDD')
        ax.add_patch(circle)
        # adding a center dot
        ax.plot(ox, oy, 'r+')

    # drawing the robot path
    ax.plot(x_list, y_list, label='Robot Trajectory', color='blue', linewidth=2)
    
    # marking start and end
    if x_list:
        ax.plot(x_list[0], y_list[0], 'go', label='Start') # green Dot
        ax.plot(x_list[-1], y_list[-1], 'bo', label='End') # blue Dot

    # styling
    ax.set_aspect('equal')
    plt.title("Robot Autonomous Path Analysis")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.legend()
    
    # show
    plt.show()

if __name__ == "__main__":
    main()