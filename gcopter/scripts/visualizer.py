import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from corridor_utils import hrep_to_vertices
import scipy.spatial
import plotly.graph_objects as go
from scipy.spatial import ConvexHull
import open3d as o3d

import numpy as np

def load_obstacles(file_path):
    obstacles = []
    with open(file_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) == 3:
                try:
                    point = list(map(float, parts))
                    obstacles.append(point)
                except ValueError:
                    continue
    return np.array(obstacles)


def parse_trajectory_and_corridor(file_path):
    trajectory = []
    corridors = []
    corridor = []
    parsing_trajectory = False
    parsing_corridor = False

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if not line or "Duration" in line or "Total" in line or "Energy" in line:
                continue

            if "Trajectory Positions" in line:
                parsing_trajectory = True
                parsing_corridor = False
                continue

            if "Corridor" in line:
                # If there was a previous corridor collected, save it before starting a new one
                if corridor:
                    corridors.append(corridor)
                    corridor = []
                parsing_corridor = True
                parsing_trajectory = False
                continue

            if "Start" in line or "Goal" in line:
                # Stop parsing altogether on these keywords
                break

            if parsing_trajectory:
                # Try parsing line as floats for trajectory points
                parts = line.split()
                try:
                    numbers = list(map(float, parts))
                    if len(numbers) >= 3:
                        trajectory.append(numbers)
                except ValueError:
                    # Ignore lines that can't be parsed as floats
                    continue

            elif parsing_corridor:
                # Corridor data might include "-----------------" lines to separate halfspaces
                if "-----------------" in line:
                    if corridor:
                        corridors.append(corridor)
                        corridor = []
                else:
                    parts = line.split()
                    try:
                        numbers = list(map(float, parts))
                        corridor.append(numbers)
                    except ValueError:
                        continue

    # Add any remaining corridor collected
    if corridor:
        corridors.append(corridor)

    return np.array(trajectory).T if trajectory else np.empty((0, 0)), corridors


def plot_hyperplane(ax, normal, d, plane_size=5.0, color='b', alpha=0.2):
    # Ensure the normal is normalized
    normal = normal / np.linalg.norm(normal)
    
    # Find a point on the plane (origin shifted by d)
    if np.abs(normal[2]) > 1e-6:
        point = np.array([0, 0, -d / normal[2]])
    elif np.abs(normal[1]) > 1e-6:
        point = np.array([0, -d / normal[1], 0])
    else:
        point = np.array([-d / normal[0], 0, 0])

    # Create two vectors orthogonal to the normal
    v = np.array([1, 0, 0]) if abs(normal[0]) < 0.9 else np.array([0, 1, 0])
    side1 = np.cross(normal, v)
    side1 /= np.linalg.norm(side1)
    side2 = np.cross(normal, side1)

    # Generate the 4 corners of the plane patch
    corners = []
    for dx in [-1, 1]:
        for dy in [-1, 1]:
            corner = point + plane_size * (dx * side1 + dy * side2)
            corners.append(corner)

    # Plot the plane patch
    poly = Poly3DCollection([corners], alpha=alpha, facecolor=color, edgecolor='k')
    ax.add_collection3d(poly)



def plot_trajectory_and_corridor_with_obs(trajectory, corridors, obstacles):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot trajectory as line + points
    if trajectory.size > 0:
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='red', marker='o', label='Trajectory')

    # Plot each corridor as a translucent 3D mesh
    for idx, corridor in enumerate(corridors):
        vertices = hrep_to_vertices(np.array(corridor))
        hull = ConvexHull(vertices)
        print(f"Corridor {idx}: {vertices.shape[0]} vertices, {hull.simplices.shape[0]} hull simplices")
        faces = [vertices[simplex] for simplex in hull.simplices]
        poly3d = Poly3DCollection(faces, alpha=0.1, facecolor='orange', edgecolor='k')
        ax.add_collection3d(poly3d)

    # Plot obstacles as black points
    if obstacles.size > 0:
        ax.scatter(obstacles[:, 0], obstacles[:, 1], obstacles[:, 2], color='black', s=20, label='Obstacles')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Trajectory, Corridor, and Obstacles Visualization')
    ax.legend()

    # Auto-scale axis limits considering all points
    all_points = np.vstack([trajectory] + [hrep_to_vertices(np.array(c)) for c in corridors] + [obstacles]) if corridors else np.vstack([trajectory, obstacles])
    if all_points.size > 0:
        max_range = np.array([all_points[:, 0].max()-all_points[:, 0].min(),
                              all_points[:, 1].max()-all_points[:, 1].min(),
                              all_points[:, 2].max()-all_points[:, 2].min()]).max() / 2.0

        mid_x = (all_points[:, 0].max()+all_points[:, 0].min()) * 0.5
        mid_y = (all_points[:, 1].max()+all_points[:, 1].min()) * 0.5
        mid_z = (all_points[:, 2].max()+all_points[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()





def plot_trajectory_and_corridor(trajectory, corridors):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot trajectory as line + points
    if trajectory.size > 0:
        ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], color='red', marker='o', label='Trajectory')

    # Plot each corridor as a translucent 3D mesh
    for idx, corridor in enumerate(corridors):
        vertices = hrep_to_vertices(np.array(corridor))
        hull = ConvexHull(vertices)
        
        print(f"Corridor {idx}: {vertices.shape[0]} vertices, {hull.simplices.shape[0]} hull simplices")

        # Extract the triangles from hull.simplices and create faces for Poly3DCollection
        faces = [vertices[simplex] for simplex in hull.simplices]
        
        poly3d = Poly3DCollection(faces, alpha=0.1, facecolor='orange', edgecolor='k')
        ax.add_collection3d(poly3d)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Trajectory and Corridor Visualization')
    ax.legend()

    # Auto-scale to the data limits
    all_points = np.vstack([trajectory] + [hrep_to_vertices(np.array(c)) for c in corridors]) if corridors else trajectory
    if all_points.size > 0:
        max_range = np.array([all_points[:, 0].max()-all_points[:, 0].min(),
                              all_points[:, 1].max()-all_points[:, 1].min(),
                              all_points[:, 2].max()-all_points[:, 2].min()]).max() / 2.0

        mid_x = (all_points[:, 0].max()+all_points[:, 0].min()) * 0.5
        mid_y = (all_points[:, 1].max()+all_points[:, 1].min()) * 0.5
        mid_z = (all_points[:, 2].max()+all_points[:, 2].min()) * 0.5

        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()


# Main execution
trajectory, corridors = parse_trajectory_and_corridor('../trajectory.txt')
#obstacles = load_obstacles('../config/obs.txt')


print(f"Parsed trajectory with {len(trajectory)} points and {len(corridors)} corridors.")
print("trajectory points:", trajectory)
plot_trajectory_and_corridor(trajectory, corridors)
