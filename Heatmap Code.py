#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import numpy as np
import matplotlib.pyplot as plt

# Function to check if a point is inside a triangle
def point_in_triangle(p, p0, p1, p2):
    def sign(a, b, c):
        return (a[0] - c[0]) * (b[1] - c[1]) - (b[0] - c[0]) * (a[1] - c[1])
    d1 = sign(p, p0, p1)
    d2 = sign(p, p1, p2)
    d3 = sign(p, p2, p0)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)

# Function to combine confidence zones into a heatmap
def combine_confidence_zones(confidence_zones_A, confidence_zones_B, grid_size=100):
    # Create a grid for the heatmap
    heatmap = np.zeros((grid_size, grid_size))

    # Define the area covered by the heatmap (e.g., 60m x 60m)
    x_range = np.linspace(-30, 30, grid_size)
    y_range = np.linspace(-30, 30, grid_size)

    # Iterate over the grid and update the heatmap
    for i in range(grid_size):
        for j in range(grid_size):
            point = np.array([x_range[i], y_range[j], 0])  # 2D point in the grid
            # Check if the point is in any confidence zone from Vehicle A
            for triangle in confidence_zones_A:
                if point_in_triangle(point, triangle[0], triangle[1], triangle[2]):
                    heatmap[i, j] += 0.5  # Add 0.5 for Vehicle A
            # Check if the point is in any confidence zone from Vehicle B
            for triangle in confidence_zones_B:
                if point_in_triangle(point, triangle[0], triangle[1], triangle[2]):
                    heatmap[i, j] += 0.5  # Add 0.5 for Vehicle B
            # Cap the heatmap value at 1
            heatmap[i, j] = min(heatmap[i, j], 1)
    
    return heatmap, x_range, y_range

# Function to visualize the heatmap
def visualize_heatmap(heatmap, x_range, y_range):
    plt.figure(figsize=(8, 6))
    plt.imshow(heatmap, extent=[x_range.min(), x_range.max(), y_range.min(), y_range.max()], origin='lower', cmap='hot')
    plt.colorbar(label="Confidence Level")
    plt.title("Combined Confidence Heatmap")
    plt.xlabel("X Position (meters)")
    plt.ylabel("Y Position (meters)")
    plt.show()

# Combine confidence zones into a heatmap
heatmap, x_range, y_range = combine_confidence_zones(confidence_zones_A, confidence_zones_B)

# Visualize the heatmap
visualize_heatmap(heatmap, x_range, y_range)


# In[ ]:


import numpy as np
import open3d as o3d

# Load LiDAR data from .ply files
def load_lidar_data(ply_file_path):
    pc_ply = o3d.io.read_point_cloud(ply_file_path)
    print(f"PLY file loaded: {ply_file_path}")
    points = np.asarray(pc_ply.points)  # Extract points (X, Y, Z)
    return points

# Calculate distances and angles
def calculate_distances_angles(points):
    distances = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)  # Distance from LiDAR
    angles = np.arctan2(points[:, 1], points[:, 0]) * 180 / np.pi  # Angle in degrees
    return distances, angles

# Generate confidence zones
def generate_confidence_zones(points, distances, angles):
    delta_angle = 360 / 52  # Angular resolution of LiDAR
    confidence_zones = []

    for i in range(len(points) - 1):
        # Check conditions for points[i] and points[i+1]
        if (points[i, 2] < -1 and points[i + 1, 2] < -1 and  # Z-coordinate < -1
            distances[i] < 30 and distances[i + 1] < 30 and  # Distance < 30 meters
            0 < angles[i + 1] - angles[i] < 1.25 * delta_angle):  # Angle difference condition
            # Mark the triangle as a confidence zone
            confidence_zones.append([points[i], points[i + 1], [0, 0, 0]])  # Triangle: points[i], points[i+1], (0,0)
    
    return np.array(confidence_zones)

# Load LiDAR data for Vehicle A and Vehicle B
points_A = load_lidar_data(r"C:\Users\OTW100920082\Downloads\A_001.ply")  # Use raw string
points_B = load_lidar_data(r"C:\Users\OTW100920082\Downloads\B_001.ply")  # Use raw string

# Calculate distances and angles for Vehicle A and Vehicle B
distances_A, angles_A = calculate_distances_angles(points_A)
distances_B, angles_B = calculate_distances_angles(points_B)

# Generate confidence zones for Vehicle A and Vehicle B
confidence_zones_A = generate_confidence_zones(points_A, distances_A, angles_A)
confidence_zones_B = generate_confidence_zones(points_B, distances_B, angles_B)

# Print results
print(f"Number of confidence zones for Vehicle A: {len(confidence_zones_A)}")
print(f"Number of confidence zones for Vehicle B: {len(confidence_zones_B)}")

