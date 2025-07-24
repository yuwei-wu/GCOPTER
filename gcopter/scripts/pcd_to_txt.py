import open3d as o3d
import numpy as np

# Load PCD file
pcd = o3d.io.read_point_cloud("../config/pt100001.pcd")

# Convert to NumPy array
points = np.asarray(pcd.points)

#visualize
o3d.visualization.draw_geometries([pcd])

# Save to TXT file
np.savetxt("../config/obs.txt", points, fmt="%.6f", delimiter=' ')

print(f"Saved {points.shape[0]} points to obs.txt")
