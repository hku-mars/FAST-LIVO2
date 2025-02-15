import os
import glob
import vdbfusion
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree

# ------------------------- Key Parameters -------------------------
# Dataset path
SOURCE_DIR = "/home/chunran/Downloads/"  # Replace with your dataset path

# VDBVolume parameters
VOXEL_SIZE = 0.02  # Voxel size (smaller values increase precision but require more memory)
SDF_TRUNC = 0.1    # Truncation distance for SDF (affects surface thickness, typically a multiple of voxel size)
MIN_WEIGHT = 0.1   # Minimum weight for mesh extraction (filters out noisy voxels)

# ------------------------- Dataset Class -------------------------
class Dataset:
    def __init__(self, folder: str):
        super().__init__()
        # Get all .pcd files in the folder
        self.scan_files = glob.glob(os.path.join(folder, "*.pcd")) 
        # Initialize poses as identity matrices
        self.poses = np.array([np.eye(4) for _ in range(len(self.scan_files))]) 

    def __getitem__(self, idx):
        if idx >= len(self.scan_files):
            raise IndexError("Index out of range")
        
        # Compute relative pose
        pose = np.linalg.inv(self.poses[0]) @ self.poses[idx]
        # Read point cloud
        points, colors = self.read_pcd(self.scan_files[idx])
        points = np.array(points, dtype=np.float64)
        return points, colors, pose

    def __len__(self):
        return len(self.scan_files)
    
    def read_pcd(self, pcd_file):
        # Read .pcd file using Open3D
        pcd = o3d.io.read_point_cloud(pcd_file)
        # Extract point cloud coordinates
        points = np.asarray(pcd.points)
        # Extract colors (if available)
        if pcd.has_colors():
            colors = np.asarray(pcd.colors)  # Open3D colors are in range [0, 1]
        else:
            colors = np.zeros_like(points)  # If no colors, fill with zeros
        return points, colors

# ------------------------- Main Program -------------------------
if __name__ == '__main__':
    # Initialize VDBVolume
    print("Initializing VDBVolume...")
    vdb_volume = vdbfusion.VDBVolume(voxel_size=VOXEL_SIZE, sdf_trunc=SDF_TRUNC)

    # Load dataset
    print("Loading dataset from", SOURCE_DIR)
    dataset = Dataset(SOURCE_DIR)

    # Integrate all point clouds into the VDBVolume
    print("Integrating point clouds into VDBVolume...")
    for i in range(len(dataset)):
        scan, colors, origin = dataset[i]
        vdb_volume.integrate(scan, origin)

    print("Point cloud integration complete!")

    # Extract triangle mesh
    print("Extracting triangle mesh...")
    vert, tri = vdb_volume.extract_triangle_mesh(min_weight=MIN_WEIGHT)

    # Create Open3D mesh object
    print("Creating Open3D mesh object...")
    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(vert),
        o3d.utility.Vector3iVector(tri),
    )
    
    # Save the mesh
    print("Saving the mesh to output_mesh.ply...")
    o3d.io.write_triangle_mesh("mesh.ply", mesh)
    print("Mesh saved successfully.")

    # ------------------------- Colorize Mesh Vertices -------------------------
    print("Starting mesh colorization...")
    if dataset[0][1] is not None:  # Check if color information exists
        # Combine all point cloud points and colors
        pcd_points = np.vstack([dataset[i][0] for i in range(len(dataset))])
        pcd_colors = np.vstack([dataset[i][1] for i in range(len(dataset))])

        # Use KDTree to find the nearest point for each vertex
        kdtree = KDTree(pcd_points)
        _, indices = kdtree.query(vert)  # Find the nearest point cloud point for each vertex
        vertex_colors = pcd_colors[indices]  # Assign colors

        # Set mesh vertex colors
        mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)

    print("Mesh colorization complete!")

    # Compute vertex normals
    print("Computing vertex normals...")
    mesh.compute_vertex_normals()

    # Save the textured mesh
    print("Saving the textured mesh to textured_mesh.ply...")
    o3d.io.write_triangle_mesh("textured_mesh.ply", mesh)
    print("Textured mesh saved successfully.")

    # Visualize the final colorized mesh
    print("Visualizing the colorized mesh...")
    o3d.visualization.draw_geometries([mesh])
