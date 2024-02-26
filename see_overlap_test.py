import numpy as np
import laspy
import open3d as o3d
import polyscope as ps
import tiler

#open a laz file and display with polyscope

# read the laz file
pcd_txt = np.loadtxt("/home/sdi-2023-01/Téléchargements/ALS_line200.txt")
coords = pcd_txt[:, 0:3]

pcd_txt2 = np.loadtxt("/home/sdi-2023-01/Téléchargements/ALS_line300.txt")
coords2 = pcd_txt2[:, 0:3]

#downsample the point cloud
downpcd = o3d.geometry.PointCloud()
downpcd.points = o3d.utility.Vector3dVector(coords)
downpcd = downpcd.voxel_down_sample(voxel_size=5)
array = np.array(downpcd.points)

downpcd2 = o3d.geometry.PointCloud()
downpcd2.points = o3d.utility.Vector3dVector(coords2)
downpcd2 = downpcd2.voxel_down_sample(voxel_size=5)
array2 = np.array(downpcd2.points)


print("nb points 1 : ", len(array))
print("nb points 2 : ", len(array2))


ROI = tiler.ROI("/home/sdi-2023-01/Téléchargements/ALS_line200.txt", "/home/sdi-2023-01/Téléchargements/ALS_line300.txt")

