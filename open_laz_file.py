import numpy as np
import laspy
import open3d as o3d
import polyscope as ps

#open a laz file and display with polyscope

# read the laz file
inFile = laspy.read("/home/sdi-2023-01/Bureau/epfl/LAB/voiture_autonome/85268 - M230905_223260_223320_LEFT.laz")


# get the x, y, z coordinates
coords = np.vstack((inFile.x, inFile.y, inFile.z)).transpose()

# create a point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(coords)

#downsample the point cloud
pcd = pcd.voxel_down_sample(voxel_size=1)
print("il y a ", len(pcd.points), "points dans le nuage de points")
#couleur
pcd.paint_uniform_color([0.1, 0.1, 0.7])

o3d.visualization.draw_geometries([pcd])

