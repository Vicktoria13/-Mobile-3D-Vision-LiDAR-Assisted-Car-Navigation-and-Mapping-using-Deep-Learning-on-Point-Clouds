#%%
import numpy as np
import laspy
import open3d as o3d
import polyscope as ps

#open a laz file and display with polyscope
#%%
# read the laz file
inFile = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT.laz")
coords = np.vstack((inFile.x, inFile.y, inFile.z)).transpose()

other = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223320_223380_LEFT.laz")
coords2 = np.vstack((other.x, other.y, other.z)).transpose()

#downsample the point cloud
downpcd = o3d.geometry.PointCloud()
downpcd.points = o3d.utility.Vector3dVector(coords)
downpcd = downpcd.voxel_down_sample(voxel_size=1)
array = np.array(downpcd.points)

downpcd2 = o3d.geometry.PointCloud()
downpcd2.points = o3d.utility.Vector3dVector(coords2)
downpcd2 = downpcd2.voxel_down_sample(voxel_size=1)
array2 = np.array(downpcd2.points)
#%%

#open 3d
#color
downpcd.paint_uniform_color([0.1, 0.1, 0.7])
downpcd2.paint_uniform_color([0.7, 0.1, 0.1])

#========================================= bounding box
box1 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(downpcd.points)
box2 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(downpcd2.points)

box1.color = (0.1, 0.1, 0.7)
box2.color = (0.7, 0.1, 0.1)


#%%
#o3d.visualization.draw_geometries([downpcd, downpcd2, box1, box2])
o3d.visualization.draw_geometries([downpcd, downpcd2, box1, box2])

