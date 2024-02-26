import numpy as np
import laspy
import open3d as o3d
import polyscope as ps

#open a laz file and display with polyscope

# read the laz file
inFile = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT.laz")
coords = np.vstack((inFile.x, inFile.y, inFile.z)).transpose()

other = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223320_223380_LEFT.laz")
coords2 = np.vstack((other.x, other.y, other.z)).transpose()

#downsample the point cloud
downpcd = o3d.geometry.PointCloud()
downpcd.points = o3d.utility.Vector3dVector(coords)
downpcd = downpcd.voxel_down_sample(voxel_size=0.1)
array = np.array(downpcd.points)

downpcd2 = o3d.geometry.PointCloud()
downpcd2.points = o3d.utility.Vector3dVector(coords2)
downpcd2 = downpcd2.voxel_down_sample(voxel_size=0.1)
array2 = np.array(downpcd2.points)


#bounding box
box1 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(downpcd.points)
box2 = o3d.geometry.AxisAlignedBoundingBox.create_from_points(downpcd2.points)


dims1 = box1.get_max_bound() - box1.get_min_bound()
bound_low1 = box1.get_min_bound()
bound_high1 = box1.get_max_bound()

dims2 = box2.get_max_bound() - box2.get_min_bound()
bound_low2 = box2.get_min_bound()
bound_high2 = box2.get_max_bound()




#polyscope
ps.init()
ps_box1 = ps.register_volume_grid("box1", bound_low1, dims1, [0.1, 0.1, 0.7])
ps_box2 = ps.register_volume_grid("box2", bound_low2, dims2, [0.7, 0.1, 0.1])

#taille des points  sphère de rayon 0.01
ps.register_point_cloud("points", array, radius=0.0006, color=[0.1, 0.1, 0.7])
ps.register_point_cloud("points2", array2, radius=0.0006, color=[0.7, 0.1, 0.1])



ps.show()
