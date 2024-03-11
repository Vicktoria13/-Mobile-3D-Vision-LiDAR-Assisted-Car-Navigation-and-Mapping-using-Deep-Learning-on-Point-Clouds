import laspy
import numpy as np
import open3d as o3d

#prends un left un right, merge les 2 et save en txt

Right = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_RIGHT.laz")
coords_right = np.vstack((Right.x, Right.y, Right.z)).transpose()

Left = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT.laz")
coords_left = np.vstack((Left.x, Left.y, Left.z)).transpose()

#merge les 2
coords = np.vstack((coords_right, coords_left))
print("coords.shape: ", coords.shape)

#downsample
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(coords)
pcd_down = pcd.voxel_down_sample(voxel_size=0.1)


#save as a laz file .laz
#on sauvegarde le pcd en .laz

laspy_obj = laspy.create(file_version="1.2", point_format=2)
laspy_obj.x = pcd_down.points[:,0]
laspy_obj.y = pcd_down.points[:,1]
laspy_obj.z = pcd_down.points[:,2]
laspy_obj.intensity = np.zeros(pcd_down.points.shape[0])
laspy_obj.save("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT_RIGHT.laz")

