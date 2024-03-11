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

coords = np.asarray(pcd_down.points)
print("coords.shape: ", coords.shape)



#save en txt
np.savetxt("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT_RIGHT.txt", coords, delimiter=" ", fmt="%s")