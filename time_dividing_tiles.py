import numpy as np
import open3d as o3d
import polyscope as ps



""" 
take 2 .txt files, with 4 columns each : x y z timestamp
They represent 2 point clouds that spatially overlap
"""


#load the 2 point clouds
file1 = np.loadtxt("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT.txt")
file2 = np.loadtxt("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_RIGHT.txt")

coords1 = file1[:, 1:4]
coords2 = file2[:, 1:4]

GPS1 = file1[:, 4]
GPS2 = file2[:, 4]


#sort the points by timestamp
sorted_indices1 = np.argsort(GPS1)
sorted_indices2 = np.argsort(GPS2)

sorted_coords1 = coords1[sorted_indices1]
sorted_coords2 = coords2[sorted_indices2]

print("================ PCD1 ================")
print("min des gps time est ",min(GPS1))
print("max des gps time est ",max(GPS1))
print("Range of time is ",max(GPS1) - min(GPS1))

print("================ PCD2 ================")
print("min des gps time est ",min(GPS2))
print("max des gps time est ",max(GPS2))
print("Range of time is ",max(GPS2) - min(GPS2))

