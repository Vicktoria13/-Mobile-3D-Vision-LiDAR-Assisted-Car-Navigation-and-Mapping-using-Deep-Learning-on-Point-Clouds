import open3d as o3d
import time
import numpy as np

import open3d as o3d
import numpy as np
import laspy


FLAG_is_tile = True
FLAG_Downsample = False


PATH1 = "/home/sdi-2023-01/Téléchargements/chunks_again_yes/chunk_a4.txt"
PATH2 = "/home/sdi-2023-01/Téléchargements/chunks_again_yes/chunk_b4.txt"


if FLAG_is_tile:
    delim = ","
else:
    delim = " "



###################
name_pcd1 = PATH1.split("/")[-1].split(".")[0] #on récupère le nom du fichier
name_pcd2 = PATH2.split("/")[-1].split(".")[0]

#pour avoir /home/sdi-2023-01/Téléchargements/pcd_one_set/
root_path = "/".join(PATH1.split("/")[:-1]) + "/"
print("root_path: ", root_path)
###################



print("Open3D version: " + o3d.__version__)
print("PATH1: " + PATH1)
print("PATH2: " + PATH2)


#on load les nuages de points sous forme txt

point_pcd1 = np.loadtxt(PATH1, delimiter=delim)
point_pcd2 = np.loadtxt(PATH2, delimiter=delim)

#on ne garde que les colonnes 2 3 4 pour les tiles
# colonnes 1 2 et 3 pour les pcd .txt

if FLAG_is_tile:
    point_pcd1 = point_pcd1[:,2:5]
    point_pcd2 = point_pcd2[:,2:5]
else:
    point_pcd1 = point_pcd1[:,1:4]
    point_pcd2 = point_pcd2[:,1:4]



pcd1 = o3d.geometry.PointCloud()
pcd2 = o3d.geometry.PointCloud()

pcd1.points = o3d.utility.Vector3dVector(point_pcd1)
pcd2.points = o3d.utility.Vector3dVector(point_pcd2)


if FLAG_Downsample:
    #on downsample les nuages de points
    print("BEFORE DOWNSAMPLE:  pcd1 has ", len(pcd1.points), " points and pcd2 has ", len(pcd2.points), " points")

    
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.05)
    pcd2 = pcd2.voxel_down_sample(voxel_size=0.05)

    print("AFTER DOWNSAMPLE:  pcd1 has ", len(pcd1.points), " points and pcd2 has ", len(pcd2.points), " points")




#les nuage est très grand ==> les coordonnées sont très grandes (2e+06)
#pour l'ICP, on shifte les coordonnées autour de l'origine
x_mean_pcd1 = np.mean(point_pcd1[:,0])
y_mean_pcd1 = np.mean(point_pcd1[:,1])
z_mean_pcd1 = np.mean(point_pcd1[:,2])
divided_pcd1_array = point_pcd1 - [x_mean_pcd1, y_mean_pcd1, z_mean_pcd1]

x_mean_pcd2 = np.mean(point_pcd2[:,0])
y_mean_pcd2 = np.mean(point_pcd2[:,1])
z_mean_pcd2 = np.mean(point_pcd2[:,2])
divided_pcd2_array = point_pcd2 - [x_mean_pcd2, y_mean_pcd2, z_mean_pcd2]


print("max pcd1: ", np.max(divided_pcd1_array))
print("max pcd2: ", np.max(divided_pcd2_array))

divided_pcd1_o3d = o3d.geometry.PointCloud()
divided_pcd2_o3d = o3d.geometry.PointCloud()

divided_pcd1_o3d.points = o3d.utility.Vector3dVector(divided_pcd1_array)
divided_pcd2_o3d.points = o3d.utility.Vector3dVector(divided_pcd2_array)




#on applique multi scale icp

voxel_sizes = o3d.utility.DoubleVector([0.1, 0.05, 0.025])

# List of Convergence-Criteria for Multi-Scale ICP:
criteria_list = [
    o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=0.0001,
                                relative_rmse=0.0001,
                                max_iteration=20),
    o3d.pipelines.registration.ICPConvergenceCriteria(0.00001, 0.00001, 15),
    o3d.pipelines.registration.ICPConvergenceCriteria(0.000001, 0.000001, 10)
]

# `max_correspondence_distances` for Multi-Scale ICP (o3d.utility.DoubleVector):
max_correspondence_distances = o3d.utility.DoubleVector([0.3, 0.14, 0.07])

# Initial alignment or source to target transform.
init_source_to_target = o3d.core.Tensor.eye(4, o3d.core.Dtype.Float32)

# Select the `Estimation Method`, and `Robust Kernel` (for outlier-rejection).
estimation =  o3d.pipelines.registration.TransformationEstimationPointToPlane()

# Save iteration wise `fitness`, `inlier_rmse`, etc. to analyse and tune result.
save_loss_log = True

# Setting Verbosity to Debug, helps in fine-tuning the performance.
# o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)

s = time.time()



ms_icp_time = time.time() - s
print("Time taken by Multi-Scale ICP: ", ms_icp_time)
print("Inlier Fitness: ", registration_ms_icp.fitness)
print("Inlier RMSE: ", registration_ms_icp.inlier_rmse)


print("Transformation Matrix: ") 
print(registration_ms_icp.transformation)