import open3d as o3d
import numpy as np
import laspy
from simpleicp import PointCloud, SimpleICP
import polyscope as ps

FLAG_is_tile = True


PATH1 = "/home/sdi-2023-01/Téléchargements/res_MLS_simple_10/tiles/chunk_a3.txt"
PATH2 = "/home/sdi-2023-01/Téléchargements/res_MLS_simple_10/tiles/chunk_b3.txt"


delim = ","




###################
name_pcd1 = PATH1.split("/")[-1].split(".")[0] #on récupère le nom du fichier
name_pcd2 = PATH2.split("/")[-1].split(".")[0]

#pour avoir /home/sdi-2023-01/Téléchargements/pcd_one_set/
root_path = "/".join(PATH1.split("/")[:-1]) + "/"
print("root_path: ", root_path)
###################





point_pcd1 = (np.loadtxt(PATH1, delimiter=delim))[:,2:5]
point_pcd2 = (np.loadtxt(PATH2, delimiter=delim))[:,2:5]

#on ne garde que les colonnes 2 3 4 pour les tiles
# colonnes 1 2 et 3 pour les pcd .txt




pcd1 = o3d.geometry.PointCloud()
pcd2 = o3d.geometry.PointCloud()

pcd1.points = o3d.utility.Vector3dVector(point_pcd1)
pcd2.points = o3d.utility.Vector3dVector(point_pcd2)




#les nuage est très grand ==> les coordonnées sont très grandes (2e+06)
#pour l'ICP, on shifte les coordonnées autour de l'origine
#the shift is the same for both pcd !!
    
x_shift_both = np.min([np.min(point_pcd1[:,0]), np.min(point_pcd2[:,0])])
y_shift_both = np.min([np.min(point_pcd1[:,1]), np.min(point_pcd2[:,1])])
z_shift_both = np.min([np.min(point_pcd1[:,2]), np.min(point_pcd2[:,2])])


divided_pcd1_array = point_pcd1 - np.array([x_shift_both, y_shift_both, z_shift_both])
divided_pcd2_array = point_pcd2 - np.array([x_shift_both, y_shift_both, z_shift_both])


#ps : #taille des points
ps.init()
ps.register_point_cloud("pcd1", divided_pcd1_array, radius=0.001)
ps.register_point_cloud("pcd2", divided_pcd2_array, radius=0.001)
ps.show()




divided_pcd1_o3d = o3d.geometry.PointCloud()
divided_pcd2_o3d = o3d.geometry.PointCloud()

divided_pcd1_o3d.points = o3d.utility.Vector3dVector(divided_pcd1_array)
divided_pcd2_o3d.points = o3d.utility.Vector3dVector(divided_pcd2_array)


################################################### VERSion SIMPLE ICP
X_fix = np.array(divided_pcd2_o3d.points)
X_moving = np.array(divided_pcd1_o3d.points)

pc_fix = PointCloud(X_fix, columns=["x", "y", "z"])
pc_moving = PointCloud(X_moving, columns=["x", "y", "z"])

icp = SimpleICP()
icp.add_point_clouds(pc_fix, pc_moving)
H, X_mov_transformed, rigid_body_transformation_params, distance_residuals = icp.run(max_overlap_distance=1)

H_matrix = np.array(H)


###################################################################

#on applique l'ICP sur les 2 nuages de points divisés
icp_res = o3d.pipelines.registration.registration_icp(divided_pcd1_o3d, divided_pcd2_o3d, 
                                                      0.05, np.eye(4), 
                                                      o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                      o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500, relative_rmse=0.000001, relative_fitness=1e-6))

#overlap = 90% 

#On applique la transformation resultante aux nuages d'origine : attention il faut "décentrer" la transformation
transformation = icp_res.transformation

############ PS : visualisation de la transformation obtenue via ICP de open3d 
intermediate_pcd1 = o3d.geometry.PointCloud()
intermediate_pcd1.points = o3d.utility.Vector3dVector(np.array(divided_pcd1_o3d.points))
intermediate_pcd1.transform(transformation)

ps.init()
ps.register_point_cloud("pcd1", np.asarray(intermediate_pcd1.points), radius=0.001)
ps.register_point_cloud("pcd2", np.asarray(divided_pcd2_o3d.points), radius=0.001)
ps.show()

################ SAVE
#save the 2 pcd : open3d icp
#on shift les coordonnées
intermediate_pcd1.points = o3d.utility.Vector3dVector(np.array(intermediate_pcd1.points) + np.array([x_shift_both, y_shift_both, z_shift_both]))




divided_pcd1_o3d.transform(H_matrix)
############### PS : visualisation de la transformation obtenue via SIMPLE ICP
ps.init()
ps.register_point_cloud("pcd1", np.asarray(divided_pcd1_o3d.points), radius=0.001)
ps.register_point_cloud("pcd2", np.asarray(divided_pcd2_o3d.points), radius=0.001)
ps.show()
############### PS


#on re-shift les coordonnées
pcd1.points = o3d.utility.Vector3dVector(np.array(divided_pcd1_o3d.points) + np.array([x_shift_both, y_shift_both, z_shift_both]))
pcd2.points = o3d.utility.Vector3dVector(np.array(divided_pcd2_o3d.points) + np.array([x_shift_both, y_shift_both, z_shift_both]))



#preprocess for pipeline : add one colomn of 0 at the beginning and 3 at the end
#pour avoir la même structure que les pcd .txt

pcd1_array = np.asarray(pcd1.points)
pcd2_array = np.asarray(pcd2.points)

pcd1_array = np.concatenate((np.zeros((pcd1_array.shape[0],1)), pcd1_array, np.zeros((pcd1_array.shape[0],3))), axis=1)
pcd2_array = np.concatenate((np.zeros((pcd2_array.shape[0],1)), pcd2_array, np.zeros((pcd2_array.shape[0],3))), axis=1)



print("============= SIMPLE ICP =============")
print("RMS: ", distance_residuals[-1])

print("============= OPEN3D ICP =============")
print("RMS: ", icp_res.inlier_rmse)


#save the 2 pcd : simple icp
np.savetxt(root_path + name_pcd1 + "_aligned_simple_icp.txt", pcd1_array, delimiter=delim)
np.savetxt(root_path + name_pcd2 + "_aligned_simple_icp.txt", pcd2_array, delimiter=delim)


print("saved in ", root_path + name_pcd1 + "_aligned.txt")