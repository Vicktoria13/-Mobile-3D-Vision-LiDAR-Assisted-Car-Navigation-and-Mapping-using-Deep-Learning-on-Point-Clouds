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




#0.023
#on applique l'ICP sur les 2 nuages de points divisés
icp_res = o3d.pipelines.registration.registration_icp(divided_pcd1_o3d, divided_pcd2_o3d, 0.035, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint(), 
                                                      o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))

#la translation pour aller de pcd1 à pcd2 donc pour les aligner, il faut appliquer cette translation sur pcd2

#on applique que la translation !!
translate_vector = icp_res.transformation[:3,3]
print("translate_vector: ", translate_vector)

#on applique la translation sur le nuage de points d'origine !! (sur le target)
pcd1.points = o3d.utility.Vector3dVector(np.asarray(pcd1.points) + translate_vector)



#preprocess for pipeline : add one colomn of 0 at the beginning and 3 at the end
#pour avoir la même structure que les pcd .txt

pcd1_array = np.asarray(pcd1.points)
pcd2_array = np.asarray(pcd2.points)

pcd1_array = np.concatenate((np.zeros((pcd1_array.shape[0],1)), pcd1_array, np.zeros((pcd1_array.shape[0],3))), axis=1)
pcd2_array = np.concatenate((np.zeros((pcd2_array.shape[0],1)), pcd2_array, np.zeros((pcd2_array.shape[0],3))), axis=1)


#save the 2 pcd
np.savetxt(root_path + name_pcd1 + "_aligned.txt", pcd1_array, delimiter=" ")
np.savetxt(root_path + name_pcd2 + "_aligned.txt", pcd2_array, delimiter=" ")

print("saved in ", root_path + name_pcd1 + "_aligned.txt")