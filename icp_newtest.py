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




def center_point_cloud(pcd):
    # Calculer le centre du nuage de points
    center = np.mean(np.asarray(pcd.points), axis=0)
    # Centrer le nuage de points
    pcd_centered = pcd.translate(-center)
    return pcd_centered, center

def align_and_transform(pcd1, pcd2):
    # Centrer les nuages de points
    pcd1_centered, center1 = center_point_cloud(pcd1)
    pcd2_centered, center2 = center_point_cloud(pcd2)
    
    # Recalage des nuages de points centrés
    icp_res = o3d.pipelines.registration.registration_icp(
        pcd2_centered, pcd1_centered, 0.023, np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))

    

    # Obtenir la transformation résultante
    transformation = icp_res.transformation
    transformation = np.array(transformation)
    
    # "Décentrer" la transformation
    transformation[:3, 3] += center1 - transformation[:3, :3] @ center2
    
    # Appliquer la transformation au nuage de points 2 non centré
    aligned_pcd2 = pcd2.transform(transformation)
    
    return aligned_pcd2



#les nuage est très grand ==> les coordonnées sont très grandes (2e+06)

pcd2 = align_and_transform(pcd1, pcd2)






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