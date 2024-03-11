import open3d as o3d
import numpy as np
import laspy



PATH1 = "/home/sdi-2023-01/Téléchargements/pcd_one_set/pcd1.txt"
PATH2 = "/home/sdi-2023-01/Téléchargements/pcd_one_set/pcd2.txt"
delim = " "
down_flag = False


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
    
    # "Décentrer" la transformation
    transformation[:3, 3] += center1 - transformation[:3, :3] @ center2
    
    # Appliquer la transformation au nuage de points 2 non centré
    aligned_pcd2 = pcd2.transform(transformation)
    
    return aligned_pcd2




#on load les nuages de points sous forme txt

point_pcd1 = np.loadtxt(PATH1, delimiter=delim)
point_pcd2 = np.loadtxt(PATH2, delimiter=delim)

#on ne garde que les colonnes 2 3 4 pour les chunks
# colonnes 1 2 et 3 pour les pcd .txt
point_pcd1 = point_pcd1[:,1:4]
point_pcd2 = point_pcd2[:,1:4]

pcd1 = o3d.geometry.PointCloud()
pcd2 = o3d.geometry.PointCloud()

pcd1.points = o3d.utility.Vector3dVector(point_pcd1)
pcd2.points = o3d.utility.Vector3dVector(point_pcd2)

pcd1.paint_uniform_color([1, 0.706, 0])
pcd2.paint_uniform_color([0, 0.651, 0.929])


print("max pcd1: ", np.max(point_pcd1))
print("max pcd2: ", np.max(point_pcd2))
o3d.visualization.draw_geometries([pcd1, pcd2])

#les nuage est très grand ==> les coordonnées sont très grandes (2e+06)
#pour l'ICP, on shifte les coordonnées autour de l'origine : pour shift, on calcule le centre de gravité
#sur chaque axe, on soustrait la valeur du centre de gravité

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



divided_pcd1_o3d.paint_uniform_color([1, 0.706, 0])
divided_pcd2_o3d.paint_uniform_color([0, 0.651, 0.929])

o3d.visualization.draw_geometries([divided_pcd1_o3d, divided_pcd2_o3d])

#on applique l'ICP sur les 2 nuages de points divisés
icp_res = o3d.pipelines.registration.registration_icp(divided_pcd1_o3d, divided_pcd2_o3d, 0.023, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint(), 
                                                      o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))



#il faut decentrer la transformation
matrix = icp_res.transformation
matrix[0,3] += x_mean_pcd1
matrix[1,3] += y_mean_pcd1
matrix[2,3] += z_mean_pcd1



#save the 2 pcd
np.savetxt(root_path + name_pcd1 + "_aligned.txt", np.asarray(pcd1.points), delimiter=" ")
np.savetxt(root_path + name_pcd2 + "_aligned.txt", np.asarray(pcd2.points), delimiter=" ")

print("saved in ", root_path + name_pcd1 + "_aligned.txt")