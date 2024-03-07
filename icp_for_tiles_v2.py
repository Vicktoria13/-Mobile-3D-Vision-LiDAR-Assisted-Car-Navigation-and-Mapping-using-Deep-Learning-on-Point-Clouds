import open3d as o3d
import numpy as np
import laspy



PATH1 = "/home/sdi-2023-01/Téléchargements/chunks/chunk_a3.txt"
PATH2 = "/home/sdi-2023-01/Téléchargements/chunks/chunk_b3.txt"
delim = ","
down_flag = False


print("Open3D version: " + o3d.__version__)
print("PATH1: " + PATH1)
print("PATH2: " + PATH2)


#on load les nuages de points sous forme txt

point_pcd1 = np.loadtxt(PATH1, delimiter=delim)
point_pcd2 = np.loadtxt(PATH2, delimiter=delim)

#on ne garde que les colonnes 2 3 4 pour les tiles
# colonnes 1 2 et 3 pour les pcd .txt
point_pcd1 = point_pcd1[:,2:5]
point_pcd2 = point_pcd2[:,2:5]

pcd1 = o3d.geometry.PointCloud()
pcd2 = o3d.geometry.PointCloud()

pcd1.points = o3d.utility.Vector3dVector(point_pcd1)
pcd2.points = o3d.utility.Vector3dVector(point_pcd2)

#les nuage est très grand ==> les coordonnées sont très grandes (2e+06)
#pour l'ICP, on shifte les coordonnées autour de l'origine

divided_pcd1_array = np.asarray(pcd1.points)/500000
divided_pcd2_array = np.asarray(pcd2.points)/500000
divided_pcd1_o3d = o3d.geometry.PointCloud()
divided_pcd2_o3d = o3d.geometry.PointCloud()

divided_pcd1_o3d.points = o3d.utility.Vector3dVector(divided_pcd1_array)
divided_pcd2_o3d.points = o3d.utility.Vector3dVector(divided_pcd2_array)




#on applique l'ICP sur les 2 nuages de points divisés
icp_res = o3d.pipelines.registration.registration_icp(divided_pcd1_o3d, divided_pcd2_o3d, 0.1, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint(), 
                                                      o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))

#la translation pour aller de pcd1 à pcd2 donc pour les aligner, il faut appliquer cette translation sur pcd2

#on applique que la translation !!
translate_vector = icp_res.transformation[:3,3]
print("translate_vector: ", translate_vector)

#on applique la translation sur le nuage de points d'origine !! (sur le target)
pcd2.points = o3d.utility.Vector3dVector(np.asarray(pcd2.points) + translate_vector)

#save the 2 pcd
np.savetxt("/home/sdi-2023-01/Téléchargements/chunks/chunk_a3_icp.txt", np.asarray(pcd1.points), delimiter=" ")
np.savetxt("/home/sdi-2023-01/Téléchargements/chunks/chunk_b3_icp.txt", np.asarray(pcd2.points), delimiter=" ")