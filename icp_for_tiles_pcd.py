import open3d as o3d
import numpy as np
import laspy




#on load les nuages de points sous forme txt

point_pcd1 = np.loadtxt("C:/Users/jeanbaptiste/Desktop/Projet de semestre ICP/point_pcd1.txt", delimiter=",")
point_pcd2 = np.loadtxt("C:/Users/jeanbaptiste/Desktop/Projet de semestre ICP/point_pcd2.txt", delimiter=",")

#on ne garde que les colonnes 1 2 3
point_pcd1 = point_pcd1[:,1:4]
point_pcd2 = point_pcd2[:,1:4]



pcd1 = o3d.geometry.PointCloud()
pcd2 = o3d.geometry.PointCloud()

pcd1.points = o3d.utility.Vector3dVector(point_pcd1)
pcd2.points = o3d.utility.Vector3dVector(point_pcd2)

pcd1.paint_uniform_color([1, 0.706, 0])
pcd2.paint_uniform_color([0, 0.651, 0.929])

print("premier nuage a ", len(pcd1.points), " points et le deuxieme nuage a ", len(pcd2.points), " points")
o3d.visualization.draw_geometries([pcd1, pcd2])



#downsampling
pcd1 = pcd1.voxel_down_sample(voxel_size=0.02)
pcd2 = pcd2.voxel_down_sample(voxel_size=0.02)

print("premier nuage a ", len(pcd1.points), " points et le deuxieme nuage a ", len(pcd2.points), " points")
print("downsampling")

#on applique l'ICP
icp_res = o3d.pipelines.registration.registration_icp(pcd1, pcd2, 0.02, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPoint(), o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))



#on applique que la translation !!
translate_vector = icp_res.transformation[:3,3]
print(translate_vector)

#on applique cet offset à tout les points du nuage de points
pcd2.points = o3d.utility.Vector3dVector(np.array(pcd2.points) + translate_vector)

#on applique juste la translation
o3d.visualization.draw_geometries([pcd1, pcd2])

