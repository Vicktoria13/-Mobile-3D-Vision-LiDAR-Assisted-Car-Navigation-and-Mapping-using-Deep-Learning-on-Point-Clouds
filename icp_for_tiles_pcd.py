import open3d as o3d
import numpy as np
import laspy


#on load les nuages de points sous forme 
inFile = laspy.read("")
coords = np.vstack((inFile.x, inFile.y, inFile.z)).transpose()

other = laspy.read("")
coords2 = np.vstack((other.x, other.y, other.z)).transpose()

pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(coords)

pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(coords2)


o3d.visualization.draw_geometries([pcd1, pcd2])


#on applique l'ICP
icp_res = o3d.pipelines.registration.registration_icp(
    pcd1, pcd2, 0.1, np.eye(4),
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200))



#on applique que la translation !!
translate_vector = icp_res.transformation[:3,3]
print(translate_vector)

#on applique cet offset à tout les points du nuage de points
pcd2.points = o3d.utility.Vector3dVector(np.array(pcd2.points) + translate_vector)

#on applique juste la translation
o3d.visualization.draw_geometries([pcd1, pcd2])

