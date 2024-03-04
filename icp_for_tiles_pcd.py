import open3d as o3d
import numpy as np



cfg_icp_preprocess = {
    'icp_thresh': 0.2,
    'icp_max_n': 50,
    'icp_conv': 1e-4,
    'voxel_size': 1
}

def run_icp(target, ref, cfg): 

    """
    target ==> object PointCloud
    ref ==> object PointCloud
    cfg ==> dictionnary with the following keys : 
        - icp_thresh : float
        - icp_max_n : int
        - icp_conv : float
        - voxel_size : float

    return : object ICPConvergenceCriteria
    """

    icp = o3d.pipelines.registration.registration_icp(
        target, ref, cfg['icp_thresh'],
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=cfg['icp_max_n'], relative_fitness=cfg['icp_conv'], relative_rmse=cfg['icp_conv']),
        )
        
    return icp



#on load les nuages de points sous forme txt

point_pcd1 = np.loadtxt("C:/Users/jeanbaptiste/Desktop/Projet de semestre ICP/point_pcd1.txt")
point_pcd2 = np.loadtxt("C:/Users/jeanbaptiste/Desktop/Projet de semestre ICP/point_pcd2.txt")

pcd1 = o3d.geometry.PointCloud()
pcd2 = o3d.geometry.PointCloud()

pcd1.points = o3d.utility.Vector3dVector(point_pcd1)
pcd2.points = o3d.utility.Vector3dVector(point_pcd2)


o3d.visualization.draw_geometries([pcd1, pcd2])


#on applique l'ICP
icp_res = run_icp(pcd1, pcd2, cfg_icp_preprocess)

print("translation : ", icp_res.transformation[:3,3])

#on applique juste la translation
pcd2.points = pcd2.points + icp_res.transformation[:3,3]

o3d.visualization.draw_geometries([pcd1, pcd2])

