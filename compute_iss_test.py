import open3d as o3d
import time
import numpy as np


def load_ALS_txt_from_AUREL_to_pcd(path):
    # Load the data txt file with , as delimiter
    data = np.loadtxt(path, delimiter=',')

    #garde la colonne 2, 2 et 4 , on enleve la colonne 0 et les colonnes 4 et suivantes
    data = data[:, 2:5]
    
    
    # Create a point cloud
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data)
    pcd.paint_uniform_color([0.5, 0.5, 0.5])

    #downsample if too big
    return pcd



pcd = load_ALS_txt_from_AUREL_to_pcd("/home/sdi-2023-01/Bureau/res_ALS/tiles/chunk_a34.txt")

#compute ISS
print("Compute ISS")
keypoints = o3d.geometry.keypoint.compute_iss_keypoints(pcd,
                                                        salient_radius=.25,
                                                        non_max_radius=.1,
                                                        gamma_21=.65,
                                                        gamma_32=.65,
                                                        min_neighbors=5)

keypoints.paint_uniform_color([1, 0, 0])
print(len(keypoints.points))



#ajoute les keypoints
o3d.visualization.draw_geometries([pcd, keypoints])