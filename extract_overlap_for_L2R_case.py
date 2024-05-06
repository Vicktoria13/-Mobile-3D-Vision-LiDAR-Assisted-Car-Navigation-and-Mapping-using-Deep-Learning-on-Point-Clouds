"""
@Author: Victoria Nguyen
@Date: 2024 spring
"""

import numpy as np
import open3d as o3d
import time
import laspy
import argparse
#for log info
import logging as logging


#same que l'autre, mais avec des txt a 3 colonnes x y z
"""
python3 extract_overlapping.py --path1 "/media/topostudent/Data1/2024spring_VictoriaZ/01_raw_data/04_CALCULS/230905/two_useful_scans/third_set/85268 - M230905_217440_217459_RIGHT.laz" --path2 "/media/topostudent/Data1/2024spring_VictoriaZ/01_raw_data/04_CALCULS/230905/two_useful_scans/third_set/85268 - M230905_218400_218437_RIGHT.laz" --output "/media/topostudent/Data1/2024spring_VictoriaZ/01_raw_data/04_CALCULS/230905/two_useful_scans/third_set" --size 8
python3 extract_overlapping.py --path1 "/media/topostudent/Data1/2024spring_VictoriaZ/01_raw_data/04_CALCULS/230905/two_useful_scans/second_set/85268 - M230905_210540_210600_RIGHT.laz" --path2 "/media/topostudent/Data1/2024spring_VictoriaZ/01_raw_data/04_CALCULS/230905/two_useful_scans/second_set/85268 - M230905_215661_215700_RIGHT.laz" --output "/media/topostudent/Data1/2024spring_VictoriaZ/01_raw_data/04_CALCULS/230905/two_useful_scans/second_set" --size 8
"""


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





def main():

    #ßet level of log info
    logging.basicConfig(level=logging.INFO)
    logging.info("Starting the program ...")

    # =============== ARGUMENTS ===============
    parser = argparse.ArgumentParser(description='Overlapping Extraction Automation')
    parser.add_argument('--path1', type=str, help='Path to the first .txt file')
    parser.add_argument('--path2', type=str, help='Path to the second .txt file')
    parser.add_argument('--output', type=str, help='Path to the output .txt file')
    parser.add_argument('--size', type=int, help='Size of the mini bounding box')
    parser.add_argument('--visualize', type=bool, help='Visualize the overlapping area')

    args = parser.parse_args()
    # =========================================

    # read the txt file
    coords = np.loadtxt(args.path1)
    coords2 =   np.loadtxt(args.path2)

    #coords is [t x y z 0 0 0]
    #onlu keep the 3 xyz columns
    coords = coords[:, 1:4]
    coords2 = coords2[:, 1:4]


    print("coords1 shape : ", coords.shape)
    print("coords2 shape : ", coords2.shape)

    ############ LOAD THE POINT CLOUDS ############

    pcd1 = o3d.geometry.PointCloud()
    pcd1.points = o3d.utility.Vector3dVector(coords)

    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(coords2)

    if args.visualize:
        o3d.visualization.draw_geometries([pcd1, pcd2])


    #########  DOWNSAMPLING car on n'a pas besoin d'autant de points pour trouver l'overlap
    downpcd = pcd1.voxel_down_sample(voxel_size=0.01)
    downpcd2 = pcd2.voxel_down_sample(voxel_size=0.01)

    array_pcd1 = np.array(downpcd.points)
    array_pcd2 = np.array(downpcd2.points)

    downpcd.paint_uniform_color([0.1, 0.1, 0.7])
    downpcd2.paint_uniform_color([0.7, 0.1, 0.1])

    if args.visualize:
        o3d.visualization.draw_geometries([downpcd, downpcd2])

    ###### BOUNDING BOXES ######
    bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np.concatenate((array_pcd1, array_pcd2), axis=0)))
    bbox.color = [1, 0, 0]
    list_mini_boxes = []  # liste avec des objets bounding box
    size = args.size  # 10 m


    for x in np.arange(bbox.min_bound[0], bbox.max_bound[0], size):
        for y in np.arange(bbox.min_bound[1], bbox.max_bound[1], size):
            for z in np.arange(bbox.min_bound[2], bbox.max_bound[2], size):
                mini_bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np.array([[x, y, z], [x+size, y+size, z+size]])))
                mini_bbox.color = [1, 0, 1]
                list_mini_boxes.append(mini_bbox)
    

    
    #fusionner toutes les mini box en une seule pour obtenir un polygone
    #on peut utiliser la fonction create_from_axis_aligned_bounding_box
                
    poly = o3d.geometry.TriangleMesh.create_from_axis_aligned_bounding_box(bbox)
    poly.paint_uniform_color([0.1, 0.1, 0.1])

    if args.visualize:
        o3d.visualization.draw_geometries([downpcd, downpcd2, poly])


    #on regarde les points de chaque nuages qui sont dans ce polygone poly
    #on garde les points de chaque nuage qui sont dans le polygone poly
    bool_has_points_from_cloud1 = np.logical_and(poly.vertices[:, 0] <= array_pcd1[:, 0], array_pcd1[:, 0] <= poly.vertices[:, 1])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, poly.vertices[:, 1] <= array_pcd1[:, 1])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, array_pcd1[:, 1] <= poly.vertices[:, 2])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, poly.vertices[:, 2] <= array_pcd1[:, 2])


    bool_has_points_from_cloud2 = np.logical_and(poly.vertices[:, 0] <= array_pcd2[:, 0], array_pcd2[:, 0] <= poly.vertices[:, 1])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, poly.vertices[:, 1] <= array_pcd2[:, 1])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, array_pcd2[:, 1] <= poly.vertices[:, 2])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, poly.vertices[:, 2] <= array_pcd2[:, 2])

    #on garde les points qui sont dans le polygone
    points_cloud1 = coords[bool_has_points_from_cloud1]
    points_cloud2 = coords2[bool_has_points_from_cloud2]

    #on cree un nuage de points avec ces points
    pcd1_in_poly = o3d.geometry.PointCloud()
    pcd1_in_poly.points = o3d.utility.Vector3dVector(points_cloud1)

    pcd2_in_poly = o3d.geometry.PointCloud()
    pcd2_in_poly.points = o3d.utility.Vector3dVector(points_cloud2)

    if args.visualize:
        o3d.visualization.draw_geometries([pcd1_in_poly, pcd2_in_poly])

        


   
    
    return 0


if __name__ == "__main__":
    main()