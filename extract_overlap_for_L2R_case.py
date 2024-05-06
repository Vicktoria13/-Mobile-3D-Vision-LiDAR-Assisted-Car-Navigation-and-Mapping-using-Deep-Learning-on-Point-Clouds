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
    downpcd = pcd1.voxel_down_sample(voxel_size=1)
    downpcd2 = pcd2.voxel_down_sample(voxel_size=1)

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
    
    
    if args.visualize:
        o3d.visualization.draw_geometries([downpcd, downpcd2, bbox] + list_mini_boxes)

    
    ###### List of mini box which contains points from both clouds ######
        
    list_overlap_mini_boxes = []
    subset_pcd1 = np.zeros((0, 3))
    subset_pcd2 = np.zeros((0, 3))

    start_time = time.time()

    logging.info("Start of the loop ...")

    nb_mini_boxes = len(list_mini_boxes)
    cmpt  = 0

    for mini_box_test in list_mini_boxes:
    
        bool_has_points_from_cloud1 = np.logical_and(mini_box_test.min_bound[0] <= array_pcd1[:, 0], array_pcd1[:, 0] <= mini_box_test.max_bound[0])
        bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, mini_box_test.min_bound[1] <= array_pcd1[:, 1])
        bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, array_pcd1[:, 1] <= mini_box_test.max_bound[1])
        bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, mini_box_test.min_bound[2] <= array_pcd1[:, 2])
        bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, array_pcd1[:, 2] <= mini_box_test.max_bound[2])

        bool_has_points_from_cloud2 = np.logical_and(mini_box_test.min_bound[0] <= array_pcd2[:, 0], array_pcd2[:, 0] <= mini_box_test.max_bound[0])
        bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, mini_box_test.min_bound[1] <= array_pcd2[:, 1])
        bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, array_pcd2[:, 1] <= mini_box_test.max_bound[1])
        bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, mini_box_test.min_bound[2] <= array_pcd2[:, 2])
        bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, array_pcd2[:, 2] <= mini_box_test.max_bound[2])


        """

        #celui la prend 5 fois plus de temps !!
        bool_has_points_from_cloud1 = np.all(np.logical_and(mini_box_test.min_bound <= array_pcd1, array_pcd1 <= mini_box_test.max_bound), axis=1)
        bool_has_points_from_cloud2 = np.all(np.logical_and(mini_box_test.min_bound <= array_pcd2, array_pcd2 <= mini_box_test.max_bound), axis=1)

        """

        if np.any(bool_has_points_from_cloud1) and np.any(bool_has_points_from_cloud2):
            list_overlap_mini_boxes.append(mini_box_test)
            subset_pcd1 = np.concatenate((subset_pcd1, array_pcd1[bool_has_points_from_cloud1]), axis=0)
            subset_pcd2 = np.concatenate((subset_pcd2, array_pcd2[bool_has_points_from_cloud2]), axis=0)


        #for display ==> loading bar
        cmpt += 1
        print("\r", "Progression : ", cmpt/nb_mini_boxes*100, "%", end="")

    logging.info("--- %s seconds ---" % (time.time() - start_time))
    logging.info(len(list_overlap_mini_boxes)) 

    if args.visualize:
        o3d.visualization.draw_geometries([downpcd, downpcd2] + list_overlap_mini_boxes)

    

    ############## 
    subset_pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(subset_pcd1))
    subset_pcd1.paint_uniform_color([1, 0.5, 0.5])
    subset_pcd2 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(subset_pcd2))
    subset_pcd2.paint_uniform_color([1, 0.5, 0.5])

    coords_subset_pcd1 = coords[bool_has_points_from_cloud1]
    coords_subset_pcd2 = coords2[bool_has_points_from_cloud2]

    subset_pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(coords_subset_pcd1))
    # jaune
    subset_pcd1.paint_uniform_color([0, 1, 1])
    subset_pcd2 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(coords_subset_pcd2))
    subset_pcd2.paint_uniform_color([1, 0,0])

    if args.visualize:
        o3d.visualization.draw_geometries([subset_pcd1, subset_pcd2])


    print("subset 1 has ", coords_subset_pcd1.shape[0], " points")
    print("subset 2 has ", coords_subset_pcd2.shape[0], " points")

    ## SAVE THE SUBSET OF POINTS as a txt file : 
    # pour s'adapter facilement : rajout de 1 colonne de 0 au debut et 3 colonnes de 0 a la fin
    
    new_coords_pcd1 = np.hstack((np.zeros((coords_subset_pcd1.shape[0],1)), coords_subset_pcd1))
    new_coords_pcd1 = np.hstack((new_coords_pcd1, np.zeros((coords_subset_pcd1.shape[0],3))))
    np.savetxt(args.output + "/pcd1.txt", new_coords_pcd1, delimiter=" ", fmt="%s")

    new_coords_pcd2 = np.hstack((np.zeros((coords_subset_pcd2.shape[0],1)), coords_subset_pcd2))
    new_coords_pcd2 = np.hstack((new_coords_pcd2, np.zeros((coords_subset_pcd2.shape[0],3))))
    np.savetxt(args.output + "/pcd2.txt", new_coords_pcd2, delimiter=" ", fmt="%s")
    
    logging.info("End of the program ...")
    
    
    return 0


if __name__ == "__main__":
    main()