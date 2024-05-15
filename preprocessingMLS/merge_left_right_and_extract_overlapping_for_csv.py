"""
@Author : Victoria NGUYEN
@Date : 8 avril 2024
ENAC - Projet de semestre : 2024
"""


import os
import argparse
import logging
import numpy as np
import time
import open3d as o3d

FLAG_MERGED = True

"""

Etant donné :

- le dossier contenant tous les .laz
- Les 2 ids des nuages de points à comparer
- Le path de sortie pour le fichier txt

Permet de :

- Si FLAG_MERGED = True :
    - Trouver les nuages de points LEFT et RIGHT correspondant aux ids
    - Les Fusionner

- Si FLAG_MERGED = False :
    - Charger les nuages de points RIGHT correspondant aux ids
++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
+
- Extraire la zone d'overlap entre les 2 nuages fusionnés
- Sauvegarder les points de l'overlap dans un fichier txt


python3 merge_left_right_and_extract_overlapping.py --data_las /media/topostudent/Data1/2024spring_VictoriaZ/01_raw_data/04_CALCULS/230905/LASER/3_OUTPUT_LAZ/ORI --id1 M230905_217560_217615 --id2 M230905_217800_217860 
--out /$HOME/Desktop --visualize true
"""


#/home/sdi-2023-01/Téléchargements/M230905_217440_217459_LEFT.rdbx.csv


def read_csv(path_csv):
    """
    Read the csv file
    """
    data = np.genfromtxt(path_csv, delimiter=',')
    print(data.shape)

    #efface la 1ere ligne
    data = data[1:]

    # xyz ==> 3 premieres colonnes
    xyz = data[:, :3]
    print("nb points: ", len(xyz))

    #æffiche les 10 premiers points
    print(xyz[:10])

    ### vecteur laz = 4 5 6
    laz_vector = data[:, 3:6]

    time_vector = data[:, 6]

    #affiche sur open3d le nuage
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    #downsample
    pcd = pcd.voxel_down_sample(voxel_size=0.1)

    
    #downsample
    print("nombre de points: ", len(pcd.points))
    #colors [0,1,2] ==> [r,g,b]
    #en rouge
    pcd.paint_uniform_color([1, 0, 0])

    o3d.visualization.draw_geometries([pcd])






def merge_left_right_CSV(path_left, path_right):

    """
    Merge the left and right clouds of a pair
    path_left : str : the path to the left cloud .las file
    path_right : str : the path to the right cloud .las file
    """

    #on lit les 2 fichiers csv
    data_left = np.genfromtxt(path_left, delimiter=',')
    data_right = np.genfromtxt(path_right, delimiter=',')

    #efface la 1ere ligne faites de str
    data_left = data_left[1:]
    data_right = data_right[1:]

    # xyz ==> 3 premieres colonnes
    xyz_left = data_left[:, :3]
    xyz_right = data_right[:, :3]

    #vecteur laz = 4 5 6
    laz_vector_left = data_left[:, 3:6]
    laz_vector_right = data_right[:, 3:6]

    time_vector_left = data_left[:, 6]
    time_vector_right = data_right[:, 6]

    ## provenance : 0 = right, 1 = left
    provenance_left = np.ones((xyz_left.shape[0], 1))
    provenance_right = np.zeros((xyz_right.shape[0], 1))


    #on merge les 2 nuages de points concatene
    xyz = np.concatenate((xyz_left, xyz_right), axis=0)
    laz_vector = np.concatenate((laz_vector_left, laz_vector_right), axis=0)
    time_vector = np.concatenate((time_vector_left, time_vector_right), axis=0)
    provenance_vector = np.concatenate((provenance_left, provenance_right), axis=0)
    

    return xyz, laz_vector, time_vector, provenance_vector






def extract_overlapping(merged_array_cloud1, merged_array_cloud2, time_array_cloud1, time_array_cloud2, provenance1, provenance2, laz1, laz2, visualize=False):

    """
    Extract the overlapping area between two clouds
    merged_array_cloud1 : np.array : the merged cloud 1 (left and right) : array of shape (n, 3)
    merged_array_cloud2 : np.array : the merged cloud 2 (left and right) : array of shape (n, 3)
    time_array_cloud1 : np.array : the time array of the merged cloud 1 : array of shape (n, 1)
    time_array_cloud2 : np.array : the time array of the merged cloud 2 : array of shape (n, 1)
    visualize : bool : whether to visualize the overlapping using open3d
    provenance1 : np.array : the provenance of the points of the merged cloud 1 : array of shape (n, 1). It has only 0 (or only 1) if not merged, or 0 and 1 if merged
    laz1 : np.array : the laz vector of the merged cloud 1 : array of shape (n, 3)
    laz2 : np.array : the laz vector of the merged cloud 2 : array of shape (n, 3)
    """

    pcd1_merged = o3d.geometry.PointCloud()
    pcd1_merged.points = o3d.utility.Vector3dVector(merged_array_cloud1)

    pcd2_merged = o3d.geometry.PointCloud()
    pcd2_merged.points = o3d.utility.Vector3dVector(merged_array_cloud2)


    #########  DOWNSAMPLING car on n'a pas besoin d'autant de points pour trouver l'overlap
    pcd1_merged_down = pcd1_merged.voxel_down_sample(voxel_size=1)
    pcd2_merged_down = pcd2_merged.voxel_down_sample(voxel_size=1)

    pcd1_merged_down.paint_uniform_color([1, 0.706, 0])
    pcd2_merged_down.paint_uniform_color([0, 0.651, 0.929])

    if visualize:
        o3d.visualization.draw_geometries([pcd1_merged_down, pcd2_merged_down])

    array_pcd1 = np.asarray(pcd1_merged_down.points)
    array_pcd2= np.asarray(pcd2_merged_down.points)


    ###### BOUNDING BOXES ######
    bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np.concatenate((array_pcd1, array_pcd2), axis=0)))
    bbox.color = [1, 0, 0]
    list_mini_boxes = []  # liste avec des objets bounding box
    size = 10

    for x in np.arange(bbox.min_bound[0], bbox.max_bound[0], size):
        for y in np.arange(bbox.min_bound[1], bbox.max_bound[1], size):
            for z in np.arange(bbox.min_bound[2], bbox.max_bound[2], size):
                mini_bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np.array([[x, y, z], [x+size, y+size, z+size]])))
                mini_bbox.color = [1, 0, 1]
                list_mini_boxes.append(mini_bbox)
    
    
    if visualize:
        o3d.visualization.draw_geometries([pcd1_merged_down, pcd2_merged_down, bbox] + list_mini_boxes)

    
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


        if np.any(bool_has_points_from_cloud1) and np.any(bool_has_points_from_cloud2):
            list_overlap_mini_boxes.append(mini_box_test)
            subset_pcd1 = np.concatenate((subset_pcd1, array_pcd1[bool_has_points_from_cloud1]), axis=0)
            subset_pcd2 = np.concatenate((subset_pcd2, array_pcd2[bool_has_points_from_cloud2]), axis=0)


        #for display ==> loading bar
        cmpt += 1
        print("\r", "Progression : ", cmpt/nb_mini_boxes*100, "%", end="")

    logging.info("--- %s seconds ---" % (time.time() - start_time))
    logging.info(len(list_overlap_mini_boxes)) 

    if visualize:
        o3d.visualization.draw_geometries([pcd1_merged_down, pcd2_merged_down] + list_overlap_mini_boxes)
  


    ############## 
    subset_pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(subset_pcd1))
    subset_pcd1.paint_uniform_color([1, 0.5, 0.5])
    subset_pcd2 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(subset_pcd2))
    subset_pcd2.paint_uniform_color([1, 0.5, 0.5])

    overlap_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate((np.array(subset_pcd1.points), np.array(subset_pcd2.points)), axis=0)))
    bbox_overlap = o3d.geometry.AxisAlignedBoundingBox.create_from_points(overlap_pcd.points)

    if visualize:
        bbox_overlap.color = [0, 1, 0]
        o3d.visualization.draw_geometries([pcd1_merged_down, pcd2_merged_down, bbox_overlap])


    bool_has_points_from_cloud1 = np.logical_and(bbox_overlap.min_bound[0] <= merged_array_cloud1[:, 0], merged_array_cloud1[:, 0] <= bbox_overlap.max_bound[0])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, bbox_overlap.min_bound[1] <= merged_array_cloud1[:, 1])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, merged_array_cloud1[:, 1] <= bbox_overlap.max_bound[1])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, bbox_overlap.min_bound[2] <= merged_array_cloud1[:, 2])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, merged_array_cloud1[:, 2] <= bbox_overlap.max_bound[2])

    bool_has_points_from_cloud2 = np.logical_and(bbox_overlap.min_bound[0] <= merged_array_cloud2[:, 0], merged_array_cloud2[:, 0] <= bbox_overlap.max_bound[0])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, bbox_overlap.min_bound[1] <= merged_array_cloud2[:, 1])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, merged_array_cloud2[:, 1] <= bbox_overlap.max_bound[1])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, bbox_overlap.min_bound[2] <= merged_array_cloud2[:, 2])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, merged_array_cloud2[:, 2] <= bbox_overlap.max_bound[2])

    coords_subset_pcd1 = merged_array_cloud1[bool_has_points_from_cloud1]
    coords_subset_pcd2 = merged_array_cloud2[bool_has_points_from_cloud2]

    subset_pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(coords_subset_pcd1))
    subset_pcd1.paint_uniform_color([0, 1, 1])
    subset_pcd2 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(coords_subset_pcd2))
    subset_pcd2.paint_uniform_color([1, 0,0])

    if visualize:
        o3d.visualization.draw_geometries([subset_pcd1, subset_pcd2])


    print("subset 1 has ", coords_subset_pcd1.shape[0], " points")
    print("subset 2 has ", coords_subset_pcd2.shape[0], " points")

    # time of the points selected
    time_subset_pcd1 = time_array_cloud1[bool_has_points_from_cloud1]
    time_subset_pcd2 = time_array_cloud2[bool_has_points_from_cloud2]


    #provenance
    provenance_subset_pcd1 = provenance1[bool_has_points_from_cloud1]
    provenance_subset_pcd2 = provenance2[bool_has_points_from_cloud2]


    #laz
    laz_subset_pcd1 = laz1[bool_has_points_from_cloud1]
    laz_subset_pcd2 = laz2[bool_has_points_from_cloud2]
    
    return coords_subset_pcd1, coords_subset_pcd2, time_subset_pcd1, time_subset_pcd2, provenance_subset_pcd1, provenance_subset_pcd2, laz_subset_pcd1, laz_subset_pcd2











###########################################

def main():


    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    logger.info(" ====== Merge Left Right for each cloud of the pair + extract overlapping ====== ")

    #get the arguments
    parser = argparse.ArgumentParser(description='Merge Left Right for each cloud of the pair + extract overlapping')
    parser.add_argument('--data_csv', type=str, help='The path to the folder containing the las files with R and L clouds')
    parser.add_argument('--id1', type=str, help='The ID of the first cloud')
    parser.add_argument('--id2', type=str, help='The ID of the second cloud')

    parser.add_argument('--out', type=str, help='The path to the output txt file')
    parser.add_argument('--visualize', type=bool, help='Whether to visualize the overlapping')

    #################### 1. Get the arguments ####################
    args = parser.parse_args()
    path_folder_data_las = args.data_csv
    id1 = args.id1
    id2 = args.id2
    path_out_txt = args.out


    if FLAG_MERGED:
        #################### 2. in the folder 'path_folder_data_las', get the left and right clouds paths
        logger.info("=== Looking for LEFT and RIGHT clouds of + " + id1 + " ...")

        #on itere dans --path_folder_data_las pour trouver les paths RIGHT et LEFT correspondant aux id1 et id2
        path_right_id1 = None
        path_left_id1 = None

        path_right_id2 = None
        path_left_id2 = None

        for root, dirs, files in os.walk(path_folder_data_las):
            for file in files:
                if id1 in file:
                    if "RIGHT" in file:
                        path_right_id1 = os.path.join(root, file)
                    if "LEFT" in file:
                        path_left_id1 = os.path.join(root, file)

                if id2 in file:
                    if "RIGHT" in file:
                        path_right_id2 = os.path.join(root, file)
                    if "LEFT" in file:
                        path_left_id2 = os.path.join(root, file)

        if path_right_id1 is None or path_left_id1 is None or path_right_id2 is None or path_left_id2 is None:
            logger.error("One of the clouds is missing")
            return
        
        logger.info("=== Found the LEFT and RIGHT clouds of " + id1)
        logger.info("=== Found the LEFT and RIGHT clouds of " + id2)

        #################### 3. Merge the left and right clouds of the pair
        # time1 est un array avec les temps des points du nuage de points 1 (LEFT + RIGHT)

        logger.info("=== Merging the LEFT and RIGHT clouds of " + id1)
        merged_cloud_id1, laz1, time1, provenance1 = merge_left_right_CSV(path_left_id1, path_right_id1)
        merged_cloud_id2, laz2, time2, provenance2 = merge_left_right_CSV(path_left_id2, path_right_id2)

    else:

        logger.info("=== Loading only the right, not merging ...")

        for root, dirs, files in os.walk(path_folder_data_las):
            for file in files:
                if id1 in file:
                    if "RIGHT" in file:
                        path_right_id1 = os.path.join(root, file)

                if id2 in file:
                    if "RIGHT" in file:
                        path_right_id2 = os.path.join(root, file)

        logger.info("=== Found the RIGHT clouds of " + id1)
        logger.info("=== Found the RIGHT clouds of " + id2)

        pcd1 = np.genfromtxt(path_right_id1, delimiter=',')
        pcd2 = np.genfromtxt(path_right_id2, delimiter=',')

        pcd1 = pcd1[1:]
        pcd2 = pcd2[1:]

        time1 = pcd1[:, 6]
        time2 = pcd2[:, 6]

        laz1 = pcd1[:, 3:6]
        laz2 = pcd2[:, 3:6]


        merged_cloud_id1 = pcd1[:, :3]
        merged_cloud_id2 = pcd2[:, :3]

        #provenance
        provenance1 = np.zeros((merged_cloud_id1.shape[0], 1))
        provenance2 = np.zeros((merged_cloud_id2.shape[0], 1))

    ################### 4. Extract the overlapping
    overlapped_merged1, overlapped_merged2, time_overlapping_point1, time_overlapping_point2, provenance_subset_pcd1, provenance_subset_pcd2 , laz_subset_pcd1 , laz_subset_pcd2= extract_overlapping(merged_cloud_id1, merged_cloud_id2, time1, time2, provenance1, provenance2, laz1, laz2, visualize=args.visualize)

    ## SAVE THE SUBSET OF POINTS as a txt file : 

    ########
    # t x y z 0 0 0 provenance

    new_coords_pcd1 = np.concatenate((time_overlapping_point1.reshape(-1, 1), overlapped_merged1, laz_subset_pcd1, provenance_subset_pcd1), axis=1)
    new_coords_pcd2 = np.concatenate((time_overlapping_point2.reshape(-1, 1), overlapped_merged2, laz_subset_pcd2, provenance_subset_pcd2), axis=1)
    




    #save pcd1
    path_out_txt_pcd1 = path_out_txt + "/" + id1 + ".txt"
    np.savetxt(path_out_txt_pcd1, new_coords_pcd1, delimiter=" ", fmt="%s")

    #save pcd2
    path_out_txt_pcd2 = path_out_txt + "/" + id2 + ".txt"
    np.savetxt(path_out_txt_pcd2, new_coords_pcd2, delimiter=" ", fmt="%s")

    logger.info("=== Saved the overlapping points to the output txt files")

    logger.info("=== END OF THE SCRIPT ===")
    return 0


if __name__ == "__main__":
    main()
    

