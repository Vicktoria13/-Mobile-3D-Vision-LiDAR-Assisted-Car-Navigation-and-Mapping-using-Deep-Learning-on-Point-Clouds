import numpy as np
import open3d as o3d
import time
import laspy
import argparse
#for log info
import logging as logging


#to call
#python extract_overlapping.py /home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT.laz /home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223320_223380_LEFT.laz 8 True



def main():

    #ßet level of log info
    logging.basicConfig(level=logging.INFO)
    logging.info("Starting the program ...")

    # =============== ARGUMENTS ===============
    parser = argparse.ArgumentParser(description='Overlapping Extraction Automation')
    parser.add_argument('--path1', type=str, help='Path to the first .laz file')
    parser.add_argument('--path2', type=str, help='Path to the second .laz file')
    parser.add_argument('--output', type=str, help='Path to the output .txt file')
    parser.add_argument('--size', type=int, help='Size of the mini bounding box')
    parser.add_argument('--visualize', type=bool, help='Visualize the overlapping area')

    args = parser.parse_args()
    # =========================================

    # read the laz file
    inFile = laspy.read(args.path1)
    coords = np.vstack((inFile.x, inFile.y, inFile.z)).transpose()

    other = laspy.read(args.path2)
    coords2 = np.vstack((other.x, other.y, other.z)).transpose()

    ############ FOR HUGE PCD ==> downsample the point cloud
    downpcd = o3d.geometry.PointCloud()
    downpcd.points = o3d.utility.Vector3dVector(coords)
    downpcd = downpcd.voxel_down_sample(voxel_size=1)
    array_pcd1 = np.array(downpcd.points)

    downpcd2 = o3d.geometry.PointCloud()
    downpcd2.points = o3d.utility.Vector3dVector(coords2)
    downpcd2 = downpcd2.voxel_down_sample(voxel_size=1)
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


    logging.info("--- %s seconds ---" % (time.time() - start_time))
    logging.info(len(list_overlap_mini_boxes)) 

    if args.visualize:
        o3d.visualization.draw_geometries([downpcd, downpcd2] + list_overlap_mini_boxes)

    

    ############## 
    subset_pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(subset_pcd1))
    subset_pcd1.paint_uniform_color([1, 0.5, 0.5])
    subset_pcd2 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(subset_pcd2))
    subset_pcd2.paint_uniform_color([1, 0.5, 0.5])

    overlap_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate((np.array(subset_pcd1.points), np.array(subset_pcd2.points)), axis=0)))
    bbox_overlap = o3d.geometry.AxisAlignedBoundingBox.create_from_points(overlap_pcd.points)

    if args.visualize:
        bbox_overlap.color = [0, 1, 0]
        o3d.visualization.draw_geometries([downpcd, downpcd2, bbox_overlap])


    bool_has_points_from_cloud1 = np.logical_and(bbox_overlap.min_bound[0] <= coords[:, 0], coords[:, 0] <= bbox_overlap.max_bound[0])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, bbox_overlap.min_bound[1] <= coords[:, 1])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, coords[:, 1] <= bbox_overlap.max_bound[1])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, bbox_overlap.min_bound[2] <= coords[:, 2])
    bool_has_points_from_cloud1 = np.logical_and(bool_has_points_from_cloud1, coords[:, 2] <= bbox_overlap.max_bound[2])

    bool_has_points_from_cloud2 = np.logical_and(bbox_overlap.min_bound[0] <= coords2[:, 0], coords2[:, 0] <= bbox_overlap.max_bound[0])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, bbox_overlap.min_bound[1] <= coords2[:, 1])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, coords2[:, 1] <= bbox_overlap.max_bound[1])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, bbox_overlap.min_bound[2] <= coords2[:, 2])
    bool_has_points_from_cloud2 = np.logical_and(bool_has_points_from_cloud2, coords2[:, 2] <= bbox_overlap.max_bound[2])

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