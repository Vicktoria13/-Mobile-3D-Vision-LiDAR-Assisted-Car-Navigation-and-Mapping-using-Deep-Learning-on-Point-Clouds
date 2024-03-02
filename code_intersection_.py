import numpy as np
import open3d as o3d
import time
import laspy
import polyscope as ps
import logging as logging
################### FAIT POUR LE LAZ A TESTER


logging.basicConfig(level=logging.INFO)
logging.info("Starting the program ...")

# read the laz file
inFile = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT.laz")
coords = np.vstack((inFile.x, inFile.y, inFile.z)).transpose()

other = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223320_223380_LEFT.laz")
coords2 = np.vstack((other.x, other.y, other.z)).transpose()


#shift
coords2 = coords2 + np.array([0, -40, -18 ])

#downsample the point cloud
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


#load a ply available in the open3d dataset

o3d.visualization.draw_geometries([downpcd, downpcd2])


#on cree une bounding box qui entoure les deux nuages
bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np.concatenate((array_pcd1, array_pcd2), axis=0)))
bbox.color = [1, 0, 0]

list_mini_boxes = [] #liste avec des objets bounding box
size = 8 #10 m

#on va discretiser la bounding box en mini bounding box de 1m de cote
for x in np.arange(bbox.min_bound[0], bbox.max_bound[0], size):
    for y in np.arange(bbox.min_bound[1], bbox.max_bound[1], size):
        for z in np.arange(bbox.min_bound[2], bbox.max_bound[2], size):
            mini_bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np.array([[x, y, z], [x+size, y+size, z+size]])))
            mini_bbox.color = [1, 0, 1]
            list_mini_boxes.append(mini_bbox)



print(len(list_mini_boxes)) #on a 1000 mini bounding box

o3d.visualization.draw_geometries([downpcd, downpcd2, bbox] + list_mini_boxes)


##################### List of mini box which contains points from both clouds
list_overlap_mini_boxes = []
subset_pcd1 = np.zeros((0, 3))
subset_pcd2 = np.zeros((0, 3))


start_time = time.time()
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


print("--- %s seconds ---" % (time.time() - start_time))

print(len(list_overlap_mini_boxes)) #on a 1000 mini bounding box

o3d.visualization.draw_geometries([downpcd, downpcd2, bbox] + list_overlap_mini_boxes)


######### Visualize the subset of points
subset_pcd1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(subset_pcd1))
subset_pcd1.paint_uniform_color([1, 0.5, 0.5])
subset_pcd2 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(subset_pcd2))
subset_pcd2.paint_uniform_color([1, 0.5, 0.5])


print("les points qui s'overlappent sont affiches en rose")
o3d.visualization.draw_geometries([downpcd, downpcd2, subset_pcd1, subset_pcd2])



#idee par la suite : creer overlap_pcd = subset_pcd1 + subset_pcd2, tracer la bounding box de overlap_pcd
# et ensuite garder les points de pc1 et pc2 qui sont dans cette bounding box. De cette manière, on ne perd trop d'information

######################  A TESTER
overlap_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.concatenate((np.array(subset_pcd1.points), np.array(subset_pcd2.points)), axis=0)))
bbox_overlap = o3d.geometry.AxisAlignedBoundingBox.create_from_points(overlap_pcd.points)

#on cree une bounding box qui entoure les deux nuages
bbox_overlap.color = [1, 0, 0]

o3d.visualization.draw_geometries([downpcd, downpcd2, bbox_overlap])


#on garde les points de pc1 et pc2 qui sont dans bbox_overlap : attention , on ne prend pas le downsampled point cloud mais le point cloud original
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

o3d.visualization.draw_geometries([subset_pcd1, subset_pcd2])



#save the 2 point clouds
o3d.io.write_point_cloud("/home/sdi-2023-01/Bureau/epfl/Data_pcd/pcd1.ply", subset_pcd1)