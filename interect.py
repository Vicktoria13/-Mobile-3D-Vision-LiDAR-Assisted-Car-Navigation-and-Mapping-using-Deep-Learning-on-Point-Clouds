import numpy as np
import open3d as o3d


"""
class MiniBox():
    def __init__(self, x, y, z, size):

        #min_ bound = point en bas a gauche
        #max_bound = point en haut a droite

        self.min_bound = np.array([x, y, z])
        self.max_bound = np.array([x+size, y+size, z+size])
        self.color = [1, 0, 1]
        self.size = size
        selfbool_has_points_from_cloud1 = False
        self.bool_has_points_from_cloud2 = False

    def contains_points(self, points):
        #etant donné un pcd, on regarde si il y a des points dans la bounding box
        #si oui, on met le bool a True

        #pas de loop, utilise les indices
        #on regarde si les points sont dans la bounding box
        #on utilise les indices pour eviter les loops

        res_bool = np.logical_and(np.logical_and(self.min_bound[0] <= points[:, 0], points[:, 0] <= self.max_bound[0]),
                                  np.logical_and(self.min_bound[1] <= points[:, 1], points[:, 1] <= self.max_bound[1]))
        res_bool = np.logical_and(res_bool, np.logical_and(self.min_bound[2] <= points[:, 2], points[:, 2] <= self.max_bound[2]))

        return res_bool


    """
 
    



#load a ply available in the open3d dataset
ply_point_cloud = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(ply_point_cloud.path)
pcd.paint_uniform_color([0.5, 1, 0.5])
array_pcd1 = np.asarray(pcd.points)


#on cree n autre nuage legerement decale
translated_pcd = o3d.geometry.PointCloud(pcd)
translated_pcd.points = o3d.utility.Vector3dVector(np.asarray(translated_pcd.points) + [0.5, 0, 0.5])
translated_pcd.paint_uniform_color([0.5, 0.5, 1])
array_pcd2 = np.asarray(translated_pcd.points)



#on cree une bounding box qui entoure les deux nuages
bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np.concatenate((array_pcd1, array_pcd2), axis=0)))
bbox.color = [1, 0, 0]

list_mini_boxes = [] #liste avec des objets bounding box
size = 1 #10 m

#on va discretiser la bounding box en mini bounding box de 1m de cote
for x in np.arange(bbox.min_bound[0], bbox.max_bound[0], size):
    for y in np.arange(bbox.min_bound[1], bbox.max_bound[1], size):
        for z in np.arange(bbox.min_bound[2], bbox.max_bound[2], size):
            mini_bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d.utility.Vector3dVector(np.array([[x, y, z], [x+size, y+size, z+size]])))
            mini_bbox.color = [1, 0, 1]
            list_mini_boxes.append(mini_bbox)



print(len(list_mini_boxes)) #on a 1000 mini bounding box


# on cherche a garder les overlaps entre les bounding box
            
#pour chque mini box, si elle contient des points des deux nuages, on la garde

o3d.visualization.draw_geometries([pcd, translated_pcd, bbox] + list_mini_boxes)

#on ne trace que la bounding box du milieu
mini_box_test = list_mini_boxes[15]
o3d.visualization.draw_geometries([pcd, translated_pcd, mini_box_test])

#on filtre pour ne garder que les points dans la bounding box
#on utilise les indices pour eviter les loops

#en 1 ligne
bool_has_points_from_cloud1 = np.all(np.logical_and(mini_box_test.min_bound <= array_pcd1, array_pcd1 <= mini_box_test.max_bound), axis=1)
bool_has_points_from_cloud2 = np.all(np.logical_and(mini_box_test.min_bound <= array_pcd2, array_pcd2 <= mini_box_test.max_bound), axis=1)

overlap = False
if np.sum(bool_has_points_from_cloud1) > 0 and np.sum(bool_has_points_from_cloud2) > 0:
    mini_box_test.color = [0, 1, 0]
    print("overlap")
    overlap = True
else:
    mini_box_test.color = [1, 0, 0]
    print("no overlap")


"""
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

print(np.sum(bool_has_points_from_cloud1)) #on a 0 points
print(np.sum(bool_has_points_from_cloud2)) #on a 0 points


overlap = False
if np.sum(bool_has_points_from_cloud1) > 0 and np.sum(bool_has_points_from_cloud2) > 0:
    mini_box_test.color = [0, 1, 0]
    print("overlap")
    overlap = True
else:
    mini_box_test.color = [1, 0, 0]
    print("no overlap")

"""

#maintenant, on construit 2 nouveaux nuages avec les points dans la bounding box
if overlap : 
    pcd1_in_box = o3d.geometry.PointCloud()
    pcd1_in_box.points = o3d.utility.Vector3dVector(array_pcd1[bool_has_points_from_cloud1])
    pcd1_in_box.paint_uniform_color([0.5, 1, 0.5])

    pcd2_in_box = o3d.geometry.PointCloud()
    pcd2_in_box.points = o3d.utility.Vector3dVector(array_pcd2[bool_has_points_from_cloud2])
    pcd2_in_box.paint_uniform_color([0.5, 0.5, 1])

    o3d.visualization.draw_geometries([pcd1_in_box, pcd2_in_box, mini_box_test] + list_mini_boxes)

