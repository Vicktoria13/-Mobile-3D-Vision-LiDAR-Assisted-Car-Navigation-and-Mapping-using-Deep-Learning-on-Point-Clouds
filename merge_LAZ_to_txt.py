import laspy
import numpy as np
import open3d as o3d

#prends un left un right, merge les 2 et save en txt

Right = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_RIGHT.laz")
coords_right = np.vstack((Right.x, Right.y, Right.z)).transpose()

Left = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT.laz")
coords_left = np.vstack((Left.x, Left.y, Left.z)).transpose()

#merge les 2
coords = np.vstack((coords_right, coords_left))
print("coords.shape: ", coords.shape)

#ajout de 1 colonne de 0 au debut et 3 colonnes de 0 a la fin
new_coords = np.hstack((np.zeros((coords.shape[0],1)), coords))
new_coords = np.hstack((new_coords, np.zeros((coords.shape[0],3))))

#save en txt
np.savetxt("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT_RIGHT.txt", new_coords, delimiter=" ", fmt="%s")