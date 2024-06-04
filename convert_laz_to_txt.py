import laspy
import numpy as np
import open3d as o3d



inFile1 = laspy.read("/home/sdi-2023-01/Downloads/85268 - M230905_200520_200522_LEFT.laz")
coords1= np.vstack((inFile1.x, inFile1.y, inFile1.z)).transpose()
print(coords1.shape)
#le temps est la premiere colonne : attention pas d'attribut time dans le fichier laz
time1 = np.vstack((inFile1.gps_time)).transpose()

inFile2 = laspy.read("/home/sdi-2023-01/Downloads/85268 - M230905_200520_200522_RIGHT.laz")
coords2= np.vstack((inFile2.x, inFile2.y, inFile2.z)).transpose()
time2 = np.vstack((inFile2.gps_time)).transpose()


#visu open3d
pcd1 = o3d.geometry.PointCloud()
pcd1.points = o3d.utility.Vector3dVector(coords1)

pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(coords2)

#coulor
pcd1.paint_uniform_color([0.1, 0.1, 0.7])
pcd2.paint_uniform_color([0.7, 0.1, 0.1])
o3d.visualization.draw_geometries([pcd1, pcd2])


#ajout de 1 colonne de 0 au debut et 3 colonnes de 0 a la fin
new_coords1 = np.hstack((np.zeros((coords1.shape[0],1)), coords1))
new_coords1 = np.hstack((new_coords1, np.zeros((coords1.shape[0],3)))  )

new_coords2 = np.hstack((np.zeros((coords2.shape[0],1)), coords2))
new_coords2 = np.hstack((new_coords2, np.zeros((coords2.shape[0],3))))


#mettre la colonne 0 = temps
new_coords1[:,0] = time1
new_coords2[:,0] = time2


#on veut t x y z 0 0 0 provenance. Ici rajouter donc une dernier colonne de 0 pour la provenance
new_coords1 = np.hstack((new_coords1, np.zeros((coords1.shape[0],1))))
new_coords2 = np.hstack((new_coords2, np.zeros((coords2.shape[0],1))))



#save en txt    
np.savetxt("/home/sdi-2023-01/Downloads/85268 - M230905_200520_200522_LEFT.txt", new_coords1, delimiter=" ", fmt="%s")
np.savetxt("/home/sdi-2023-01/Downloads/85268 - M230905_200520_200522_RIGHT.txt", new_coords2, delimiter=" ", fmt="%s")

