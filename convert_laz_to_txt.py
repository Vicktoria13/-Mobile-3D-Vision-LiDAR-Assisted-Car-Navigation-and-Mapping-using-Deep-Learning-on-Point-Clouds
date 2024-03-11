import laspy
import numpy as np
import open3d as o3d



inFile1 = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT.laz")
coords1= np.vstack((inFile1.x, inFile1.y, inFile1.z)).transpose()

inFile2 = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_RIGHT.laz")
coords2= np.vstack((inFile2.x, inFile2.y, inFile2.z)).transpose()

#ajout de 1 colonne de 0 au debut et 3 colonnes de 0 a la fin
new_coords1 = np.hstack((np.zeros((coords1.shape[0],1)), coords1))
new_coords1 = np.hstack((new_coords1, np.zeros((coords1.shape[0],3)))  )

new_coords2 = np.hstack((np.zeros((coords2.shape[0],1)), coords2))
new_coords2 = np.hstack((new_coords2, np.zeros((coords2.shape[0],3))))

#save en txt    
np.savetxt("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT.txt", new_coords1, delimiter=" ", fmt="%s")
np.savetxt("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_RIGHT.txt", new_coords2, delimiter=" ", fmt="%s")

