
#%%
import numpy as np
import laspy
import open3d as o3d
import polyscope as ps


inFile = laspy.read("/home/sdi-2023-01/Bureau/epfl/Data_pcd/85268 - M230905_223260_223320_LEFT.laz")
coords = np.vstack((inFile.x, inFile.y, inFile.z)).transpose()


#%%

print(coords.shape)
print("1ere ligne = ", coords[0])
#ajouter une colonne de 0 au debut et 2 colonnes de 0 a la fin



new_coords = np.hstack((np.zeros((coords.shape[0],1)), coords))
new_coords = np.hstack((new_coords, np.zeros((coords.shape[0],2))))

print(new_coords.shape)
print("1ere ligne = ", new_coords[0])


def from_laz_to_txt_with_7_columns(inPath, outPath):
    inFile = laspy.read(inPath)
    coords = np.vstack((inFile.x, inFile.y, inFile.z)).transpose()
    # new coordinate to easily adapt with the current code

    new_coords = np.hstack((np.zeros((coords.shape[0],1)), coords))
    new_coords = np.hstack((new_coords, np.zeros((coords.shape[0],3))))
    np.savetxt(outPath, new_coords, delimiter=" ", fmt="%s")




def from_seg_to_txt_with_7_columns(inPath, outPath):
    inFile = np.loadtxt(inPath)
    
    #keep only the pcd
    coords = inFile[:, 0:3]

    #then add the 4 columns of 0
    new_coords = np.hstack((coords, np.zeros((coords.shape[0],4))))

    np.savetxt(outPath, new_coords, delimiter=" ", fmt="%s")
    
