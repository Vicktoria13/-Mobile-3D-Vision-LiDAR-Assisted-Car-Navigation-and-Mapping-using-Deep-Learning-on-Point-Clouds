import numpy as np
import open3d as o3d
import pickle as pkl
from pathlib import Path
import tools


class ROI:
    def __init__(self, path_1, path_2):
        self.pc_1 = np.loadtxt(path_1)
        self.pc_2 = np.loadtxt(path_2)

        index_1 = np.arange(self.pc_1.shape[0]).reshape(-1, 1)
        index_2 = np.arange(self.pc_2.shape[0]).reshape(-1, 1)

        self.pc_1 = np.concatenate((index_1, self.pc_1), axis=1)
        self.pc_2 = np.concatenate((index_2, self.pc_2), axis=1)

        self.x_min = max(np.min(self.pc_1[:, 2]), np.min(self.pc_2[:, 2]))
        self.x_max = min(np.max(self.pc_1[:, 2]), np.max(self.pc_2[:, 2]))
        self.y_min = max(np.min(self.pc_1[:, 3]), np.min(self.pc_2[:, 3]))
        self.y_max = min(np.max(self.pc_1[:, 3]), np.max(self.pc_2[:, 3]))

    def visualize(self):
        pcd_1 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.pc_1[:, 2:5]))
        pcd_2 = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(self.pc_2[:, 2:5]))
        pcd_1.paint_uniform_color([1, 0.706, 0])
        pcd_2.paint_uniform_color([0.5, 0.5, 1])

        o3d.visualization.draw([pcd_1, pcd_2])

    def keepOverlap(self):

        #cela permet de garder que les points qui sont dans la zone de recouvrement
        mask_1 = np.any((self.pc_1[:, 2] < self.x_min, self.pc_1[:, 2] > self.x_max,
                         self.pc_1[:, 3] < self.y_min, self.pc_1[:, 3] > self.y_max),
                        axis=0)
        mask_2 = np.any((self.pc_2[:, 2] < self.x_min, self.pc_2[:, 2] > self.x_max,
                         self.pc_2[:, 3] < self.y_min, self.pc_2[:, 3] > self.y_max),
                        axis=0)
        
        #self.pc_1 est de taille (n, 8) et mask_1 est de taille (n,)
        self.pc_1 = self.pc_1[~mask_1]
        self.pc_2 = self.pc_2[~mask_2]
    
    def assignTileID(self, step_x, step_y):
        self.id_x1 = np.floor((self.pc_1[:, 2]-self.x_min)/step_x)
        self.id_y1 = np.floor((self.pc_1[:, 3]-self.y_min)/step_y)

        self.id_x2 = np.floor((self.pc_2[:, 2]-self.x_min)/step_x)
        self.id_y2 = np.floor((self.pc_2[:, 3]-self.y_min)/step_y)

    def exportTile(self, i, j, tile_number, step_x, step_y, match_file, outpath, P2P_path, visualize):
        mask1 = np.logical_and(self.id_x1 == i, self.id_y1 == j)
        mask2 = np.logical_and(self.id_x2 == i, self.id_y2 == j)

        tile_1 = self.pc_1[mask1, :]
        tile_2 = self.pc_2[mask2, :]

        den1 = tile_1.shape[0]/(step_x*step_y)
        den2 = tile_2.shape[0]/(step_x*step_y)

        print(f"Tile {i}-{j}: densities are d1={den1} - d2={den2}")

        if (den1 > 20 and den2 > 20) and (den1/den2 > 0.6 and den1/den2 < 1/0.6):

            np.savetxt(f"{outpath}tiles/chunk_a{tile_number}.txt",tile_1,fmt='%d, %.8f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f',comments='')
            np.savetxt(f"{outpath}tiles/chunk_b{tile_number}.txt",tile_2,fmt='%d, %.8f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f',comments='')

            match_file.write(f"python3 {P2P_path}/matching_pipeline.py -n {P2P_path}/LCD/LCD_source/logs/LCD-D256/model.pth -f {outpath} -i1 a{tile_number} -i2 b{tile_number}\n")

            print(f"Saving Tile {i}-{j}, to tile n°{tile_number}")
            tile_number += 1
        else:
            print(f"Skiping tile {i}-{j}, density to low or to unbalanced")
            tile_number = tile_number
            
        return tile_number
            


    