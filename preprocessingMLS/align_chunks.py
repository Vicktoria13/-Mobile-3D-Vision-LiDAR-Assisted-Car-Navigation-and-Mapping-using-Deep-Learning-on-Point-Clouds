"""
@Author : Victoria NGUYEN
@Date : 8 avril 2024
ENAC - Projet de semestre : 2024
"""


import open3d as o3d
import numpy as np
import laspy
from simpleicp import PointCloud, SimpleICP
import polyscope as ps
import logging
import argparse

"""

Input : Chunk A and Chunk B, in .txt format
ChunkA.txt and ChunkB.txt are the point clouds of the two chunks : 7 columns. X Y Z are columns 2 3 4

Output : ChunkA_aligned.txt and ChunkB_aligned.txt, in .txt format

"""
FLAG_is_tile = True
delim = ","




"""
python align_chunks.py --path1 /home/sdi-2023-01/Téléchargements/res_MLS_simple_8/tiles/chunk_a4.txt --path2 /home/sdi-2023-01/Téléchargements/res_MLS_simple_8/tiles/chunk_b4.txt --visualize True
"""



def main():

    #ßet level of log info
    logging.basicConfig(level=logging.INFO)
    logging.info("Starting the program ...")

    # =============== ARGUMENTS ===============
    parser = argparse.ArgumentParser(description='Align Pair Chunks')
    parser.add_argument('--path1', type=str, help='Path to the first .txt file')
    parser.add_argument('--path2', type=str, help='Path to the second .txt file')
    parser.add_argument('--visualize', action="store_true")
    #--visualize True or False
    args = parser.parse_args()


    print(" Visualze set to: ", args.visualize)


    logging.info("Extracting the pcds paths ...")

    ###################
    name_pcd1 = args.path1.split("/")[-1].split(".")[0] #on récupère le nom du fichier
    name_pcd2 = args.path2.split("/")[-1].split(".")[0]

    #pour avoir /home/sdi-2023-01/Téléchargements/pcd_one_set/
    root_path = "/".join(args.path1.split("/")[:-1]) + "/"
    ###################



    #on ne garde que les colonnes 2 3 4 pour les tiles
    point_pcd1 = (np.loadtxt(args.path1, delimiter=delim))[:,2:5]
    point_pcd2 = (np.loadtxt(args.path2, delimiter=delim))[:,2:5]



    pcd1 = o3d.geometry.PointCloud()
    pcd2 = o3d.geometry.PointCloud()

    pcd1.points = o3d.utility.Vector3dVector(point_pcd1)
    pcd2.points = o3d.utility.Vector3dVector(point_pcd2)



    logging.info("Shifting the chunks around the origin ...")
    x_shift_both = np.min([np.min(point_pcd1[:,0]), np.min(point_pcd2[:,0])])
    y_shift_both = np.min([np.min(point_pcd1[:,1]), np.min(point_pcd2[:,1])])
    z_shift_both = np.min([np.min(point_pcd1[:,2]), np.min(point_pcd2[:,2])])

    print("x_shift_both: ", x_shift_both)
    print("y_shift_both: ", y_shift_both)
    print("z_shift_both: ", z_shift_both)
    print("=====================================\n")

    shifted_pcd1_array = point_pcd1 - np.array([x_shift_both, y_shift_both, z_shift_both])
    shifted_pcd2_array = point_pcd2 - np.array([x_shift_both, y_shift_both, z_shift_both])


    if args.visualize:
        ps.init()
        ps.register_point_cloud("pcd1", shifted_pcd1_array, radius=0.0005)
        ps.register_point_cloud("pcd2", shifted_pcd2_array, radius=0.0005)
        ps.show()



    shifted_pcd1_o3d = o3d.geometry.PointCloud()
    shifted_pcd2_o3d = o3d.geometry.PointCloud()

    shifted_pcd1_o3d.points = o3d.utility.Vector3dVector(shifted_pcd1_array)
    shifted_pcd2_o3d.points = o3d.utility.Vector3dVector(shifted_pcd2_array)


    ################################################### VERSion SIMPLE ICP
    logging.info(" \n =============== SIMPLE ICP ==============")
    X_fix = np.array(shifted_pcd2_o3d.points)
    X_moving = np.array(shifted_pcd1_o3d.points)

    pc_fix = PointCloud(X_fix, columns=["x", "y", "z"])
    pc_moving = PointCloud(X_moving, columns=["x", "y", "z"])

    icp = SimpleICP()
    icp.add_point_clouds(pc_fix, pc_moving)
    H, X_mov_transformed, rigid_body_transformation_params, distance_residuals = icp.run(max_overlap_distance=1)

    H_matrix = np.array(H)
    ###################################################################



    transformation_SIMPLE_ICP = H_matrix
    print("RMSE for SIMPLE ICP: ", distance_residuals[-1])
 

    logging.info("Applying the transformation to the original point clouds ...")


    ########################### SIMPLE ICP
    result_pcd1_SIMPLE_ICP = o3d.geometry.PointCloud()
    result_pcd1_SIMPLE_ICP.points = o3d.utility.Vector3dVector(np.array(shifted_pcd1_o3d.points))
    result_pcd1_SIMPLE_ICP.transform(transformation_SIMPLE_ICP)
    

    if args.visualize:
        ps.init()
        ps.register_point_cloud("pcd1", np.asarray(result_pcd1_SIMPLE_ICP.points), radius=0.0005   )
        ps.register_point_cloud("pcd2", np.asarray(shifted_pcd2_o3d.points), radius=0.0005)
        ps.show()

    






    ########################### AFFICHE LES 4 POINT CLOUDS
    if args.visualize:
        ps.init()
        
        #couleur différente pour chaque point cloud : par defaut, seuls les originaux sont affichés
        ps.register_point_cloud("pcd1 original", np.asarray(shifted_pcd1_o3d.points), radius=0.0005, color=[1,0,0], enabled=True)
        ps.register_point_cloud("pcd2 original", np.asarray(shifted_pcd2_o3d.points), radius=0.0005, color=[0,1,0], enabled=True)

        #ICP
        ps.register_point_cloud("pcd1 ICP", np.asarray(result_pcd1_SIMPLE_ICP.points), radius=0.0005, color=[0,0,1], enabled=False)
        ps.register_point_cloud("pcd2 ICP", np.asarray(shifted_pcd2_o3d.points), radius=0.0005, color=[0,0.75,1], enabled=False)


        ps.show()

    ########################### UNSHIFT 
        
    result_pcd1_SIMPLE_ICP.points = o3d.utility.Vector3dVector(np.array(result_pcd1_SIMPLE_ICP.points) + np.array([x_shift_both, y_shift_both, z_shift_both]))




    logging.info("Saving the aligned point clouds ...")

    ### ICI : adding 2 columns of 0 at the beginning and 3 at the end



    ### SIMPLE ICP : overwrite the original files
    np.savetxt(root_path + name_pcd1 + ".txt", (np.concatenate((np.zeros((np.asarray(result_pcd1_SIMPLE_ICP.points).shape[0],2)), np.asarray(result_pcd1_SIMPLE_ICP.points), np.zeros((np.asarray(result_pcd1_SIMPLE_ICP.points).shape[0],3))), axis=1)), delimiter=delim)
    np.savetxt(root_path + name_pcd2 + ".txt", (np.concatenate((np.zeros((np.asarray(pcd2.points).shape[0],2)), np.asarray(pcd2.points), np.zeros((np.asarray(pcd2.points).shape[0],3))), axis=1)), delimiter=delim)

    


    logging.info("Program finished ...")

if __name__ == "__main__":
    main()