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



#python align_chunks.py --path1 /home/sdi-2023-01/Téléchargements/res_MLS_simple_8/tiles/chunk_a3.txt --path2 /home/sdi-2023-01/Téléchargements/res_MLS_simple_8/tiles/chunk_b3.txt --visualize True

def main():

    #ßet level of log info
    logging.basicConfig(level=logging.INFO)
    logging.info("Starting the program ...")

    # =============== ARGUMENTS ===============
    parser = argparse.ArgumentParser(description='Align Pair Chunks')
    parser.add_argument('--path1', type=str, help='Path to the first .txt file')
    parser.add_argument('--path2', type=str, help='Path to the second .txt file')
    parser.add_argument('--visualize', type=bool, help='Visualize the overlapping area')

    args = parser.parse_args()




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




    ################################################### OPEN3D ICP
    logging.info(" \n =============== OPEN3D ICP ==============")
    icp_res = o3d.pipelines.registration.registration_icp(shifted_pcd1_o3d, shifted_pcd2_o3d, 
                                                        0.9, np.eye(4), 
                                                        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                                                        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=500, relative_rmse=0.000001, relative_fitness=1e-6))
    
    ###################################################################
    


    transformation_OPEN3D = icp_res.transformation
    transformation_SIMPLE_ICP = H_matrix

    print("RMSE for OPEN3D ICP: ", icp_res.inlier_rmse)
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

    result_pcd1_SIMPLE_ICP.points = o3d.utility.Vector3dVector(np.array(result_pcd1_SIMPLE_ICP.points) + np.array([x_shift_both, y_shift_both, z_shift_both]))



    ########################### OPEN3D ICP

    


    result_pcd1_OPEN3D = o3d.geometry.PointCloud()
    result_pcd1_OPEN3D.points = o3d.utility.Vector3dVector(np.array(shifted_pcd1_o3d.points)) 
    result_pcd1_OPEN3D.transform(transformation_OPEN3D)


    if args.visualize:
        ps.init()
        ps.register_point_cloud("pcd1", np.asarray(result_pcd1_OPEN3D.points), radius=0.0005)
        ps.register_point_cloud("pcd2", np.asarray(shifted_pcd2_o3d.points), radius=0.0005)
        ps.show()
   
    # UNSHIFT 
    result_pcd1_OPEN3D.points = o3d.utility.Vector3dVector(np.array(result_pcd1_OPEN3D.points) + np.array([x_shift_both, y_shift_both, z_shift_both]))






    logging.info("Saving the aligned point clouds ...")

    ### SIMPLE ICP
    np.savetxt(root_path + name_pcd1 + "_aligned_SIMPLE_ICP.txt", np.asarray(result_pcd1_SIMPLE_ICP.points), delimiter=delim)
    np.savetxt(root_path + name_pcd2 + "_aligned_SIMPLE_ICP.txt", np.asarray(pcd2.points), delimiter=delim)

    ### OPEN3D ICP
    np.savetxt(root_path + name_pcd1 + "_aligned_OPEN3D_ICP.txt", np.asarray(result_pcd1_OPEN3D.points), delimiter=delim)
    np.savetxt(root_path + name_pcd2 + "_aligned_OPEN3D_ICP.txt", np.asarray(pcd2.points), delimiter=delim)

   

    logging.info("Program finished ...")

if __name__ == "__main__":
    main()