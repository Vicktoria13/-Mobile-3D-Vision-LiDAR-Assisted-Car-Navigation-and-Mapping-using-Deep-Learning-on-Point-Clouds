"""
@Author : Victoria NGUYEN
@Date : 8 avril 2024
ENAC - Projet de semestre : 2024
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import argparse
import logging



"""
python3 stats_MLS.py --path_glob "/home/sdi-2023-01/Bureau/DATA P2P/res_18_mars/res_fourth_set_10_old_weight/log/"
"""


def main():
    logging.basicConfig(level=logging.INFO)
    logging.info("Starting the program ...")

    # =============== ARGUMENTS ===============
    parser = argparse.ArgumentParser(description='Plots RMSE')
    parser.add_argument('--path_glob', type=str, help='Path to the glob folder')

    RMSE_RAW_POINTS1 = []
    RMSE_GEOM_POINTS1 = []
    RMSE_RANSAC1 = []
    RMSE_ICP_POINTS1 = []

    args = parser.parse_args()
    path_folder1 = args.path_glob

    for filename in os.listdir(path_folder1):
        with open(path_folder1 + filename, 'r') as file:
            
            
            lines = file.readlines()
            
            #1ere ligne ==> RMSE_RAW_POINTS
            RMSE_RAW_POINTS1.append(lines[0].split(" ")[-1])

            #2eme ligne ==> RMSE_GEOM_POINTS
            RMSE_GEOM_POINTS1.append(lines[1].split(" ")[-1])

            #3eme ligne ==> RMSE_RANSAC
            RMSE_RANSAC1.append(lines[2].split(" ")[-1])

            #4eme ligne ==> RMSE_ICP_POINTS
            RMSE_ICP_POINTS1.append(lines[3].split(" ")[-1])

    


    #arrondir au 2eme chiffre apres la virgule
    for i in range(len(RMSE_RAW_POINTS1)):
        RMSE_RAW_POINTS1[i] = round(float(RMSE_RAW_POINTS1[i]), 2)
        RMSE_GEOM_POINTS1[i] = round(float(RMSE_GEOM_POINTS1[i]), 2)
        RMSE_RANSAC1[i] = round(float(RMSE_RANSAC1[i]), 2)
        RMSE_ICP_POINTS1[i] = round(float(RMSE_ICP_POINTS1[i]), 2)


    #enleve le max
    RMSE_RAW_POINTS1.remove(max(RMSE_RAW_POINTS1))
    RMSE_GEOM_POINTS1.remove(max(RMSE_GEOM_POINTS1))
    RMSE_RANSAC1.remove(max(RMSE_RANSAC1))
    RMSE_ICP_POINTS1.remove(max(RMSE_ICP_POINTS1))

    

    # =============== PLOT ===============
    fig, ax = plt.subplots()
    ax.plot(RMSE_RAW_POINTS1, label='RMSE_RAW_POINTS')
    ax.plot(RMSE_GEOM_POINTS1, label='RMSE_GEOM_POINTS')
    ax.plot(RMSE_RANSAC1, label='RMSE_RANSAC')
    ax.plot(RMSE_ICP_POINTS1, label='RMSE_ICP_POINTS')


    title = "RMSE ICP mean : " + str(round(np.mean(RMSE_ICP_POINTS1), 4)) + " std : " + str(round(np.std(RMSE_ICP_POINTS1), 4)) + "[" + str(round(np.min(RMSE_ICP_POINTS1), 4)) + ", " + str(round(np.max(RMSE_ICP_POINTS1), 4)) + "]"
    ax.set_title(title)
    ax.legend()
    ax.set(xlabel='chunks number', ylabel='RMSE value')


    #on save le plot en reculant d'un cran dans le path
    #ie au lieu de /home/sdi-2023-01/Bureau/DATA P2P/res_18_mars/res_fourth_set_10_old_weight/log/
    #on aura /home/sdi-2023-01/Bureau/DATA P2P/res_18_mars/res_fourth_set_10_old_weight/rmse.png

    out_path = path_folder1.split("/")[:-2]
    out_path = "/".join(out_path) + "/"
    out_path = out_path + "rmse.png"




    plt.savefig(out_path)

    logging.info("Plot saved at : " + out_path)


if __name__ == "__main__":
    main()