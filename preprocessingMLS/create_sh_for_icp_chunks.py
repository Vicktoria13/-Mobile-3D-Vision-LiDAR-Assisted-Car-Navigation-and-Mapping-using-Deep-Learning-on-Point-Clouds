"""
@Author : Victoria NGUYEN
@Date : 8 avril 2024
ENAC - Projet de semestre : 2024
"""

import argparse
import open3d as o3d
import numpy as np
import logging
import os

""" 
Code that creaat a .sh file to run the align_chunks.py script on all the pairs of chunks in the tiles directory
exemple of use :


python create_sh_for_icp_chunks.py --path_tiles /media/topostudent/Data1/2024spring_VictoriaZ/TEST-RES-after-eastern/res-8-april-merged/tiles/

Un .sh va etre creer dans 
/media/topostudent/Data1/2024spring_VictoriaZ/TEST-RES-after-eastern/res-8-april-merged/run_icp_MLS.sh

Ainsi, chaque ligne du .sh devra etre du type

python3 chemin to preprocessingMLS/align_chunks.py --path1 /media/topostudent/Data1/2024spring_VictoriaZ/TEST-RES-after-eastern/res-8-april-merged/tiles/chunk_a4.txt --path2 /media/topostudent/Data1/2024spring_VictoriaZ/TEST-RES-after-eastern/res-8-april-merged/tiles/chunk_b4.txt --visualize True

""" 


""" 
WARNING !
On a le fichier path/tiles/ et On copie tout le contenu dans un autre fichier path/tiles_non_aligned/
"""

def extract_pairs(rooth_path):
    """
    Extract pairs of chunks from the tiles directory
    Output :  dictionary of pairs keys : PATH/chunks_a1.txt, values : PATH/chunks_b1.txt
    """

    pairs = {}

    # =============== TILES ===============
    tiles = sorted(os.listdir(rooth_path))
    nb_pairs = len(tiles) // 2

    #create the dictionary of pairs
    for i in range(nb_pairs):
        pairs[rooth_path + tiles[i]] = rooth_path + tiles[i+nb_pairs]

    return pairs
  

def main():

    """
    OUTPUT : run_icp_MLS.sh

    where there are nb_pairs of lines, each line is a command to align two chunks eg
    python3 align_chunks.py --path1 /home/sdi-2023-01/Téléchargements/res_MLS_simple_10/tiles/chunk_a4.txt --path2 /home/sdi-2023-01/Téléchargements/res_MLS_simple_10/tiles/chunk_b4.txt --visualize True
    """

    logging.basicConfig(level=logging.INFO)
    logging.info(" =============== Starting the program ... =============== ")
    logging.info(" === This program will output a run_icp_MLS.sh file ===")

    # =============== ARGUMENTS ===============
    parser = argparse.ArgumentParser(description='ICPs')
    parser.add_argument('--path_tiles', type=str, help='Path to the tiles directory', required=True)
    args = parser.parse_args()

    pairs = extract_pairs(args.path_tiles)

    ########### RECUPERE LE CHEMIN ENTIER VERS align_chunks.py
    current_path = os.getcwd() #... /preprocessingMLS/
    
    


    logging.info("Creating the .sh file ...")

  

    #copy the tiles directory to tiles_non_aligned pour sauvegarder les fichiers originaux
    os.system(f"cp -r {args.path_tiles} {args.path_tiles[:-1] + '_non_aligned/'}")
    print("Copy into {} done".format(args.path_tiles[:-1] + '_non_aligned/') )

    

    #Save the sh file the same path as the tiles directory
    #sh_path = path/res_MLS_simple_10/
    #path_tiles = path/res_MLS_simple_10/tiles/
    #so we want to go to path/res_MLS_simple_10/

    sh_path = "/".join(args.path_tiles.split("/")[:-2]) + "/"
    os.chdir(sh_path)
    sh_path = sh_path + "run_icp_MLS.sh"
    print("sh_path : ", sh_path)

    path_to_align_chunks = current_path + "/align_chunks.py"

    # write the commands in the .sh file
    with open(sh_path , "w") as f:
        for path1, path2 in pairs.items():
            f.write(f"python3 {path_to_align_chunks} --path1 {path1} --path2 {path2}\n")

    #make the file executable
    os.system(f"chmod +x {sh_path}")
    




if __name__ == "__main__":

    main()
