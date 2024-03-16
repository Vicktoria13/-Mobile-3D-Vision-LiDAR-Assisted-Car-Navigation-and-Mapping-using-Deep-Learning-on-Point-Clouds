import argparse
import open3d as o3d
import numpy as np
import logging
import os



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
    parser.add_argument('--visualize', action="store_true")
    args = parser.parse_args()

    pairs = extract_pairs(args.path_tiles)

    logging.info("Creating the .sh file ...")

    print(" Visualze set to: ", args.visualize)
    
    if args.visualize:
        str_visu = "--visualize True"
    else:
        str_visu = ""
    with open("run_icp_MLS.sh", "w") as f:
        for path1, path2 in pairs.items():
            f.write(f"python3 align_chunks.py --path1 {path1} --path2 {path2} {str_visu}\n")


    #make the file executable
    os.system("chmod +x run_icp_MLS.sh")




if __name__ == "__main__":

    main()
