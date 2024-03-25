import os
import argparse
import logging


"""
python3 --data_las /media/topostudent/Data1/2024spring_VictoriaZ/data
"""

def main():

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    logger.info(" ====== Merge Left Right for each cloud of the pair + extract overlapping ====== ")

    #get the arguments
    parser = argparse.ArgumentParser(description='Merge Left Right for each cloud of the pair + extract overlapping')
    parser.add_argument('--data_las', type=str, help='The path to the folder containing the las files with R and L clouds')
    parser.add_argument('--id1', type=str, help='The ID of the first cloud')
    parser.add_argument('--id2', type=str, help='The ID of the second cloud')

    parser.add_argument('--out', type=str, help='The path to the output txt file')



    #################### 1. Get the arguments ####################
    args = parser.parse_args()
    path_folder_data_las = args.data_las
    id1 = args.id1
    id2 = args.id2
    path_out_txt = args.out

    #################### 2. in the folder 'path_folder_data_las', get the left and right clouds paths
    logger.info("=== Looking for LEFT and RIGHT clouds of + " + id1)
    
