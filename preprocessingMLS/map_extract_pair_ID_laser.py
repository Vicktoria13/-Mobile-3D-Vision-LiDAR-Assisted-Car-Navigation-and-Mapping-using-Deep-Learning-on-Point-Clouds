from qgis.core import QgsProject, QgsVectorLayer, QgsSpatialIndex
import os
import argparse
import logging

"""
python3 map_extract_pair_ID_laser.py --qgz_file /media/topostudent/Data1/2024spring_VictoriaZ/chevauchement.txt
"""
def main():

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger(__name__)

    logger.info(" ====== Extracting pair of ID of overlapping scans from a QGZ file ====== ")

    #get the arguments
    parser = argparse.ArgumentParser(description='Extract pair of ID of overlapping scans from a QGZ file')
    parser.add_argument('--qgz_file', type=str, help='The path to the QGZ file containing the scans')
    parser.add_argument('--out', type=str, help='The path to the output txt file')
    args = parser.parse_args()

    path_qgz = args.qgz_file
    path_out_txt = args.out

    QgsProject.instance().read(path_qgz)
    couche_scans = QgsProject.instance().mapLayersByName("EXTEND_SDC_RIGHT_M230905")[0]
    index_spatial = QgsSpatialIndex(couche_scans)

    print("il y a ", couche_scans.featureCount(), "scans dans la couche")

    logger.info("=== Reading the scans ...  ")

    chevauchements = []
    list_chevauchements = []

    for scan in couche_scans.getFeatures():

        chevauchements = index_spatial.intersects(scan.geometry().boundingBox())
        chevauchements.remove(scan.id())
        
        for chevauchement in chevauchements:
            list_chevauchements.append([scan.id(), chevauchement])

    
    #get rid of duplicates
    list_chevauchements = list(set([tuple(sorted(item)) for item in list_chevauchements]))
    print("BEFORE: ", len(list_chevauchements))

    #sort to have only overlapping width > threshold
    new_list_chevauchements = []
    seuil_area = 0.5

    for chevauchement in list_chevauchements:
        scan1 = couche_scans.getFeature(chevauchement[0])
        scan2 = couche_scans.getFeature(chevauchement[1])

        if scan1.geometry().intersects(scan2.geometry()):
            intersection = scan1.geometry().intersection(scan2.geometry())
            if intersection.area() > seuil_area:
                new_list_chevauchements.append(chevauchement)

    logger.info("=== Writing the pairs of overlapping scans to the output file ...  ")
    with open(path_out_txt, 'w') as f:
        for item in new_list_chevauchements:
            #Dont write M230905_207850_207900_RIGHT.rxp but only M230905_207850_207900
            #f.write(couche_scans.getFeature(item[0]).attribute("name") + " " + couche_scans.getFeature(item[1]).attribute("name") + "\n")
            id1 = couche_scans.getFeature(item[0]).attribute("name").split("_")[0] + "_" + couche_scans.getFeature(item[0]).attribute("name").split("_")[1] + "_" + couche_scans.getFeature(item[0]).attribute("name").split("_")[2]
            id2 = couche_scans.getFeature(item[1]).attribute("name").split("_")[0] + "_" + couche_scans.getFeature(item[1]).attribute("name").split("_")[1] + "_" + couche_scans.getFeature(item[1]).attribute("name").split("_")[2]
            f.write(id1 + " " + id2 + "\n")
            
    
    
    print("AFTER: ", len(new_list_chevauchements))


    ############## Write a sh file where each line is :
    # python3 merge_left_right_and_extract_overlapping
    
    return 0


if __name__ == "__main__":
    main()