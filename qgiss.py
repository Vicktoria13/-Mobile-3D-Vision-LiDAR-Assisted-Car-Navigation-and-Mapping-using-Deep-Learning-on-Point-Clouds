# Importer les modules nécessaires de QGIS
from qgis.core import QgsProject, QgsVectorLayer, QgsSpatialIndex

# Chemin vers le fichier GQZ contenant vos scans géoréférencés
chemin_fichier_gqz = "/media/topostudent/Data1/2024spring_VictoriaZ/01_raw_data/04_CALCULS/230905/LASER/5_VECTORS/scan_map.qgz"


QgsProject.instance().read(chemin_fichier_gqz)

#affiche le nom des couches disponibles dans le projet
for couche in QgsProject.instance().mapLayers().values():
    print(couche.name())

# Obtenir la couche de scans à partir du projet
couche_scans = QgsProject.instance().mapLayersByName("EXTEND_SDC_RIGHT_M230905")[0]

# Créer un index spatial pour la couche de scans
index_spatial = QgsSpatialIndex(couche_scans)

# lister tous les scans et leur nom
for scan in couche_scans.getFeatures():
    print(scan.attribute("name"))

print("il y a ", couche_scans.featureCount(), "scans dans la couche")


#affiche la liste des attributs disponibles pour la couche
for field in couche_scans.fields():
    print(field.name())

#affiche les attributs du 1er scan
print(couche_scans.getFeature(0).attributes())


#creer une liste de pair de scans qui se chevauchent : on inclut pas le scan lui-même



chevauchements = []
list_chevauchements = []

# Parcourir tous les scans
for scan in couche_scans.getFeatures():

    #scan est un objet : cela represente un scan parmi les 352 scans
    
    # Obtenir les identifiants des scans qui se chevauchent avec le scan actuel
    chevauchements = index_spatial.intersects(scan.geometry().boundingBox())

    
    #enlever le scan actuel de la liste des scans chevauchants
    chevauchements.remove(scan.id())
    
    for chevauchement in chevauchements:
        list_chevauchements.append([scan.id(), chevauchement])
    
    #print("Scan name: ", scan.attribute("name"))
    #print("Chevauchements: ", chevauchements)



#enlever les doublons
list_chevauchements = list(set([tuple(sorted(chevauchement)) for chevauchement in list_chevauchements]))

#pour chaque pairs de scans, afficher la longeur de chevauchement des bounding box !! : ainsi, il faut calculer la longueur de chevauchement des bounding box
new_list_chevauchements = []

seuil = 0.5
for chevauchement in list_chevauchements:
    chevauchement1 = couche_scans.getFeature(chevauchement[0]).geometry()
    chevauchement2 = couche_scans.getFeature(chevauchement[1]).geometry()
    chevauchement1.intersection(chevauchement2).length()
    
    longueur_chevauchement = chevauchement1.intersection(chevauchement2).length()

    #si la longueur est inférieure à un seuil, on ne garde pas la paire de scans
    if longueur_chevauchement < seuil:
        new_list_chevauchements.append(chevauchement)


#ecrire les paires de scans qui se chevauchent dans un fichier texte
#ligne 1 : nom_scan1 nom_scan2

with open("chevauchements.txt", "w") as f:
    for chevauchement in list_chevauchements:
        f.write(couche_scans.getFeature(chevauchement[0]).attribute("name") + " " + couche_scans.getFeature(chevauchement[1]).attribute("name") + "\n")

"""
# Charger le projet QGIS à partir du fichier GQZ
QgsProject.instance().read(chemin_fichier_gqz)

#affiche le nom des couches disponibles dans le projet
for couche in QgsProject.instance().mapLayers().values():
    print(couche.name())


# Obtenir la couche de scans à partir du projet
couche_scans = QgsProject.instance().mapLayersByName("Nom_de_votre_couche")[0]

# Créer un index spatial pour la couche de scans
index_spatial = QgsSpatialIndex(couche_scans)

# Liste pour stocker les identifiants des scans qui se chevauchent
scans_chevauchants = {}

#key = id du scan, list = id des scans chevauchants
#ici, les ids sont du type M20395-1514080-05045-
#ne pas inclure lui-même dans la liste des scans chevauchants !!! 

# Parcourir tous les scans
for scan in couche_scans.getFeatures():
    # Obtenir les identifiants des scans qui se chevauchent avec le scan actuel
    scans_chevauchants[scan.id()] = index_spatial.intersects(scan.geometry().boundingBox())

    #enlever le scan actuel de la liste des scans chevauchants
    scans_chevauchants[scan.id()].remove(scan.id())


#æffiche les attributs disponibles pour la couche
for field in couche_scans.fields():
    print(field.name())


# Afficher les identifiants des scans qui se chevauchent : les noms des fichiers !!
for scan, chevauchants in scans_chevauchants.items():
    print("=======================================")

    print("Scan name: ", couche_scans.getFeature(scan).attribute("name"))


"""