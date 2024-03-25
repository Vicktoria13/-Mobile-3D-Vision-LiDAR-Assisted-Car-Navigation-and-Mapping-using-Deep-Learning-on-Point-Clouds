# Importer les modules nécessaires de QGIS
from qgis.core import QgsProject, QgsVectorLayer, QgsSpatialIndex

# Chemin vers le fichier GQZ contenant vos scans géoréférencés
chemin_fichier_gqz = "/chemin/vers/votre/fichier.gqz"


#print l'attribut name de chque scan

# Charger le projet QGIS à partir du fichier GQZ
QgsProject.instance().read(chemin_fichier_gqz)

#affiche le nom des couches disponibles dans le projet
for couche in QgsProject.instance().mapLayers().values():
    print(couche.name())

# Obtenir la couche de scans à partir du projet
couche_scans = QgsProject.instance().mapLayersByName("Nom_de_votre_couche")[0]

# Créer un index spatial pour la couche de scans
index_spatial = QgsSpatialIndex(couche_scans)

# lister tous les scans et leur nom
for scan in couche_scans.getFeatures():
    print(scan.attribute("name"))

print("il y a ", couche_scans.featureCount(), "scans dans la couche")

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