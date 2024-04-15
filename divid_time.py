import laspy
import numpy as np
import open3d as o3d
import polyscope as ps


#read file

infile = laspy.read("/home/sdi-2023-01/Downloads/85268 - M230905_217740_217756_RIGHT.laz")


print("il y a ",infile.header.point_count,"points dans le fichier")


#min des GPS time (donné en secondes)
print("le min des gps time est ",min(infile.gps_time))
print("le max des gps time est ",max(infile.gps_time))

#l'attribut gps time est un tableau de float, qui donne le temps en secondes durant lequel le point a été capturé

#ordonner les points par gps time
#on peut utiliser np.argsort pour obtenir les indices qui trieraient le tableau
import numpy as np
sorted_indices = np.argsort(infile.gps_time)

#on peut ensuite utiliser ces indices pour trier les points
sorted_points = infile.points[sorted_indices]


sorted_point_xyz = np.vstack((sorted_points.x, sorted_points.y, sorted_points.z)).transpose()


time_range = max(infile.gps_time) - min(infile.gps_time)
print("la plage de temps est de ",time_range,"secondes")


#on peut diviser les points en 10 groupes de temps
time_divisions = 10
time_step = time_range / time_divisions
print("les divisions de temps sont de ",time_step,"secondes")

#on peut ensuite diviser les points en fonction de leur temps
divided_points = []

for i in range(time_divisions):
    division_indices = (infile.gps_time >= min(infile.gps_time) + i * time_step) & (infile.gps_time < min(infile.gps_time) + (i+1) * time_step)
    division_points = infile.points[division_indices]
    divided_points.append(division_points)

#on peut ensuite afficher le nombre de points dans chaque division
for i in range(time_divisions):
    print("il y a ",len(divided_points[i]),"points dans la division ",i)

## polyscope

#register with radius and random color
radius = 0.0003

ps.init()
for i in range(time_divisions):
    ps.register_point_cloud("points"+str(i), sorted_point_xyz, radius=radius, color=[np.random.rand(), np.random.rand(), np.random.rand()])
ps.show()