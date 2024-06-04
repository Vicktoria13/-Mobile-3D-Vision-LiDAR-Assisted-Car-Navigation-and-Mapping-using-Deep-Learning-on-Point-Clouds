

import numpy as np
import argparse
import matplotlib.pyplot as plt




PATHS = ["/home/sdi-2023-01/Downloads/fourth_set/merged/log.txt",
         "/home/sdi-2023-01/Downloads/res_veg_merged/log.txt",
         "/home/sdi-2023-01/Downloads/L2R/urb/log.txt",
         "/home/sdi-2023-01/Downloads/L2R/veg/log.txt"]


DESCRIPTIONS = ["S2S urban", "S2S veg", "L2R urban", "L2R veg"]
"""
PATHS = ["/home/sdi-2023-01/Downloads/fourth_set/not_merged_and_downsample/log.txt",
         "/home/sdi-2023-01/Downloads/fourth_set/res_not_merged/log.txt",
         "/home/sdi-2023-01/Downloads/fourth_set/merged/log.txt"]


DESCRIPTIONS = ["Downsample + not merged", "no Downsample + not merged", "no Downsample +  merged"]
"""
"""



PATHS = ["/home/sdi-2023-01/Downloads/L2R/urb/log.txt",
         "/home/sdi-2023-01/Downloads/L2R/veg/log.txt"]


DESCRIPTIONS = ["L2R urban", "L2R veg"]
"""


"""

PATHS = ["/home/sdi-2023-01/Downloads/res_veg_not_merged/log.txt",
        "/home/sdi-2023-01/Downloads/res_veg_merged/log.txt"]


DESCRIPTIONS = ["Veg not MERGED", "Veg MERGED"]

"""
colors = ['pink', 'lightblue', 'orange', 'purple', 'orange', 'yellow']

def fill_pkl_dict(path,description):
    global pkl_data

    #chaque fichier est du type
    # id_number rawKPA rawKPB error_icp nb_correspondancesA nb_correspondancesB densiteA densiteB time



    #remplir le dictionnaire pkl_data
    with open(path, "r") as f:
        for line in f:
            line = line.split() #split the line separated by space

            #KP
            pkl_data[description]["rawKPA"].append(int(line[1]))
            pkl_data[description]["rawKPB"].append(int(line[2]))

            #error icp
            pkl_data[description]["error_icp"].append(float(line[3]))

            #nb correspondances
            pkl_data[description]["nb_correspondancesA"].append(int(line[4]))
            pkl_data[description]["nb_correspondancesB"].append(int(line[5]))

            #densite
            pkl_data[description]["densityA"].append(float(line[6]))
            pkl_data[description]["densityB"].append(float(line[7]))

            #time
            pkl_data[description]["time"].append(float(line[8]))

                                                          
                                                                
            
                                                                


def plot_error_comparison():
    global pkl_data

    #plot error icp for each description, with 2 bboxplot on the same graph
    fig, ax = plt.subplots()
    
    #2 boites de couleurs differentes : la moyenne est affichee en vert via une ligne. PAS de mediane : pas de ligne
    
    """
    bp = plt.boxplot([pkl_data[DESCRIPTIONS[0]]["error_icp"], 
                      pkl_data[DESCRIPTIONS[1]]["error_icp"]], 
                      patch_artist=True, showmeans=True, meanline=True, meanprops={'color':'green', 'linewidth':2},showcaps=False, medianprops = None)
    """
    lis = []
    for desc in DESCRIPTIONS:
        lis.append(pkl_data[desc]["error_icp"])

    bp = plt.boxplot(lis, patch_artist=True, showmeans=True, meanline=True, meanprops={'color':'green', 'linewidth':2},showcaps=False, medianprops = None)
    
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)



    plt.grid()
    #titre
    plt.title("Error ICP comparison", fontsize=25)

    #for each box, we put the description. [1, 2] are the xticks
    plt.xticks([i for i in range(1, len(DESCRIPTIONS) + 1)], DESCRIPTIONS)
    plt.ylabel("Mean error between correspondances (m)", fontsize=20)


    #legend en fonction des couleurs + affiche la moyenne

    """
    plt.legend([DESCRIPTIONS[0] + ": {:.4f}".format(np.mean(pkl_data[DESCRIPTIONS[0]]["error_icp"])), 
                DESCRIPTIONS[1] + ": {:.4f}".format(np.mean(pkl_data[DESCRIPTIONS[1]]["error_icp"]) )], fontsize=25)
    """

    #pour la legendre : carré de couleur + description + moyenne
    #ax.legend([bp1["boxes"][0], bp2["boxes"][0]], ['A', 'B'], loc='upper right')
    plt.legend([bp["boxes"][i] for i in range(len(DESCRIPTIONS))], [desc + ": {:.4f}".format(np.mean(pkl_data[desc]["error_icp"])) for desc in DESCRIPTIONS], fontsize=25, loc = "upper right")

    plt.tick_params(axis='both', which='major', labelsize=20)

    #
    
    plt.show()
    
    



def plot_CDF_error():

    """
    for each description, plot the CDF of the error icp
    """

    global pkl_data

    fig, ax = plt.subplots()

    max = 0.75
    for desc in DESCRIPTIONS:
        error_icp = pkl_data[desc]["error_icp"]
        
        #plot jusqu'a error_icp = 1, 
        #on trie les valeurs
        error_icp.sort()

        #on cree un vecteur de proba
        y = np.linspace(0, 100, len(error_icp))
        plt.plot(error_icp, y, label = desc, color = colors[DESCRIPTIONS.index(desc)], linewidth = 5)

        #relie le 1er point a (0, 0). Mais aucne legende

    #actuellement, la courbe s'arrete a la valeur max de error_icp. On veut qu'elle affiche 100% jusqu'a la fin
    #plot une ligne horizontale entre le dernier point de la courbe et (max, 100)
    for desc in DESCRIPTIONS:
        error_icp = pkl_data[desc]["error_icp"]
        error_icp.sort()
        plt.plot([error_icp[-1], max], [100, 100], color = colors[DESCRIPTIONS.index(desc)], linewidth = 5)
        
   
    #taille des axes
    #l'axe de x va de 0 a 1
    plt.xlim(0, max)
    plt.grid()
    plt.title("CDF of the error ICP", fontsize=25)
    plt.xlabel("Error ICP (m)", fontsize=25)
    plt.ylabel("CDF (%)", fontsize=25)
    #afficher un tick tous les x = 0.05
    space = 0.05
    plt.xticks(np.arange(0, max + space, space))
    #legend : desc err 0.1 num _corr 100
    plt.legend([desc + " | err = {:.4f} |".format(np.mean(pkl_data[desc]["error_icp"])) for desc in DESCRIPTIONS], fontsize=25, loc = "lower right")



def plot_nb_correspondances():

    global pkl_data

    #plot error icp for each description, with 2 bboxplot on the same graph
    fig, ax = plt.subplots()

    lis = []
    for desc in DESCRIPTIONS:
        lis.append(pkl_data[desc]["nb_correspondancesA"])

    bp = plt.boxplot(lis, patch_artist=True, showmeans=True, meanline=True, meanprops={'color':'green', 'linewidth':2},showcaps=False, medianprops = None)

 
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)

    plt.grid()
    #titre
    plt.title("Number of correspondances", fontsize=25)
    plt.xticks([i for i in range(1, len(DESCRIPTIONS) + 1)], DESCRIPTIONS)
    plt.ylabel("Number of correspondances", fontsize=25)
    
    #taille des axes
    plt.tick_params(axis='both', which='major', labelsize=20)

    #legend en fonction des couleurs + affiche la moyenne
    plt.legend([bp["boxes"][i] for i in range(len(DESCRIPTIONS))], [desc + ": {:.4f}".format(np.mean(pkl_data[desc]["nb_correspondancesA"])) for desc in DESCRIPTIONS], fontsize=25, loc = "upper right")
    plt.show()








def plot_nb_total_KP():
    global pkl_data

    #plot error icp for each description, with 2 bboxplot on the same graph
    fig, ax = plt.subplots()
    
    lis = []
    for desc in DESCRIPTIONS:
        lis.append(pkl_data[desc]["rawKPA"])

    bp = plt.boxplot(lis, patch_artist=True, showmeans=True, meanline=True, meanprops={'color':'green', 'linewidth':2},showcaps=False, medianprops = None)
    #couleurs

    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)

    plt.grid()
    #titre
    plt.title("Total number of keypoints", fontsize=25)
    plt.xticks([i for i in range(1, len(DESCRIPTIONS) + 1)], DESCRIPTIONS)
    plt.ylabel("Total number of keypoints", fontsize=25)

    #taille des axes
    plt.tick_params(axis='both', which='major', labelsize=20)

    #legend en fonction des couleurs + affiche la moyenne
    plt.legend([bp["boxes"][i] for i in range(len(DESCRIPTIONS))], [desc + ": {:.4f}".format(np.mean(pkl_data[desc]["rawKPA"])) for desc in DESCRIPTIONS], fontsize=25, loc = "upper right")
    plt.show()




def plot_mean_time():

    global pkl_data

    #plot error icp for each description, with 2 bboxplot on the same graph
    fig, ax = plt.subplots()

    lis = []
    for desc in DESCRIPTIONS:
        lis.append(pkl_data[desc]["time"])


    bp = plt.boxplot(lis, patch_artist=True, showmeans=True, meanline=True, meanprops={'color':'green', 'linewidth':2},showcaps=False, medianprops = None)

    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)

    plt.grid()
    #titre
    plt.title("Time", fontsize=25)
    plt.xticks([i for i in range(1, len(DESCRIPTIONS) + 1)], DESCRIPTIONS)
    plt.ylabel("Time (s)", fontsize=25)
    
    #taille des axes
    plt.tick_params(axis='both', which='major', labelsize=15)

    #legend en fonction des couleurs + affiche la moyenne
    plt.legend([bp["boxes"][i] for i in range(len(DESCRIPTIONS))], [desc + ": {:.4f}".format(np.mean(pkl_data[desc]["time"])) for desc in DESCRIPTIONS], fontsize=25, loc = "upper right")
    plt.show()


#¢ree un dictionnaire pour chaque path {description: path}
info_dict = {}
for i in range(len(PATHS)):
    info_dict[DESCRIPTIONS[i]] = PATHS[i]



#pour chaque description,  load les fichiers pkl. On creera un dict :
#pkl data : {description: {"rawA" : [rawA1, rawA2, ...], "rawB" : [rawB1, rawB2, ...], "nb_correspondances" : [nb_correspondances1, nb_correspondances2, ...], "error_icp" : [error_icp1, error_icp2, ...], "densityAB" : [densityAB1, densityAB2, ...]}}


#initie le dictionnaire
pkl_data = {}
for desc in DESCRIPTIONS:
    pkl_data[desc] = {"rawKPA" : [], "rawKPB" : [], "error_icp" :[],"nb_correspondancesA" : [], "nb_correspondancesB" : [], "densityA" : [], "densityB" : [], "time" : []}


#remplir le dictionnaire
for desc in DESCRIPTIONS:
    fill_pkl_dict(info_dict[desc], desc)




######## PLOT ERROR COMPARISON
plot_error_comparison()

######## PLOT NB TOTAL KP
plot_nb_total_KP()

######## PLOT NB CORRESPONDANCES
plot_nb_correspondances()


######## PLOT CDF ERROR
plot_CDF_error()

######## PLOT MEAN TIME
plot_mean_time()