# %%
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import glob
import pickle as pkl
from matplotlib.colors import LinearSegmentedColormap


PATH = "/home/sdi-2023-01/Downloads/res_veg_merged/logs/logs/"

"""
PATH is a folder containing :
|
| -filt
| --- fichier1.pkl
| --- fichier2.pkl
| --- ...

| -glob
| --- fichier1.txt
| --- fichier2.txt
| --- ...

| -icp
| --- fichier1.pkl
| --- fichier2.pkl
| --- ...

| -raw
| --- fichier1.pkl
| --- fichier2.pkl
| --- ...
"""







    

class Stats:

    def __init__(self, PATH):
        #RAW
        raw_files = glob.glob(PATH + "raw/*.pkl")
        raw_files.sort()

        #After ICP
        icp_files = glob.glob(PATH + "icp/*.pkl")
        icp_files.sort()

        #aFTER FILT raNSAC
        rsc_files = glob.glob(PATH + "filt/*.pkl")
        rsc_files.sort()

        stats_raw = []
        stats_rsc = []
        stats_icp = []

        
        ################# READ RAW
        for i in range(len(raw_files)):
            f = open(raw_files[i], "rb")
            stats_raw.append(pkl.load(f))

        ################# READ RSC
        for i in range(len(rsc_files)):
            f = open(rsc_files[i], "rb")
            stats_rsc.append(pkl.load(f))

        ################# READ ICP
        for i in range(len(icp_files)):
            stats_icp.append(pkl.load(open(icp_files[i], "rb")))


        print("There are {} raw files, {} icp files and {} rsc files".format(len(raw_files), len(icp_files), len(rsc_files)))
        
        #keys 
        print("keys of stats_icp: ", stats_icp[0].keys())

        if len(raw_files) ==0:
            print("No piclkles files found in the raw folder")
            return
    




        self.meanICPerror = []
        self.meanRSCerror = []
        self.meanRawError = []

        self.nb_correspondances_after_RAW = []
        self.nb_correspondances_after_ICP = []
        self.nb_correspondances_after_RSC = []

        self.nb_correspondances_afer_icp_A = []
        self.nb_correspondances_afer_icp_B = []
        self.tiles_number = []

        #pour chaque dictionnaire de stats_icp, on recupere la moyenne de la value correspondant a la clé 'Distances'
        for dic in stats_icp:
            #meanICPerror contient l'error icp pour chaque paire de tuiles
            self.meanICPerror.append(np.mean(dic['Distances']))
            self.nb_correspondances_after_ICP.append(dic['Number corr a'] + dic['Number corr b'])
            self.nb_correspondances_afer_icp_A.append(dic['Number corr a'])
            self.nb_correspondances_afer_icp_B.append(dic['Number corr b'])
            
            pair_tuiles = dic['Tile number'] #['a13', 'b13']
            tile_number = int(pair_tuiles[1][1:])
            self.tiles_number.append(tile_number)

        for dic in stats_rsc:
            self.meanRSCerror.append(np.mean(dic['Distances']))
            self.nb_correspondances_after_RSC.append(dic['Number corr a'] + dic['Number corr b'])

        for dic in stats_raw:
            self.meanRawError.append(np.mean(dic['Distances']))
            self.nb_correspondances_after_RAW.append(dic['Number corr a'] + dic['Number corr b'])



    def plot_matrice_nb_correspondances(self):
        """
        print une matrice 2D. Abscisse Tuile id 1, ordonnée Tuile id 2, valeur nombre de correspondances
        """

        matrice = np.zeros((len(self.tiles_number), len(self.tiles_number)))

        #on mettra par exemple le nb de correspondances entre a13 et b13 dans la case (13, 13)

        for i in range(len(self.tiles_number)):
            for j in range(len(self.tiles_number)):
                if i==j:
                    matrice[i][j] = self.nb_correspondances_afer_icp_A[i] + self.nb_correspondances_afer_icp_B[i]
        
                else:
                    #couleur blanc si pas de correspondances
                    matrice[i][j] = 0


        matrice2 = np.zeros((len(self.tiles_number), len(self.tiles_number)))

        #on remplit avec l'erreur moyenne entre les tuiles
        for i in range(len(self.tiles_number)):
            for j in range(len(self.tiles_number)):
                if i==j:
                    matrice2[i][j] = self.meanICPerror[i]

                else:
                    #couleur blanc si pas de correspondances
                    matrice2[i][j] = 0


        

        #plot : coulor bar . 0 means no correspondances = white
        plt.figure()
        plt.subplot(121)
        plt.imshow(matrice, cmap='hot', interpolation='nearest')
        plt.colorbar()
        plt.title('Number of correspondances between tiles', fontsize=25)
        plt.xlabel('Tile id 1', fontsize=15)
        plt.ylabel('Tile id 2', fontsize=15)
        
        plt.subplot(122)
        plt.imshow(matrice2, cmap='hot', interpolation='nearest')
        plt.colorbar()
        plt.title('Mean distance between correspondances', fontsize=25)
        plt.xlabel('Tile id 1', fontsize=15)
        plt.ylabel('Tile id 2', fontsize=15)
        plt.show()








    def plot_box_mean_distance_between_correspondances(self):

        #plot via plt.boxplot
        plt.figure()
        #grid
        plt.grid()
        plt.boxplot([self.meanRSCerror, self.meanICPerror], labels=['RANSAC', 'ICP'], showmeans=True, meanline=True, showfliers=False)
        plt.title('Mean distance between correspondances', fontsize=25)
        #affiche la moyenne sur chaque boxplot
        plt.xlabel('Filter step', fontsize=15)
        plt.ylabel('Mean distance meters', fontsize=15)

        #afficher la moyenne de meanRSCerror et meanICPerror sur la boite
        plt.legend(['RANSAC: {:.4f}'.format(np.mean(self.meanRSCerror)), 'ICP: {:.4f}'.format(np.mean(self.meanICPerror))], fontsize=25)
        plt.show()


        #plot le CDF de meanICPerror et meanRSCerror avec y : erreur et x abscisse le cdf
        plt.figure()
        plt.grid()
        #histogramme cumulé avec ordonnée l'erreur et abscisse le cdf
        plt.hist(self.meanICPerror, bins=100, cumulative=True, histtype='step', density=True)
        plt.hist(self.meanRSCerror, bins=100, cumulative=True, histtype='step', density=True)
        plt.title('CDF of mean distance between correspondances', fontsize=25)
        plt.xlabel('Mean distance meters', fontsize=15)
        plt.ylabel('CDF', fontsize=15)
        plt.legend(['ICP', 'RANSAC'], fontsize=25)
        plt.show()




    def visualize_nb_correspondances(self):
        
        """
        use a color bar to show the number of correspondances after icp
        Utilise une matrice de couleur pour afficher le nombre de correspondances après ICP
        """

        #æffixe le nombre de correspondances après ICP
        plt.figure()
        plt.grid()
        plt.boxplot([ self.nb_correspondances_after_ICP], labels=['ICP'], showmeans=True, showfliers=False)
        plt.title('Number of correspondances after ICP', fontsize=25)
        plt.xlabel('Filter step', fontsize=15)
        plt.ylabel('Number of correspondances', fontsize=15)
        plt.legend(['ICP: {:.2f}'.format(np.mean(self.nb_correspondances_after_ICP))], fontsize=25)
        plt.show()



        





if __name__=="__main__":
    stat = Stats(PATH)
    stat.plot_box_mean_distance_between_correspondances()
    stat.visualize_nb_correspondances()
    stat.plot_matrice_nb_correspondances()
