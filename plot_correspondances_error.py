import numpy
import matplotlib.pyplot as plt
import argparse


"""
python plot_correspondances_error.py --file /home/raphael/Documents/Stage/Code/3DMatch-pytorch/3DMatch-pytorch/outputs
"""

def main():

    #argparse
    parser = argparse.ArgumentParser(description='Plot correspondances error')
    parser.add_argument('file', type=str, help='file containing the error with correspondances')
    args = parser.parse_args()

    #every line of the file is
    #atile error nb_correspondancesA  nb_correspondancesB

    #on va sortir 3 plot :
    # - 1 plot tracant l'erreur en fonction du nombre de correspondances A+B
    # - 1 plot tracant l'erreur en fonction du nombre de correspondances A
    # - 1 plot tracant l'erreur en fonction du nombre de correspondances B

    ARRAY_ERROR = []
    ARRAY_CORRESPONDANCES_A = []
    ARRAY_CORRESPONDANCES_B = []
    id_array = []

    with open(args.file, 'r') as f:
        for line in f:
            line = line.split(' ')
            ARRAY_ERROR.append(float(line[1]))
            ARRAY_CORRESPONDANCES_A.append(int(line[2]))
            ARRAY_CORRESPONDANCES_B.append(int(line[3]))
            #pour recuperer l'ID de la tuile, on enleve le 1er caractere a
            id_tuile = line[0][1:]
            id_array.append(id_tuile)

    plt.figure(figsize=(10,15))

    #espacement entre les plots
    plt.subplots_adjust(hspace=0.5)
    #subplots
    plt.subplot(3, 1, 1)
    #with cross
    plt.plot(ARRAY_CORRESPONDANCES_A, ARRAY_ERROR, 'co')
    plt.title('Error en fonction du nombre de correspondances A')
    plt.xlabel('Nombre de correspondances A')
    plt.ylabel('Erreur apres ICP refinement')

    #a chaque point, lui associer un ID
    for i, txt in enumerate(id_array):
        plt.annotate(txt, (ARRAY_CORRESPONDANCES_A[i], ARRAY_ERROR[i]))


    plt.subplot(3, 1, 2)
    plt.plot(ARRAY_CORRESPONDANCES_B, ARRAY_ERROR, 'ro')
    plt.title('Error en fonction du nombre de correspondances B')
    plt.xlabel('Nombre de correspondances B')
    plt.ylabel('Erreur')

    #associer a chaque ratio un ID
    for i, txt in enumerate(id_array):
        plt.annotate(txt, (ARRAY_CORRESPONDANCES_A[i], ARRAY_ERROR[i]))



    plt.subplot(3, 1, 3)
    plt.plot(numpy.add(ARRAY_CORRESPONDANCES_A, ARRAY_CORRESPONDANCES_B), ARRAY_ERROR, 'ro')
    plt.title('Error en fonction du nombre de correspondances A+B')
    plt.xlabel('Nombre de correspondances A+B')
    plt.ylabel('Erreur')

    for i, txt in enumerate(id_array):
        plt.annotate(txt, (ARRAY_CORRESPONDANCES_A[i], ARRAY_ERROR[i]))


    plt.show()

    x = numpy.arange(len(id_array))
        ##affichage ratio correspondances/erreur
    plt.figure(figsize=(10,15))
    plt.subplots_adjust(hspace=0.5)
    plt.subplot(3, 1, 1)
    ratio = numpy.divide(ARRAY_ERROR, ARRAY_CORRESPONDANCES_A)
    plt.plot(x, ratio, 'co')
    #a chaque point de ce ratio, lui associer un ID

    #array de 0 1 ... nb_tuiles
    

    #a chaque point, lui associer un ID. 
    for i, txt in enumerate(id_array):
        plt.annotate(txt, (x[i], ratio[i]))

    

   

    plt.title('Ratio erreur/correspondances en fonction du nombre de correspondances A')
    plt.xlabel('Ntuile')
    plt.ylabel('Ratio erreur/correspondances')

  



    plt.subplot(3, 1, 2)
    plt.plot(numpy.divide(ARRAY_ERROR, ARRAY_CORRESPONDANCES_B), 'ro')
    plt.title('Ratio erreur/correspondances en fonction du nombre de correspondances B')
    plt.xlabel('tuile')
    plt.ylabel('Ratio erreur/correspondances')


    plt.subplot(3, 1, 3)
    plt.plot(numpy.divide(ARRAY_ERROR, numpy.add(ARRAY_CORRESPONDANCES_A, ARRAY_CORRESPONDANCES_B)), 'ro')
    plt.title('Ratio erreur/correspondances en fonction du nombre de correspondances A+B')
    plt.xlabel('tuile')
    plt.ylabel('Ratio erreur/correspondances')

    for i, txt in enumerate(id_array):
        plt.annotate(txt, (ARRAY_CORRESPONDANCES_A[i], ARRAY_ERROR[i]))

    plt.show()


if __name__=="__main__":
    main()