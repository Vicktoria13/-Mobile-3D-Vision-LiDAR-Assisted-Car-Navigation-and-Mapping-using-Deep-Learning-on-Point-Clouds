import numpy as np
import argparse
import matplotlib.pyplot as plt

#/home/sdi-2023-01/Downloads/log.txt





def plot_densite_error(densiteA, densiteB, value_error_icp):
    """
    plot la densiteA et la densiteB en fonction de value_error_icp
    """ 

    plt.figure()
    plt.scatter(value_error_icp, densiteA, label='densiteA')
    plt.scatter(value_error_icp, densiteB, label='densiteB')
    plt.xlabel('value_error_icp')
    plt.ylabel('densite')
    plt.legend()
    plt.show()







def main():
    parser = argparse.ArgumentParser(description='Compute pipeline statistics')
    parser.add_argument('log_file', type=str, help='Path to the log file')

    args = parser.parse_args()

    log_file = args.log_file
    print("log_file: ", log_file)

    ### The file is structured as follows:
    # id_number id_number value_error_icp nb_correspondancesA nb_correspondancesB densiteA densiteB time
    #avec id_number  = a45
    #on recuperere les valeurs de chaque colonne. Attention, il y a aussi des str

    

    id,value_error_icp, densiteA, densiteB, time = [], [], [], [], []
    
    with open(log_file, 'r') as f:
        for line in f:
            tuile_id = line.split()[0]  #a45
            #on rajoute juste le 45
            id.append(int(tuile_id[1:]))
            line = line.split()
            value_error_icp.append(float(line[2]))
            densiteA.append(float(line[5]))
            densiteB.append(float(line[6]))
            time.append(float(line[7]))



    #################### plot
    plot_densite_error(densiteA, densiteB, value_error_icp)
    


if __name__ == '__main__':
    main()
