import numpy as np


PATH_TXT = "file/85268_M230905_MAR3_2_SMV_Q3_10cm_no_header.txt"
LENGTH_TIME = 120 #seconds : interval of time for DN needed ==> 1min30
FREQ = 1 #Hz
#only the first column is the time



def main():

    # Read the txt file
    data = np.loadtxt(PATH_TXT, delimiter=",")
    #only the first column is the time
    time = data[:,0]

    #ßtart time arrondi au sup
    start_time = int(np.ceil(time[0]))
    end_time = int(np.floor(time[-1]))

    print("Start time: ", start_time)
    print("End time: ", end_time)


    """
    #iterate throut the time to find the first interval of time with no outage, ie difference between two consecutive time should be less
    #than Te = 1/FREQ
    
    #return the tuple (start, end) of the interval of time with no outage that last LENGTH_TIME

    accumulated_time = 0
    delta_T = 1.05 #2 consecutive time should be less than delta_T
    begin_no_outage = 0.0
    end_no_outage = 0.0

    #trouver le 1er intervalle de temps qui dur LENGTH_TIME, tel que chaque intervalle de temps entre 2 temps consecutifs soit inferieur a delta_T
    for i in range(len(time)//2, len(time)-1):
        while accumulated_time < LENGTH_TIME:
            #si no outage
            if time[i+1] - time[i] < delta_T:
                if accumulated_time == 0:
                    begin_no_outage = time[i]

                accumulated_time += time[i+1] - time[i]
                end_no_outage = time[i+1]
                print("End time: ", end_no_outage, "accumulated time: ", accumulated_time)
                i += 1

            #si outage, on recommence
            else:
                accumulated_time = 0
                break

        if accumulated_time >= LENGTH_TIME:
            break
      
    print("Interval of time with no outage: ", begin_no_outage, end_no_outage)
    print("Duration of the interval of time with no outage: ", end_no_outage - begin_no_outage)

    #int
    begin_no_outage = int(begin_no_outage)
    end_no_outage = int(end_no_outage)

    print("Interval of time with no outage: ", begin_no_outage, end_no_outage)
    print("Duration of the interval of time with no outage: ", end_no_outage - begin_no_outage)


    """

if __name__ == "__main__":
    main()