import numpy as np
import numpy.random as nprnd
from Zmienne import *
from Symulacja import *

def fc(positions_in_time):
    colis=colision_detection(positions_in_time)
    fc=len(positions_in_time[0])+(1000*len(colis))
    return fc

def is_in_tabu(tabu_list,comb):
    for k in range(len(tabu_list)):
        if tabu_list[k]==comb:
            return True
    return False

def algorithm(list_of_positions,type_of_algorithm): #1-p. krótkoterminowa, 2-p. długoterminowa
    if type_of_algorithm==1:
        it_list=position_change_in_time_for_one_change(list_of_positions)
        go_on=True
        k_tabu_list=[]
        Q_best=fc(it_list)
        result=list_of_positions
        iteration=0
        fc_list=[]
        while go_on:
            print(k_tabu_list)
            iteration+=1
            print(iteration)
            i=nprnd.randint(drones_amount)
            eq=True
            while eq:
                j=nprnd.randint(drones_amount)
                if j==i:
                    eq=True
                else:
                    eq=False
            if i>j:
                rem=i
                i=j
                j=rem
            if is_in_tabu(k_tabu_list, (i,j)): #aspiracja
                asp=list_of_positions
                i_p_a=asp[1][i]
                j_p_a=asp[1][j]
                asp[1][i]=j_p_a
                asp[1][j]=i_p_a
                it_list=position_change_in_time_for_one_change(list_of_positions)
                Q=fc(it_list)
                
                if Q_best>Q:
                    Q_best=Q
                    result=asp
                    list_of_positions=asp
                    k_tabu_list.append((i,j))
                    if len(k_tabu_list)==tabu_length+1:
                        del k_tabu_list[0]
            else:
                i_p=list_of_positions[1][i]
                j_p=list_of_positions[1][j]
                list_of_positions[1][i]=j_p
                list_of_positions[1][j]=i_p
                it_list=position_change_in_time_for_one_change(list_of_positions)
                Q=fc(it_list)
                if Q_best>Q:
                    Q_best=Q
                    result=list_of_positions
                    k_tabu_list.append((i,j))
                    if len(k_tabu_list)==tabu_length+1:
                        del k_tabu_list[0]
                else:
                    k_tabu_list.append((i,j))
                    if len(k_tabu_list)==tabu_length+1:
                        del k_tabu_list[0]
            if iteration==20:
                go_on=False
            print(i,j,"tu")
            fc_list.append(Q)
        return result, Q_best, k_tabu_list, fc_list
                    
    if type_of_algorithm==2:
        print("Waiting for implementation")
    

    
    pass


W=algorithm(positions,1)
Y=position_change_in_time_for_one_change(W[0])