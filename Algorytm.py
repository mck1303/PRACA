import numpy as np
import numpy.random as nprnd
from Zmienne import *
from Symulacja import *

def fc(positions_in_time):
    colis=colision_detection(positions_in_time)
    fc=len(positions_in_time[0])+(1000*len(colis))
    return fc

def fc_d(positions_in_time,d_tabu,it):
    colis=colision_detection(positions_in_time)
    if it==0: 
        it=1
    print((d_alfa*d_tabu)/it)
    fc=len(positions_in_time[0])+(1000*len(colis))+(d_alfa*d_tabu)/it
    return fc
        

def algorithm(list_of_positions,type_of_algorithm): #1-p. krótkoterminowa, 2-p. długoterminowa
    if type_of_algorithm==1:
        iteration=0
        it_list=position_change_in_time_for_one_change(list_of_positions)
        go_on=True
        k_tabu_list=np.zeros((drones_amount,drones_amount))
        Q_best=fc(it_list)
        result=list_of_positions        
        fc_list=[]   
        posit=[]
        while go_on:
            iteration+=1
            print(iteration)
            i=nprnd.randint(drones_amount)
            Q_min=np.Inf
            Q_tabu=np.Inf
            for k in range(drones_amount):
                if k!=i:
                    inside_l=list_of_positions
                    l=i
                    m=k
                    rem=inside_l[1][l]
                    inside_l[1][l]=inside_l[1][m]
                    inside_l[1][m]=rem
                    it_list=position_change_in_time_for_one_change(inside_l)
                    Q=fc(it_list)
                    if k_tabu_list[l][m]==0:   
                        if Q_min>Q:
                            j=m
                            Q_min=Q
                            posit=inside_l
                    else:
                        if Q_tabu>Q:
                            j_tabu=m
                            Q_tabu=Q
                            posit_tabu=inside_l
                            
            if Q_best>Q_min:
                Q_best=Q_min
                result=posit
                list_of_positions=posit
                k_tabu_list[i][j]=tabu_length+1
                k_tabu_list[j][i]=tabu_length+1
                print(k_tabu_list)

                
            elif Q_best>Q_tabu:
                Q_best=Q_tabu
                result=posit_tabu
                list_of_positions=posit
                k_tabu_list[i][j_tabu]=tabu_length+1
                k_tabu_list[j_tabu][i]=tabu_length+1
    
            else:
                list_of_positions=posit
                k_tabu_list[i][j]=tabu_length+1
                k_tabu_list[j][i]=tabu_length+1

            for o in range(drones_amount):
                for p in range(drones_amount):
                    if k_tabu_list[o][p]>0:
                        k_tabu_list[o][p]-=1
            if iteration==20:
                go_on=False

            fc_list.append(Q_min)
        return result, Q_best, k_tabu_list, fc_list, position_change_in_time_for_one_change(result)
                    
    if type_of_algorithm==2:
        it_list=position_change_in_time_for_one_change(list_of_positions)
        go_on=True
        k_tabu_list=np.zeros((drones_amount,drones_amount))
        d_tabu_list=np.zeros((drones_amount,drones_amount))
        Q_best=fc_d(it_list,0,0)
        result=list_of_positions
        iteration=0
        fc_list=[]
        posit=[]
        while go_on:
            iteration+=1
            print(iteration)
            i=nprnd.randint(drones_amount)
            Q_min=np.Inf
            Q_tabu=np.Inf
            for k in range(drones_amount):
                if k!=i:
                    inside_l=list_of_positions
                    l=i
                    m=k
                    rem=inside_l[1][l]
                    inside_l[1][l]=inside_l[1][m]
                    inside_l[1][m]=rem
                    it_list=position_change_in_time_for_one_change(inside_l)
                    Q=fc_d(it_list,d_tabu_list[l][m],iteration)
                    if k_tabu_list[l][m]==0:   
                        if Q_min>Q:
                            j=m
                            Q_min=Q
                            posit=inside_l
                    else:
                        if Q_tabu>Q:
                            j_tabu=m
                            Q_tabu=Q
                            posit_tabu=inside_l
                            
            if Q_best>Q_min:
                Q_best=Q_min
                result=posit
                list_of_positions=posit
                k_tabu_list[i][j]=tabu_length+1
                k_tabu_list[j][i]=tabu_length+1
                d_tabu_list[i][j]+=1
                d_tabu_list[j][i]+=1

                
            elif Q_best>Q_tabu:
                Q_best=Q_tabu
                result=posit_tabu
                list_of_positions=posit
                k_tabu_list[i][j_tabu]=tabu_length+1
                k_tabu_list[j_tabu][i]=tabu_length+1
                d_tabu_list[i][j_tabu]+=1
                d_tabu_list[j_tabu][i]+=1
  
            else:
                list_of_positions=posit
                k_tabu_list[i][j]=tabu_length+1
                k_tabu_list[j][i]=tabu_length+1
                d_tabu_list[i][j]+=1
                d_tabu_list[j][i]+=1
                print(k_tabu_list)
                
            for o in range(drones_amount):
                for p in range(drones_amount):
                    if k_tabu_list[o][p]>0:
                        k_tabu_list[o][p]-=1
            if iteration==20:
                go_on=False

            fc_list.append(Q_min)
        return result, Q_best, k_tabu_list, d_tabu_list, fc_list, position_change_in_time_for_one_change(result)
    

    
    


W=algorithm(positions,2)
Y=position_change_in_time_for_one_change(W[0])