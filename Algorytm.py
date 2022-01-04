import numpy as np
import numpy.random as nprnd
import copy
import pandas as pd
from Zmienne import *
from Symulacja import *

name=input("Write the name of the test: ")

def fc(positions_in_time):
    colis, time=colision_detection(positions_in_time)
    mean=mean_dist(positions_in_time)
    fc=(a*time)+(b*len(colis))+(c*(1/mean))
    return fc

def fc_d(positions_in_time,d_tabu,it):
    colis, time=colision_detection(positions_in_time)
    mean=mean_dist(positions_in_time)
    if it==0: 
        it=1
    fc=(a*time)+(b*len(colis))+(c*mean)+((d_alfa*d_tabu)/it)
    return fc

def algorithm(list_of_positions,type_of_algorithm): #1-p. krótkoterminowa, 2-p. długoterminowa
    if type_of_algorithm==1:
        iteration=0
        it_list=position_change_in_time_for_one_change(list_of_positions)[0]
        go_on=True
        k_tabu_list=np.zeros((drones_amount,drones_amount))
        Q_best=fc(it_list)
        result=list_of_positions        
        fc_list=[]   
        posit=[]
        b_counter=0
        while go_on:
            iteration+=1
            print(iteration)
            better=False
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
                    it_list=position_change_in_time_for_one_change(inside_l)[0]
                    Q=fc(it_list)
                    if k_tabu_list[l][m]==0:  
                        if Q_min>Q:
                            j=m
                            Q_min=Q
                            
                            posit=copy.deepcopy(inside_l)
                    else:
                        if Q_tabu>Q*Wa:
                            j_tabu=m
                            Q_tabu=Q
                            posit_tabu=copy.deepcopy(inside_l)
                            
            if Q_best>=Q_min:
                if Q_best>Q_min:
                    better=True
                Q_best=copy.deepcopy(Q_min)
                result=copy.deepcopy(posit)
                list_of_positions=copy.deepcopy(posit)
                k_tabu_list[i][j]=tabu_length+1
                k_tabu_list[j][i]=tabu_length+1

                

                
            elif Q_best>=Q_tabu:
                if Q_best>Q_tabu:
                    better=True
                Q_best=copy.deepcopy(Q_tabu)
                result=copy.deepcopy(posit_tabu)
                list_of_positions=copy.deepcopy(posit)
                k_tabu_list[i][j_tabu]=tabu_length+1
                k_tabu_list[j_tabu][i]=tabu_length+1

    
            else:
                list_of_positions=copy.deepcopy(posit)
                k_tabu_list[i][j]=tabu_length+1
                k_tabu_list[j][i]=tabu_length+1

            for o in range(drones_amount):
                for p in range(drones_amount):
                    if k_tabu_list[o][p]>0:
                        k_tabu_list[o][p]-=1
            
            if better:
                b_counter=0
            else:
                b_counter+=1
            if b_counter>=drones_amount-1:
                go_on=False

            fc_list.append(Q_min)
        return result, Q_best, k_tabu_list, fc_list
                    
    if type_of_algorithm==2:
        it_list=position_change_in_time_for_one_change(list_of_positions)[0]
        go_on=True
        k_tabu_list=np.zeros((drones_amount,drones_amount))
        d_tabu_list=np.zeros((drones_amount,drones_amount))
        Q_best=fc_d(it_list,0,0)
        result=list_of_positions
        iteration=0
        fc_list=[]
        posit=[]
        b_counter=0
        while go_on:
            better=False
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
                    it_list=position_change_in_time_for_one_change(inside_l)[0]
                    Q=fc_d(it_list,d_tabu_list[l][m],iteration)
                    if k_tabu_list[l][m]==0:   
                        if Q_min>Q:
                            j=m
                            Q_min=Q
                            posit=copy.deepcopy(inside_l)
                    else:
                        if Q_tabu>Q*Wa:
                            j_tabu=m
                            Q_tabu=Q
                            posit_tabu=copy.deepcopy(inside_l)
                            
            if Q_best>=Q_min:
                if Q_best>Q_min:
                    better=True
                Q_best=copy.deepcopy(Q_min)
                result=copy.deepcopy(posit)
                list_of_positions=copy.deepcopy(posit)
                b_list=position_change_in_time_for_one_change(posit)[0]
                k_tabu_list[i][j]=tabu_length+1
                k_tabu_list[j][i]=tabu_length+1
                d_tabu_list[i][j]+=1
                d_tabu_list[j][i]+=1
                fc_list.append(Q_min)

                
            elif Q_best>=Q_tabu:
                if Q_best>Q_tabu:
                    better=True
                Q_best=copy.deepcopy(Q_tabu)
                result=copy.deepcopy(posit_tabu)
                list_of_positions=copy.deepcopy(posit)
                b_list=position_change_in_time_for_one_change(posit)[0]
                k_tabu_list[i][j_tabu]=tabu_length+1
                k_tabu_list[j_tabu][i]=tabu_length+1
                d_tabu_list[i][j_tabu]+=1
                d_tabu_list[j_tabu][i]+=1
                fc_list.append(Q_tabu)
                better=True
            else:
                list_of_positions=copy.deepcopy(posit)
                k_tabu_list[i][j]=tabu_length+1
                k_tabu_list[j][i]=tabu_length+1
                d_tabu_list[i][j]+=1
                d_tabu_list[j][i]+=1
                fc_list.append(Q_min)
            for o in range(drones_amount):
                for p in range(drones_amount):
                    if k_tabu_list[o][p]>0:
                        k_tabu_list[o][p]-=1
            if better:
                b_counter=0
            else:
                b_counter+=1
            if b_counter>=drones_amount-1:
                go_on=False

            
        return result, Q_best, k_tabu_list, fc_list, d_tabu_list
    

    
def ful_m(f_result):
    list_of_positions=copy.deepcopy(f_result)
    maximum=max([len(i) for i in list_of_positions])
    mini=min([len(i) for i in list_of_positions])
    list_of_positions
    for i in range(len(list_of_positions)):
        if len(list_of_positions[i])<maximum:
            rem=maximum-len(list_of_positions[i])
            for j in range (rem):
                list_of_positions[i].append(list_of_positions[i][-1])
    return list_of_positions

W=algorithm(positions,type_of_algorithm)
Y=position_change_in_time_for_one_change(W[0])
l=[]
for i in Y[0]:
    n=len(i)
    k=[]
    for j in range(n-2):
        k.append(pitagoras(i[j],i[j+1]))
    l.append(k)

f_result=ful_m(Y[0])
f_result_matrix=[]
for j in range(len(f_result[0])):
    little_m=[]
    for i in range(len(f_result)):
        little_m.append(f_result[i][j])  
    f_result_matrix.append(little_m)

matrix=pd.DataFrame(f_result_matrix)

matrix.to_csv(name +'.csv')

if type_of_algorithm==1:
    Al='KLT'
else:
    Al='KLT + DLT'

with open(name+'.txt','w') as f:
    f.write('Space: '+str(max_x)+', '+str(max_y)+', '+str(max_z))
    f.write('\n')
    f.write('Drones: '+str(drones_amount))
    f.write('\n')
    f.write('Max velocity: '+str(drones_max_velocity))
    f.write('\n')
    f.write('Drones safe zone: '+str(drones_safe_zone))
    f.write('\n')
    f.write('Sampling time: '+str(sampling_time))
    f.write('\n')
    f.write('Wind [x/y]: '+str(x_wind)+', '+str(y_wind))
    f.write('\n')
    f.write('Drone force: '+ str(drone_force))
    f.write('\n')
    f.write('Drone mass: '+str(drone_mass))
    f.write('\n')
    f.write('Algorithm type: '+Al)
    f.write('\n')
    f.write('Tabu length: '+str(tabu_length))
    f.write('\n')
    f.write('DLT Alpha: '+str(d_alfa))
    f.write('\n')
    f.write('Criteria [a,b,c]: '+str(a)+', '+str(b)+', '+str(c))
    f.write('\n')
    f.write('Start positions: '+str(positions[0]))
    f.write('\n')
    f.write('Final positions: '+str(positions[1]))
    f.write('\n')
    f.write('FC Value: '+str(W[1]))
    f.write('\n')
    f.write('FC List: '+str(W[3]))
    f.write('\n')
    f.write('Result: '+str(W[0]))
    f.write('\n')
    f.write('Trajectory:'+str(Y))
    