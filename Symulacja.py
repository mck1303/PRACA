import numpy as np
from Zmienne import *

def pitagoras(pos1,pos2): #zwraca odleglosc miedzy punktami
    x1,y1,z1=pos1
    x2,y2,z2=pos2
    return np.sqrt(((x2-x1)**2)+((y2-y1)**2)+((z2-z1)**2))

def pitagoras_pos_and_dist(pos1,dist,fpos): #return next position, check if it is final one
    pass

def count_velocity(d:Drone,time):
    if d.velocity==drones_max_velocity:
        pass
    else:
        v=drone_acceleration*time
        if v>drones_max_velocity:
            d.change_velocity(drones_max_velocity) # if this case correct the distance
        else:
            d.change_velocity(v)

def position_change_in_time_for_one_change(list_of_allocation_two_rows,multipoints): #do rozszerzenia lista punktów posrednich (dla trajektorii nieliniowej)
    list_of_positions_in_time=[]#Każda lista w srodku to kolejne pozycje drona w kolejnej turze (sampling time)
    if multipoints==False:
        for d in range(len(list_of_allocation_two_rows[0])):
            first_pos=list_of_allocation_two_rows[0][i]
            second_pos=list_of_allocation_two_rows[1][i]
            #tutaj dodać do rozszerzenia punkty posrednie
            traj=[first_pos,second_pos]
            no_end=True
            d=drones[i]
            drone_points=[]
            time=0
            for j in range(len(traj)-1):
                d.change_final_destination(traj[j+1])
                while no_end:
                    time=time+sampling_time
                    distance=(d.velocity*time)+(drone_acceleration*(time**2))/2
                    next_pos=pitagoras_pos_and_dist(d.position, distance, d.final_position)
                    drone_points.append(next_pos)
                    d.change_position(next_pos)
                    count_velocity(d,time)
                
            list_of_positions_in_time.append(drone_points)
        return list_of_positions_in_time
	
    elif multipoints==True:
        print("Multipoints not implemented")
        
    pass     
        
def simlulation(list_of_allocation,position_list):
    for part in list_of_allocation:
        pass
    pass
