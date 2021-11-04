import numpy as np
from Zmienne import *

def pitagoras(pos1,pos2): #zwraca odleglosc miedzy punktami
    x1,y1,z1=pos1
    x2,y2,z2=pos2
    return np.sqrt(((x2-x1)**2)+((y2-y1)**2)+((z2-z1)**2))

def pitagoras_pos_and_dist(pos1,dist,fpos): #return next position
    pass

def pitagoras_give_pos_on_dist(pos1,fpos,break_dist): #return  remaining distance, and time for remaining distance
    pass
    #return r_dist, r_time

def count_velocity(d:Drone,time):
    if d.velocity==drones_max_velocity:
        return drones_max_velocity
    else:
        v=drone_acceleration*time
        if v>drones_max_velocity:
            return drones_max_velocity # if this case correct the distance
        else:
            return v



def position_change_in_time_for_one_change(list_of_allocation_two_rows,multipoints): #do rozszerzenia lista punktów posrednich (dla trajektorii nieliniowej)
    list_of_positions_in_time=[]#Każda lista w srodku to kolejne pozycje drona w kolejnej turze (sampling time)
    if multipoints==False:
        for d in range(len(list_of_allocation_two_rows[0])):
            first_pos=list_of_allocation_two_rows[0][i]
            second_pos=list_of_allocation_two_rows[1][i]
            #tutaj dodać do rozszerzenia punkty posrednie
            traj=[first_pos,second_pos]
            d=drones[i]
            drone_points=[]
            time=0
            for j in range(len(traj)-1):
                d.change_final_destination(traj[j+1])
                no_end=True
                while no_end:
                    time=time+sampling_time
                    
                    if d.velocity==drones_max_velocity: #dron leci z jednostajną predkoscia
                        distance=(d.velocity*sampling_time)
                        if pitagoras(d.position,d.final_position)-distance<drone_breaking_distance:
                            r_dist, r_time=pitagoras_give_pos_on_dist(d.position, d.final_position, drone_breaking_distance)
                            distance=(d.velocity*r_time)+((d.velocity*(sampling_time-r_time))+(drone_break_acceleration*((sampling_time-r_time)**2))/2)
                            d.change_break(True)
                    
                    #DODAĆ VELOCITY!!!!!!!!!!
                    
                    else:
                        
                        if d.breaking==False: #dron przyspiesza
                            if count_velocity(d,time)==drones_max_velocity:    #dron leci z przyspieszeniem (+ lub -)
                                total_ac_time=drones_max_velocity/drone_acceleration
                                ac_time=total_ac_time-time
                                if ac_time>sampling_time:
                                    raise ValueError("Fail in counting")
                                f_time=sampling_time-ac_time
                                distance=((d.velocity*ac_time)+(drone_acceleration*(ac_time**2))/2) +(drones_max_velocity*f_time)
                            else:
                                distance=(d.velocity*sampling_time)+(drone_acceleration*(sampling_time**2))/2   
                        elif d.breaking==True: #dron hamuje
                            distance=(d.velocity*sampling_time)-(drone_break_acceleration*(sampling_time**2))/2  
                            if distance>pitagoras(d.position,d.final_position):
                                distance=pitagoras(d.position,d.final_position)
                                no_end=False
                    
                                
                        
                    
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
