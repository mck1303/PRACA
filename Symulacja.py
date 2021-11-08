import numpy as np
from Zmienne import *

def pitagoras(pos1,pos2): #zwraca odleglosc miedzy punktami
    x1,y1,z1=pos1
    x2,y2,z2=pos2
    return np.sqrt(((x2-x1)**2)+((y2-y1)**2)+((z2-z1)**2))

def pitagoras_pos_and_dist(pos1,dist,fpos): #return next position
    x1,y1,z1=pos1
    x2,y2,z2=fpos
    full_dist=np.sqrt((abs(x1-x2)**2)+(abs(y1-y2)**2)+(abs(z1-z2)**2))
    x_dist=abs(x1-x2)
    y_dist=abs(y1-y2)
    z_dist=abs(z1-z2)
    x3_dist=(dist*x_dist)/full_dist
    y3_dist=(dist*y_dist)/full_dist
    z3_dist=(dist*z_dist)/full_dist
    if x1<=x2:
        x3=x1+x3_dist
    else:
        x3=x1-x3_dist
    if y1<=y2:
        y3=y1+y3_dist
    else:
        y3=y1-y3_dist
    if z1<=z2:
        z3=z1+z3_dist
    else:
        z3=z1-z3_dist
    return (x3,y3,z3)

def pitagoras_give_pos_on_dist(pos1,fpos,break_dist): #return  remaining distance, and time for remaining distance
    x1,y1,z1=pos1
    x2,y2,z2=fpos
    full_dist=np.sqrt((abs(x1-x2)**2)+(abs(y1-y2)**2)+(abs(z1-z2)**2))
    r_dist=full_dist-break_dist
    r_time=r_dist/drones_max_velocity
    return r_dist, r_time

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
        for i in range(len(list_of_allocation_two_rows[0])):
            first_pos=list_of_allocation_two_rows[0][i]
            second_pos=list_of_allocation_two_rows[1][i]
            #tutaj dodać do rozszerzenia punkty posrednie
            traj=[first_pos,second_pos]
            d=drones[i]
            drone_points=[]
            time=0
            for j in range(len(traj)-1):
                d.change_position(traj[j])
                d.change_final_destination(traj[j+1])
                no_end=True
                no_max_velocity=False
                acc=False
                bre=False
                normal=False
                inner_pos=()
                while no_end:
                    time=time+sampling_time
                    
                    if (d.velocity*sampling_time)+(drone_acceleration*(sampling_time**2))/2 >= pitagoras(d.position,d.final_position):
                        no_end=False
                        distance=pitagoras(d.position,d.final_position)
                    elif d.velocity==0 and no_max_velocity==False and normal==False:
                        if pitagoras(d.position,d.final_position)<drone_breaking_distance*2:
                            no_max_velocity=True
                            half_dist=pitagoras(d.position,d.final_position)/2
                            inner_pos=pitagoras_pos_and_dist(d.position,half_dist,d.final_position)
                            half_point_velocity=drone_acceleration*np.sqrt((2*half_dist)/drone_acceleration)
                            acc=True
                        else:
                            normal=True
                    elif no_max_velocity:
                            
                        if acc:
                            distance=(d.velocity*sampling_time)+(drone_acceleration*(sampling_time**2))/2 
                            if distance<half_dist:
                                d.change_velocity(d.velocity+(drone_acceleration*sampling_time))
                            else:
                                acc=False
                                bre=True
                                distance_a=pitagoras(d.position,inner_pos)
                                time_a=((half_point_velocity-d.velocity)/drone_acceleration)
                                time_b=sampling_time-time_a
                                distance_b=(half_point_velocity*time_b)-(drone_break_acceleration*(time_b**2))/2
                                d.change_velocity(half_point_velocity-(drone_break_acceleration*time_b))
                        elif bre:
                            distance=(d.velocity*sampling_time)-(drone_break_acceleration*(sampling_time**2))/2
                            if distance<0 or distance>pitagoras(d.position,d.final_position):
                                distance=pitagoras(d.position,d.final_position)
                                d.change_velocity(0)
                                no_end=False
                                bre=False
                                
                            else:
                                d.change_velocity(d.velocity-(drone_break_acceleration*sampling_time))
        
                            
                    if normal:
                        if d.velocity==drones_max_velocity: #dron leci z jednostajną predkoscia
                            distance=(d.velocity*sampling_time)
                            if pitagoras(d.position,d.final_position)-distance<drone_breaking_distance:
                                
                                r_dist, r_time=pitagoras_give_pos_on_dist(d.position, d.final_position, drone_breaking_distance)
                                distance=(d.velocity*r_time)+((d.velocity*(sampling_time-r_time))-(drone_break_acceleration*((sampling_time-r_time)**2))/2)
                                d.change_break(True)
                                if distance>pitagoras(d.position,d.final_position)or distance<=0 or drone_max_breaking_time<sampling_time-r_time:
                                    distance=pitagoras(d.position,d.final_position)
                                    d.change_velocity(0)
                                    no_end=False
                                
                                else:
                                    d.change_velocity(d.velocity-(drone_break_acceleration*(sampling_time-r_time)))
                        
                        else:
                            
                            if d.breaking==False: #dron przyspiesza
                        
                                if count_velocity(d,time)==drones_max_velocity:    
                                    ac_time=(drones_max_velocity-d.velocity)/drone_acceleration
                                    if ac_time>sampling_time:
                                        raise ValueError("Fail during the counting")
                                    f_time=sampling_time-ac_time
                                    distance=((d.velocity*ac_time)+(drone_acceleration*(ac_time**2))/2) +(drones_max_velocity*f_time)
                                    d.change_velocity(drones_max_velocity)
                                else:
                                    distance=(d.velocity*sampling_time)+(drone_acceleration*(sampling_time**2))/2 
                                    d.change_velocity(d.velocity+(drone_acceleration*sampling_time))
                                    
                            elif d.breaking==True: #dron hamuje
                                distance=(d.velocity*sampling_time)-(drone_break_acceleration*(sampling_time**2))/2
                                if distance>pitagoras(d.position,d.final_position)-max_error_value or distance<=0 or drone_max_breaking_time<sampling_time-r_time:
                                    distance=pitagoras(d.position,d.final_position)
                                    d.change_velocity(0)
                                    no_end=False
                                else:
                                    d.change_velocity(d.velocity-(drone_break_acceleration*sampling_time))
                                
                        
                    
                    next_pos=pitagoras_pos_and_dist(d.position, distance, d.final_position)
                    drone_points.append(next_pos)
                    d.change_position(next_pos)
                    
                
            list_of_positions_in_time.append(drone_points)
        return list_of_positions_in_time
	
    elif multipoints==True:
        print("Multipoints not implemented")
            
        
def simlulation(list_of_allocation,position_list):
    for part in list_of_allocation:
        pass
    pass

for i in range(100):
    drones=start()
    X=random_pos_list(drones_amount,1,start_type)
    W=position_change_in_time_for_one_change(X,False)
    for j in range(len(W)):
        if abs(X[1][j][0]-W[j][-1][0])>0.1:
            print("Error ",X[1][j]," ",W[j][-1]," ",X[0][j])