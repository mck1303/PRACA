import numpy as np
from Zmienne import *
import copy

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

def count_velocity(d:Drone,time,drone_acceleration):
    if d.velocity==drones_max_velocity:
        return drones_max_velocity
    else:
        v=(drone_acceleration*sampling_time)+d.velocity
        if v>drones_max_velocity:
            return drones_max_velocity # if this case correct the distance
        else:
            return v

def xyzvector_easy(tra,value):
    x1,y1,z1=tra
    if x1!=0:
        if y1!=0 and z1!=0:
            x3=np.sqrt((value**2)/(1+((y1/x1)**2)+((z1/x1)**2)))
            y3=x3*y1/x1
            z3=x3*z1/x1
        elif y1==0:
            if z1!=0:
                x3=np.sqrt((value**2)/(1+((z1/x1)**2)))
                y3=0
                z3=x3*z1/x1
            else:
                x3=value
                z3=0
                y3=0
        else:
            x3=np.sqrt((value**2)/(1+((y1/x1)**2)))
            y3=x3*y1/x1
            z3=0
    elif y1!=0:
        if z1!=0:
            y3=np.sqrt((value**2)/(1+((z1/y1)**2)))
            x3=0
            z3=y3*z1/y1
        else:
            y3=value
            z3=0
            x3=0
    elif z1!=0:
        z3=value
        y3=0
        x3=0
    else:
        raise valueError("Trajectory 0,0,0")
    return x3, y3, z3

def count_D(d:Drone,velocity,Vx,Vy,Vz,x1,y1,z1):
    if Vx==0 and Vy==0 and Vz==0 and x_wind==0 and y_wind==0:
        return 0, 0,(drone_mass*gr) 
    
    if x1==0:
        Wx=x_wind
    elif x1==1:
        Wx=Vx-x_wind
    else:
        Wx=Vx+x_wind
    if y1==0:
        Wy=y_wind
    elif y1>0:
        Wy=Vy-y_wind
    else:
        Wy=Vy+y_wind
    Wz=Vz
    W=np.sqrt((Wx**2)+(Wy**2)+(Wz**2))
    drag=air_drag*(W**2)
    Fx,Fy,Fz=xyzvector_easy((Wx,Wy,Wz),drag)
    if z1==0:
        Fz=(drone_mass*gr)
    elif z1>0:
        Fz=Fz+(drone_mass*gr)
    else:
        Fz=Fz-(drone_mass*gr)
    return Fx, Fy, Fz
    
def count_acc(d:Drone,velocity):
    Xx, Xy, Xz=d.trajectory
    x1,y1,z1=d.xyz
    Vx, Vy, Vz=xyzvector_easy(d.trajectory, velocity)
    # if x1==0:
    #     Dx=air_drag*((x_wind)**2)
    # elif x1==1:
    #     Dx=air_drag*((Vx-x_wind)**2)
    # else:
    #     Dx=air_drag*((Vx+x_wind)**2)
    # if y1==0:
    #     Dy=air_drag*((y_wind)**2)
    # elif y1>0:
    #     Dy=air_drag*((Vy-y_wind)**2)
    # else:
    #     Dy=air_drag*((Vy+y_wind)**2)
    # if z1==0:
    #     Dz=(drone_mass*gr)
    # elif z1>0:
    #     Dz=(air_drag*(Vz**2))+(drone_mass*gr)
    # else:
    #     Dz=(air_drag*(Vz**2))-(drone_mass*gr)
    Dx,Dy,Dz=count_D(d,velocity,Vx,Vy,Vz,x1,y1,z1)    
    if x1!=0 and y1!=0 and z1!=0:
        a=Xy/Xx
        b=Xz/Xx
        Fx=np.roots([1+(a**2)+(b**2),(2*Dx)+(2*a*Dy)+(2*b*Dz),(Dx**2)+(Dy**2)+(Dz**2)-(drone_force**2)])
        
        Fy=Fx*a
        Fz=Fx*b
        F=np.sqrt((Fx**2)+(Fy**2)+(Fz**2))
        if type(F[0])==np.complex128:
            raise ValueError('Not enough force')
        return F[0]/drone_mass
    
    elif x1==0 and y1!=0 and z1!=0 or x1!=0 and y1==0 and z1!=0 or x1!=0 and y1!=0 and z1==0:
        if x1==0:
            variant=1
            l=Dy
            la=Xz/Xy
            m=Dz
            w=Dx
        if y1==0:
            variant=2
            l=Dx
            la=Xz/Xx
            m=Dz
            w=Dy
        if z1==0:
            variant=3
            l=Dx
            la=Xy/Xx
            m=Dy
            w=Dz
        Fl=np.roots([1+la**2,(2*l)+(2*la*m),(m**2)+(w**2)+(l**2)-(drone_force**2)])
        Fm=Fl*la
        
        if variant==1:
            Fx=0
            Fy=Fl
            Fz=Fm
        elif variant==2:
            Fx=Fl
            Fy=0
            Fz=Fm
        elif variant==3:
            Fx=Fl
            Fy=Fm
            Fz=0
            
        F=np.sqrt((Fx**2)+(Fy**2)+(Fz**2))
        if type(F[0])==np.complex128:
            raise ValueError('Not enough force')
        return F[0]/drone_mass
    else:
        if x1==0 and y1==0 and z1!=0:
            Fz=np.roots([1,2*Dz,(Dx**2)+(Dy**2)+(Dz**2)-(drone_force**2)])
            Fy=0
            Fx=0
        elif x1!=0 and y1==0 and z1==0:
            Fx=np.roots([1,2*Dx,(Dx**2)+(Dy**2)+(Dz**2)-(drone_force**2)])
            Fy=0
            Fz=0
        elif x1==0 and y1!=0 and z1==0:
            Fy=np.roots([1,2*Dy,(Dx**2)+(Dy**2)+(Dz**2)-(drone_force**2)])
            Fx=0
            Fz=0
        F=np.sqrt((Fx**2)+(Fy**2)+(Fz**2))
        if type(F[0])==np.complex128:
            raise ValueError('Not enough force')
        return F[0]/drone_mass
   
def count_bre(d:Drone,velocity):
    Xx, Xy, Xz=d.trajectory
    x1,y1,z1=d.xyz
    Vx, Vy, Vz=xyzvector_easy(d.trajectory, velocity)
    # if x1==0:
    #     Dx=air_drag*((x_wind)**2)
    # elif x1==1:
    #     Dx=air_drag*((Vx-x_wind)**2)
    # else:
    #     Dx=air_drag*((Vx+x_wind)**2)
    # if y1==0:
    #     Dy=air_drag*((y_wind)**2)
    # elif y1>0:
    #     Dy=air_drag*((Vy-y_wind)**2)
    # else:
    #     Dy=air_drag*((Vy+y_wind)**2)
    # if z1==0:
    #     Dz=(drone_mass*gr)
    # elif z1>0:
    #     Dz=(air_drag*(Vz**2))+(drone_mass*gr)
    # else:
    #     Dz=(air_drag*(Vz**2))-(drone_mass*gr)
    Dx,Dy,Dz=count_D(d,velocity,Vx,Vy,Vz,x1,y1,z1)
    if x1!=0 and y1!=0 and z1!=0:
        a=Xy/Xx
        b=Xz/Xx
        Fx=np.roots([1+(a**2)+(b**2),(2*Dx*(-1))-(2*a*Dy)-(2*b*Dz),(Dx**2)+(Dy**2)+(Dz**2)-(drone_force**2)])
        
        Fy=Fx*a
        Fz=Fx*b
        F=np.sqrt((Fx**2)+(Fy**2)+(Fz**2))
        if type(F[0])==np.complex128:
            raise ValueError('Not enough force')
        return F[0]/drone_mass
    
    elif x1==0 and y1!=0 and z1!=0 or x1!=0 and y1==0 and z1!=0 or x1!=0 and y1!=0 and z1==0:
        if x1==0:
            variant=1
            l=Dy
            la=Xz/Xy
            m=Dz
            w=Dx
        if y1==0:
            variant=2
            l=Dx
            la=Xz/Xx
            m=Dz
            w=Dy
        if z1==0:
            variant=3
            l=Dx
            la=Xy/Xx
            m=Dy
            w=Dz
        Fl=np.roots([1+la**2,(2*l*(-1))-(2*la*m),(m**2)+(w**2)+(l**2)-(drone_force**2)])
        Fm=Fl*la
        
        if variant==1:
            Fx=0
            Fy=Fl
            Fz=Fm
        elif variant==2:
            Fx=Fl
            Fy=0
            Fz=Fm
        elif variant==3:
            Fx=Fl
            Fy=Fm
            Fz=0
            
        F=np.sqrt((Fx**2)+(Fy**2)+(Fz**2))
        if type(F[0])==np.complex128:
            raise ValueError('Not enough force')
        return F[0]/drone_mass
    else:
        if x1==0 and y1==0 and z1!=0:
            Fz=np.roots([1,(-2)*Dz,(Dx**2)+(Dy**2)+(Dz**2)-(drone_force**2)])
            Fy=0
            Fx=0
        elif x1!=0 and y1==0 and z1==0:
            Fx=np.roots([1,(-2)*Dx,(Dx**2)+(Dy**2)+(Dz**2)-(drone_force**2)])
            Fy=0
            Fz=0
        elif x1==0 and y1!=0 and z1==0:
            Fy=np.roots([1,(-2)*Dy,(Dx**2)+(Dy**2)+(Dz**2)-(drone_force**2)])
            Fx=0
            Fz=0
        F=np.sqrt((Fx**2)+(Fy**2)+(Fz**2))
        if F[0]<0:
            raise ValueError("Wrong F")
        if type(F[0])==np.complex128:
            raise ValueError('Not enough force')
        return F[0]/drone_mass
   
def drone_bre_dist(d:Drone):
    go=True
    velocity=drones_max_velocity
    dist=0
    while go:
        a=count_bre(d,velocity)
        new_velocity=velocity-(a*sampling_time)
        if new_velocity<=0:
            go=False
            
        else:
            dist=dist+(velocity*sampling_time)-((a*(sampling_time**2))/2)
            velocity=new_velocity
    return dist



def position_change_in_time_for_one_change(list_of_allocation_two_rows,multipoints=multi): #do rozszerzenia lista punktów posrednich (dla trajektorii nieliniowej)
    list_of_positions_in_time=[]#Każda lista w srodku to kolejne pozycje drona w kolejnej turze (sampling time)
    velocities=[]
    if multipoints==False:
        for i in range(len(list_of_allocation_two_rows[0])):
            first_pos=list_of_allocation_two_rows[0][i]
            second_pos=list_of_allocation_two_rows[1][i]
            #tutaj dodać do rozszerzenia punkty posrednie
            traj=[first_pos,second_pos]
            #print(i)
            d=drones[i]
            drone_points=[]
            velos=[d.velocity]
            time=0
            for j in range(len(traj)-1):
                d.change_position(traj[j])
                d.change_final_destination(traj[j+1])
                drone_points.append(d.position)
                no_end=True
                no_max_velocity=False
                acc=False
                bre=False
                normal=False
                inner_pos=()
                drone_acceleration_distance=(drones_max_velocity**2)/(2*count_acc(d, 0)+count_acc(d, drones_max_velocity)/2)
                drone_acceleration=count_acc(d,d.velocity)
                drone_break_acceleration=count_bre(d,d.velocity)
                drone_breaking_distance=drone_bre_dist(d)#((drones_max_velocity**2)/(2*((count_bre(d,0)+count_bre(d,drones_max_velocity))/2)))#+(drones_max_velocity/(10))
                drone_max_breaking_time=drones_max_velocity/(2*((count_bre(d,0)+count_bre(d,drones_max_velocity))/2))
                #print(drone_break_acceleration)
                if (d.velocity*sampling_time)+(drone_acceleration*(sampling_time**2))/2 >= pitagoras(d.position,d.final_position):
                    no_end=False
                    #print("W1")
                    d.change_break(False)
                    distance=pitagoras(d.position,d.final_position)
                elif d.velocity==0 and no_max_velocity==False and normal==False:
                    if pitagoras(d.position,d.final_position)<drone_breaking_distance+drone_acceleration_distance:
                        no_max_velocity=True
                        #print("W2.1")
                        half_dist=pitagoras(d.position,d.final_position)*(drone_acceleration_distance/(drone_acceleration_distance+drone_breaking_distance))#(pitagoras(d.position,d.final_position)*2*drone_acceleration*drone_break_acceleration)/(2*drone_acceleration*(drone_acceleration+drone_break_acceleration))
                        inner_pos=pitagoras_pos_and_dist(d.position,half_dist,d.final_position)
                        #half_point_velocity=#np.sqrt((pitagoras(d.position,d.final_position)*2*drone_acceleration*drone_break_acceleration)/(drone_break_acceleration+drone_acceleration))
                        acc=True
                    else:
                        normal=True
                        #print("W2.2")
                while no_end:
                    time=time+sampling_time
                    drone_acceleration=count_acc(d,d.velocity)
                    drone_break_acceleration=count_bre(d,d.velocity)
                    #print(drone_acceleration,drone_break_acceleration)
                    
                    if no_max_velocity:
                        
                        #print("Z1")    
                        if acc:
                            distance=(d.velocity*sampling_time)+(drone_acceleration*(sampling_time**2))/2 
                            if distance<half_dist:
                                d.change_velocity(d.velocity+(drone_acceleration*sampling_time))
                                half_dist=half_dist-distance
                                #print("Z2")
                            else:
                                acc=False
                                bre=True
                                
                                distance_a=pitagoras(d.position,inner_pos)
                                time_a=np.roots([(drone_acceleration/2),d.velocity,-distance_a])[1]
                                if type(time_a)==np.complex128:
                                    raise ValueError('Wrong time')
                                time_b=sampling_time-time_a
                                half_point_velocity=d.velocity+(drone_acceleration*time_a)
                                distance_b=(half_point_velocity*time_b)-(drone_break_acceleration*(time_b**2))/2
                                
                                d.change_velocity(half_point_velocity-(drone_break_acceleration*time_b))
                                distance=distance_a + distance_b
                                #print("Z3")
                        elif bre:
                            distance=(d.velocity*sampling_time)-(drone_break_acceleration*(sampling_time**2))/2
                            if distance<0 or distance>pitagoras(d.position,d.final_position) or d.velocity-(drone_break_acceleration*sampling_time)<0:
                                if d.velocity-(drone_break_acceleration*sampling_time)<0:
                                    print("Warning: close enough",pitagoras(d.position,d.final_position))
                                distance=pitagoras(d.position,d.final_position)
                                d.change_velocity(0)
                                no_end=False
                                bre=False
                                d.change_break(False)
                                #print("Z4")                                         
                            else:
            
                                d.change_velocity(d.velocity-(drone_break_acceleration*sampling_time))
                                #print("Z5")
        
                            
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
                                    d.change_break(False)
                                    #print("Z6")
                                else:
                                    d.change_velocity(d.velocity-(drone_break_acceleration*(sampling_time-r_time)))
                                    #print("Z7")
                        else:
                            
                            if d.breaking==False: #dron przyspiesza
                        
                                if count_velocity(d,time,drone_acceleration)==drones_max_velocity:    
                                    ac_time=(drones_max_velocity-d.velocity)/drone_acceleration
                                    if ac_time>sampling_time:
                                        raise ValueError("Fail during the counting")
                                    f_time=sampling_time-ac_time
                                    distance=((d.velocity*ac_time)+(drone_acceleration*(ac_time**2))/2) +(drones_max_velocity*f_time)
                                    d.change_velocity(drones_max_velocity)
                                    #print("Z8")
                                else:
                                    distance=(d.velocity*sampling_time)+(drone_acceleration*(sampling_time**2))/2 
                                    d.change_velocity(d.velocity+(drone_acceleration*sampling_time))
                                    #print("Z9")
                                    
                            elif d.breaking==True: #dron hamuje
                                distance=(d.velocity*sampling_time)-(drone_break_acceleration*(sampling_time**2))/2
                                if distance>pitagoras(d.position,d.final_position) or distance<=0 or d.velocity-(drone_break_acceleration*sampling_time)<0:
                                    distance=pitagoras(d.position,d.final_position)
                                    d.change_velocity(0)
                                    no_end=False
                                    d.change_break(False)
                                    #print("Z10")
                                else:
                                    d.change_velocity(d.velocity-(drone_break_acceleration*sampling_time))
                                    #print("Z11")
                        
                    next_pos=pitagoras_pos_and_dist(d.position, distance, d.final_position)
                    drone_points.append(next_pos)
                    velos.append(d.velocity)
                    d.change_position(next_pos)
                    
                
            list_of_positions_in_time.append(drone_points)
            velocities.append(velos)
        return list_of_positions_in_time, velocities
	
    elif multipoints==True:
        print("Multipoints not implemented")
            
        
def simlulation(list_of_allocation,position_list):
    for part in list_of_allocation:
        pass
    pass

def colision_detection(list_of_positions1):
    list_of_positions=copy.deepcopy(list_of_positions1)
    maximum=max([len(i) for i in list_of_positions])
    mini=min([len(i) for i in list_of_positions])
    maxi=maximum
    list_of_positions
    for i in range(len(list_of_positions)):
        if len(list_of_positions[i])<maximum:
            rem=maximum-len(list_of_positions[i])
            for j in range (rem):
                list_of_positions[i].append(list_of_positions[i][-1])
    colisions=[]
    for i in range (len(list_of_positions[0])):
        for j in range(len(list_of_positions)):
            for k in range(1,len(list_of_positions)-j):
                x1,y1,z1=list_of_positions[j][i]
                x2,y2,z2=list_of_positions[j+k][i]
                if abs(x1-x2)<drones_safe_zone[0]/2 and abs(y1-y2)<drones_safe_zone[1]/2 and abs(z1-z2)<drones_safe_zone[2]/2:
                    colisions.append((j,list_of_positions[j][i],j+k,list_of_positions[j+k][i]))
    if time_full_or_diff==0:            
        return colisions, maxi
    elif time_full_or_diff==1:
        return colisions, maxi-mini
    else:
        print("Warning: time_full_or_diff")
        return colisions, maxi
def mean_dist(list_of_positions):
    list_of_positions=copy.deepcopy(list_of_positions)
    maximum=max([len(i) for i in list_of_positions])
    #print(max([len(i) for i in list_of_positions]),min([len(i) for i in list_of_positions]))
    maxi=maximum
    list_of_positions
    for i in range(len(list_of_positions)):
        if len(list_of_positions[i])<maximum:
            rem=maximum-len(list_of_positions[i])
            for j in range (rem):
                list_of_positions[i].append(list_of_positions[i][-1])
    mean=0
    added=0
    suma=0
    for i in range (len(list_of_positions[0])):
        for j in range(len(list_of_positions)):
            for k in range(1,len(list_of_positions)-j):
               suma=suma+ pitagoras(list_of_positions[j][i],list_of_positions[j+k][i])
               added=added+1
    return suma/added

# drones, positions=start()
# positions=[[(0,0,0)],[(0,0,12)]]
# W=position_change_in_time_for_one_change(positions)
#Z=colision_detection(W)

