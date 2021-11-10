import numpy as np
import numpy.random as nprnd
from typing import Optional, List
#Przestrzeń wykonywania działań [m]
max_x=100
max_y=100
max_z=100

#Drony
drones_amount=4
drones_max_velocity=0.5 #[m/s]
drones_safe_zone=(1,1,2)#x,y,z[m] Przestrzeń zapewniająca dronom bezpieczne wzajemne mijanie się z innymi dronami (dron jest w centrum strefy)
drone_acceleration=0.25 #[m/s^2]
drone_break_acceleration=1 #[m/s^2]
drone_breaking_distance=(drones_max_velocity**2)/(2*drone_break_acceleration)
drone_max_breaking_time=drones_max_velocity/drone_break_acceleration
sampling_time=1 #[s]


#Symulacja
amount_of_parts=1
start_type=1 #starting from: 1-ground, 2- air
version=1 #version: 1-random, 2-prepared list
max_error_value=0.5

#Lista pozycji zajmowanych przez drony w poszczególnych etapach
#Ilosc kolumn równa liczbie dronów. Ilosc rzędów równa ilosci etapów

class Drone:
    def __init__(self,ID,position,final_position):
        self.ID=ID
        self.position=position
        self.final_position=final_position
        self.velocity=0
        self.breaking=False
        self.fly_type=0  #0- brak ruchu 1- przyspieszanie, 2- ruch jednostajny 3- hamowanie
        if self.position!=self.final_position:
            self.is_it_on_final_position=False
        else:
            self.is_it_on_final_position=True

    def change_final_destination(self,new_final_position):
        if new_final_position==self.final_position:
            print("New position is not new")
        else:
            if self.position!=new_final_position:
                self.final_position=new_final_position
                self.is_it_on_final_position=False
            else:
                self.final_position=new_final_position
                self.is_it_on_final_position=True
    def change_velocity(self,new_vel):
        if new_vel<0:
            print(self.velocity,new_vel)
            raise ValueError("Wrong velocity")
        else:
            self.velocity=new_vel
        
    def change_position(self,new_pos):
        self.position=new_pos
    
    def change_break(self,breaking):
        self.breaking=breaking
    
    
    
    
def create_drones(pos_list): #to create Drones
    list_of_drones=[]
    for i in range(drones_amount):
        list_of_drones.append(Drone(i,pos_list[0][i],pos_list[1][i]))
    return list_of_drones

def start(prepared_pos: Optional[List]=None): 
        if version==1:
            return create_drones(random_pos_list(drones_amount,amount_of_parts,start_type))
        elif version==2:
            return create_drones(prepared_pos)
        else:
            raise ValueError("Wrong version in start")
    
def start_positions(number_of_drones,type_of_start_position): #type_of_start_position: 1-ground, 2- air
    start_positions=[]
    if type_of_start_position==1:
        for i in range(number_of_drones):
            start_positions.append((nprnd.randint(0,high=max_x),nprnd.randint(0,high=max_y),0))
    elif type_of_start_position==2:
        for i in range(number_of_drones):
            start_positions.append((nprnd.randint(0,high=max_x),nprnd.randint(0,high=max_y),nprnd.randint(0,high=max_z)))
    else:
        raise ValueError("Wrong type_of_start_position")
    return start_positions


def random_pos_list(number_of_drones,number_of_parts,type_of_start_position): #fun to create random positions
    rand_pos_list=[]
    rand_pos_list.append(start_positions(number_of_drones,type_of_start_position))
    for j in range(number_of_parts):
        part_pos_list=[]
        for i in range(number_of_drones):
            d=0
            while d==0:
                d=1
                
                part_pos_list.append((nprnd.randint(1,high=max_x),nprnd.randint(1,high=max_y),nprnd.randint(1,high=max_z)))
                x,y,z=part_pos_list[-1]
                for k in range(len(part_pos_list)-1):
                    x1,y1,z1=part_pos_list[k]
                    if x==x1 and y==y1 and z==z1:
                        d=0
        rand_pos_list.append(part_pos_list)
    return rand_pos_list
  

drones=start()