import numpy as np
import numpy.random as nprnd

#Przestrzeń wykonywania działań [m]
max_x=100
max_y=100
max_z=100

#Drony
drones_amount=4
drones_velocity=0.5 #[m/s]
drones_list=[]

#Lista pozycji zajmowanych przez drony w poszczególnych etapach
#Ilosc kolumn równa liczbie dronów. Ilosc rzędów równa ilosci etapów
position_list=[]

def random_pos_list(number_of_drones,number_of_parts):
    rand_pos_list=[]
    for j in range(number_of_parts):
        part_pos_list=[]
        for i in range(number_of_drones):
            d=0
            while d==0:
                d=1
                
                part_pos_list.append((nprnd.randint(0,high=max_x),nprnd.randint(0,high=max_y),nprnd.randint(0,high=max_z)))
                x,y,z=part_pos_list[-1]
                for k in range(len(part_pos_list)-1):
                    x1,y1,z1=part_pos_list[k]
                    if x==x1 and y==y1 and z==z1:
                        d=0
        rand_pos_list.append(part_pos_list)
    return rand_pos_list
  
        

