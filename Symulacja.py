import numpy as np
from Zmienne import *

class Drone:
    def __init__(self,ID,position):
        self.ID=ID
        self.position=position

def position_change_in_time_for_one_change(list_of_allocation_two_rows): #do rozszerzenia lista punktów posrednich (dla trajektorii nieliniowej)
    list_of_positions_in_time=[]#Każda lista w srodku to kolejne pozycje drona w kolejnej sekundzie
    for d in range(len(list_of_allocation_two_rows[0])):
        first_pos=list_of_allocation_two_rows[0][i]
        second_pos=list_of_allocation_two_rows[1][i]
        traj=[first_pos,second_pos]
        
        #tutaj dodać do rozszerzenia punkty posrednie
        
        for p in range(len(traj)-1):
            
            #dodać obliczanie kolejnych punktów przez pitagorasa itp.
            pass
        
        
    pass     
        
def simlulation(list_of_allocation,position_list):
    for part in list_of_allocation:
        pass
    pass