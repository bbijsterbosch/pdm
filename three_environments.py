import math
import numpy as np
import bisect
import sys
import pathlib
import matplotlib.pyplot as plt

def build_environment(env_id):
    
    if env_id == 0:
        
        obstacleList = []
        
        
        return obstacleList
    elif env_id ==1:
        
        obstacleList = [(4,5,1),
                (4,1,1),
                (4,3,1), 
                (4,7,1) , 
                (4,-1,1),
                (4,-3,1),
                (0,14,1),
                (2,14,1),
                (4,14,1),
                (6,14,1),
                (8,14,1),
                (10,14,1),
                (12,14,1),
                (14,14,1),
                (16,14,1),
                (10,12,1),
                (10,10,1),
                (10,8,1),
                (10,6,1),
                (10,4,1),
                ]
        
        
        
        return obstacleList
    elif env_id == 2:
        
        obstacleList = []
        
        
        
        return obstacleList
    else:
        print(f'Please choose between environments 0 (easy), 1 (medium), 2 (hard)!')