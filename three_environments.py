import math
import numpy as np
import bisect
import sys
import pathlib
import matplotlib.pyplot as plt


def build_environment(env_id):
    
    # the easy environment
    if env_id == 0:
        
        obstacleList = [(10,10,2), (15,24,2), (26,15,2)]
        
        return obstacleList
    
    # the medium environment
    elif env_id ==1:
        
        obstacleList = []
        
        for i in range(-1,12):
            obstacle = (6,2*i,1)
            obstacleList.append(obstacle)
        
        for i in range(3,17):
            obstacle = (20,2*i,1)
            obstacleList.append(obstacle)    
        
        return obstacleList
    
    # the hard environment
    elif env_id == 2:
        
        obstacleList = []
        
        
        
        return obstacleList
    else:
        print(f'Please choose between environments 0 (easy), 1 (medium), 2 (hard)!')
        
        
