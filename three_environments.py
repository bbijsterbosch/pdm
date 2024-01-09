import math
import numpy as np
import bisect
import sys
import pathlib
import matplotlib.pyplot as plt


def build_environment(env_id):
    
    if env_id == 0:
        
        obstacleList = [(4,12,2), (14,8,2), (15,24,2), (26,15,2)]
        
        
        return obstacleList
    elif env_id ==1:
        
        obstacleList = []
        
        for i in range(-1,16):
            obstacle = (2*i, 31, 1)
            obstacleList.append(obstacle)
        
        
        
        return obstacleList
    elif env_id == 2:
        
        obstacleList = []
        
        
        
        return obstacleList
    else:
        print(f'Please choose between environments 0 (easy), 1 (medium), 2 (hard)!')
        
        
