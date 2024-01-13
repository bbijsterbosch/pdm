import math
import numpy as np
import bisect
import sys
import pathlib
import matplotlib.pyplot as plt

from real_enviroment.wall_of_spheres import create_sphere
from mpscenes.obstacles.sphere_obstacle import SphereObstacle


def build_environment(env_id):
    
    # the easy environment
    if env_id == 0:
        
        obstacleList = [(10,10,2), (15,24,2), (26,15,2)]
        
        return obstacleList
    
    # the medium environment
    elif env_id ==1:
        
        obstacleList = []
        
        for i in range(-17,8):
            obstacle = (-5, i, 0.5)
            obstacleList.append(obstacle)
        
        for i in range(-8,18):
            obstacle = (5, i, 0.5)
            obstacleList.append(obstacle)    
        
        return obstacleList
    
    # the hard environment
    elif env_id == 2:
        
        obstacleList = []
        
        for i in range(-1,7):
            obstacle = (7,2*i,1)
            obstacleList.append(obstacle)
        
        for i in range(3,11):
            obstacle = (20,2*i,1)
            obstacleList.append(obstacle)    
            
        for i in range(-1,11):
            obstacle = (2*i, 22, 1)
            obstacleList.append(obstacle)
        
        
        
        return obstacleList
    else:
        raise Exception("Please choose between environments 0 (easy), 1 (medium), 2 (hard)!")
        
        
def circles_to_spheres(obstacle_list):
    sphere_list = []
    for circle in obstacle_list:
        center_x, center_y, radius = circle  # Ignore the circle's radius
        sphere = create_sphere([center_x, center_y, 0], radius)  # Assuming z=0 for 2D visualization
        sphere_list.append(sphere)
        
        sphere_list_export = [SphereObstacle(name=f"simpleSphere_{i}", content_dict=sphere_i) for i, sphere_i in enumerate(sphere_list)]
        
    return sphere_list_export

# Example usage:
# environment = build_environment(1)  # Choose the environment (0 for easy, 1 for medium, 2 for hard)
# converted_spheres = circles_to_spheres(environment, radius=0.4)  # Convert circles to spheres with a radius of 0.4

