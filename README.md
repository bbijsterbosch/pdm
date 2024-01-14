# Path planning and following in a dynamic environment with kinematic bicycle model constraints

This repository contains a framework for global and local path planning with static and dynamical obstacles.

**Authors:** Bas Bijsterbosch, Wimer Leijendekker, Tjeerd Wams, Jules Zwanen.

## File structure
├─ 📁 **environment** contains the descriptions of the three different environments and its obstacles.	
│   ├─ ```dynamic_obstacle.py```  
│   ├─ ```goal.py```  
│   ├─ ```three_environments.py```  
│   └─ ```wall_of_spheres.py```  
├ 📁 **global_path_planner:** contains all global path planning algorithms, such as RRT, RRT*, Dubins path and cubic splines.   
│   ├─ ```cubic_spline_planner.py```  
│   ├─ ```dubins_path_planner.py```  
│   ├─ ```main_global_path_planner.py```  
│   ├─ ```RRT_dubins.py```  
│   ├─ ```rrt_star_dubins.py```  
│   ├─ ```rrt_star.py```  
│   └─ ```rrt.py```  
├ 📁 **local_path_planner:** contains the code for the MPC.  
│   └─ ```mpc.py```  
├ 📁 **utils** contains some utilization functions that perform (mathematical) operations.  
│   ├─ ```angle.py```  
│   └─ ```plot.py```  
└  ```main.py:``` run the this file to run  the project.  

## Project description
The objective is for the car to reach the end goal through a slalom road with obstacles next to it while two moving obstacles are crossing the road. The model is tested by first letting it plan in easier environments without the moving obstacles and increasing the difficulty of the environment by making the lanes smaller and adding more turns and eventually adding the dynamic obstacles.
The task of the robot is to reach an end goal by making a global path, then following this path and avoiding dynamic obstacles. The robot will do this under kinematic constraints of the bicycle model. 

For planning the trajectory of the path a sampling-based motion planning algorithm is chosen. The algorithm Rapidly exploring Random Trees (RRT) is used for its easy use in high dimensional space, adaptability to non-holonomic constraints and as it has many extensions for specific tasks. For this project RRT and RRT* are compared and RRT* is chosen for its optimality. The RRT* is implemented with a Dubins Path planner to assure feasibility with the kinematic constraints of the bicycle model. The RRT* algorithm combined with the Dubins path planner proves to find the best solutions according to running time and minimal path length. When the path is found a Model Predictive Controller (MPC) will guide the robot along the path and avoid the obstacles by creating control commands for the environment.

## Installation
1. Install packages:
First install the environment
```console
pip3 install urdfenvs
```
Install package
```console
pip install dccp
```
2. clone this repository:
```console
git clone git@github.com:bbijsterbosch/pdm.git
```

## Running instructions
Run the main
```console
cd pdm
python3 main.py
```
Note: For hiding the visuals of the global path planner set```animation = False``` in ```main.py```

> [!IMPORTANT]
> The local planner will start only when the user has closed the plots of the local planner.
