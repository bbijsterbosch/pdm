# Path planning and following in a dynamic environment with kinematic bicycle model constraints

This repository contains a framework for global and local path planning with static and dynamical obstacles.

**Authors:** Bas Bijsterbosch, Wimer Leijendekker, Tjeerd Wams, Jules Zwanen.

## File structure
├─ 📁 **environment** contains the descriptions of the three different environments and its obstacles.  
│   ├─ ```dynamic_obstacle.py```  
│   ├─ ```goal.py```  
│   ├─ ```three_environments.py```  
│   └─ ```wall_of_spheres.py```  
├ 📁 **global_path_planner:** contains all global path planning algorithms  
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
This repository is made as a TU Delft project for the course Planning & Decision making. The goal of the project is first use Rapidly exploring Random Tree's with extensions for global path planning and tracking this path using Model Predictive Control and to avoid dynamic obstacles. This is done in three different environments to measure performance.

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
