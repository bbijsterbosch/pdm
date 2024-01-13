# Path planning and following in a dynamic environment with kinematic bicycle model constraints

This repository contains a framework for global and local path planning with static and dynamical obstacles.

**Authors:** Bas Bijsterbosch, Wimer Leijendekker, Tjeerd Wams, Jules Zwanen.

## File structure
- Environment: contains the descriptions of the three different environments and its obstacles.	
- global_path_planner: contains all global path planning algorithms, such as RRT, RRT*, Dubins path and cubic splines. 
- local_path_planner: contains the code for the MPC.
- utils: contains some utilization functions that perform (mathematical) operations.
- main.py: run the this file to run  the project.

## Project description
The robot is a car and its objective is to reach the end goal through a slalom road with obstacles next to it while two moving obstacles are crossing the road. The model is tested by first letting it plan in easier environments [IMAGE EASIER ENVIRONMENTS] without the moving obstacles and increasing the difficulty of the environment by making the lanes smaller and adding more turns and eventually adding the dynamic obstacles.
The task of the robot is to reach an end goal by making a global path, then following this path and avoiding dynamic obstacles. The robot will do this under kinematic constraints of the bicycle model. 
For planning the trajectory of the path a sampling-based motion planning algorithm is chosen. This has the advantage of being probabilistically complete and therefore the chance of finding a solution approaches 1 as the number of iterations approaches infinity. The algorithm Rapidly exploring Random Trees (RRT) is used for its easy use in high dimensional space, adaptability to non-holonomic constraints and as it has many extensions for specific tasks. [LAVALLE] For this project RRT and RRT* are compared and RRT* is chosen for its optimality [SOLOVEY]. The RRT* is implemented with a Dubins Path planner to assure feasibility with the kinematic constraints of the bicycle model. The RRT* algorithm combined with the Dubins path planner proves to find the best solutions according to running time and maximum . When the path is found an  Model Predictive Controller (MPC) will guide the robot along the path and avoid the obstacles by creating control commands for the environment.


## Installation and run instructions
1. Install packages:
```
pip install blablabla
pip install blabla
```
2. clone this repository:
```
git clone git@github.com:bbijsterbosch/pdm.git
```
3. Run the main
```
python3 main.py
```
