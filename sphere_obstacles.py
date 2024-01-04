from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from mpscenes.obstacles.urdf_obstacle import UrdfObstacle
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.cylinder_obstacle import CylinderObstacle
import numpy as np
import os




# wall_length = 5
# wall_obstacles_dicts = [
#     {
#         'type': 'box', 
#          'geometry': {
#              'position': [-2, 0, 0], 'width': wall_length, 'height': 1, 'length': 0.1
#         }
        
#     },

#     {
#         'type': 'box', 
#          'geometry': {
#              'position': [1, -2, 0], 'width': 0.1, 'height': 1, 'length': 5
#         }
#     },

#     {
#         'type': 'box', 
#          'geometry': {
#              'position': [3, 1, 0], 'width': wall_length/2, 'height': 1, 'length': 0.1
#         }
#     }
# ]

# wall_obstacles_jules = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(wall_obstacles_dicts)]


sphere_list = [
    {
        "type": "sphere",
        "movable": False,
        "geometry": {"position": [2.0, -0.0, 0.0], "radius": 0.4},
    },
    {
        "type": "sphere",
        "movable": False,
        "geometry": {"position": [1, -2, 0.0], "radius": 0.4},
    },
    {
        "type": "sphere",
        "movable": False,
        "geometry": {"position": [3, 1, 0.0], "radius": 0.4},
    }
]



sphere_list_export = [SphereObstacle(name=f"simpleSphere_{i}", content_dict=sphere_i) for i, sphere_i in enumerate(sphere_list)]
