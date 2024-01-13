from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from mpscenes.obstacles.urdf_obstacle import UrdfObstacle
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.cylinder_obstacle import CylinderObstacle





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

pos1 = [2.0, 2.0, 0.0]
pos2 = [1, -2, 0.0]
pos3 = [3, 0, 0.0]
radius = 0.4

sphere_list = [
    {
        "type": "sphere",
        "movable": False,
        "geometry": {"position": pos1, "radius": radius},
    },
    {
        "type": "sphere",
        "movable": False,
        "geometry": {"position": pos2, "radius": radius},
    },
    {
        "type": "sphere",
        "movable": False,
        "geometry": {"position": pos3, "radius": radius},
    }
]

sphere_list_export = [SphereObstacle(name=f"simpleSphere_{i}", content_dict=sphere_i) for i, sphere_i in enumerate(sphere_list)]
