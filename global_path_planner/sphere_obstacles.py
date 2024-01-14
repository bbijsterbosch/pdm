from mpscenes.obstacles.sphere_obstacle import SphereObstacle

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
