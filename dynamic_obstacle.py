from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle

dynamicObst1Dict = {
    "type": "sphere",
    "geometry": {"trajectory": ["17.0 - 0.4 * t", "15.0", "0.1"], "radius": 0.5},
    "movable": False,
}

dynamicObst2Dict = {
    "type": "sphere",
    "geometry": {"trajectory": ["6.0 + 0.4 * t", "6.0", "0.1"], "radius": 0.5},
    "movable": False,
}

dynamicSphereObst1 = DynamicSphereObstacle(
    name="simpleSphere", content_dict=dynamicObst1Dict)

dynamicSphereObst2 = DynamicSphereObstacle(
    name="simpleSphere", content_dict=dynamicObst2Dict)