from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle

dynamicObst1Dict = {
    "type": "sphere",
    "geometry": {"trajectory": ["5.0 - 0.4 * t", "5.0", "0.1"], "radius": 0.5},
    "movable": False,
}

dynamicObst2Dict = {
    "type": "sphere",
    "geometry": {"trajectory": ["-5.0 + 0.4 * t", "-5.", "0.1"], "radius": 0.5},
    "movable": False,
}

dynamicSphereObst1 = DynamicSphereObstacle(
    name="dynamicSphere1", content_dict=dynamicObst1Dict)

dynamicSphereObst2 = DynamicSphereObstacle(
    name="dynamicSphere2", content_dict=dynamicObst2Dict)