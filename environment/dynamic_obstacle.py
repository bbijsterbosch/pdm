from mpscenes.obstacles.dynamic_sphere_obstacle import DynamicSphereObstacle
from mpscenes.obstacles.sphere_obstacle import SphereObstacle

dynamicObst1Dict = {
    "type": "sphere",
    "geometry": {"trajectory": ["-5.0 + 0.2 * t ", "5.0 - 0.2* t", "0.1"], "radius": 1},
    "movable": False,
}

dynamicObst2Dict = {
    "type": "sphere",
    "geometry": {"trajectory": ["-5.0 + 0.4 * t", "-5.", "0.1"], "radius": 0.5},
    "movable": False,
}
staticObst1Dict = {
        "type": "sphere",
        "movable": False,
        "geometry": {"position": [0.0, 0.0, 0.0], "radius": 1},
}

dynamicSphereObst1 = SphereObstacle(
    name="Sphere1", content_dict=staticObst1Dict)

dynamicSphereObst2 = DynamicSphereObstacle(
    name="dynamicSphere2", content_dict=dynamicObst1Dict)

