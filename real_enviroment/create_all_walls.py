from real_enviroment.wall_of_spheres import (wall_of_n_spheres_x, wall_of_n_spheres_y, wall_of_n_spheres_x_right_to_point, wall_of_n_spheres_y_right_to_point)
from mpscenes.obstacles.sphere_obstacle import SphereObstacle


wall_x_1_list = wall_of_n_spheres_x([-4, 2, 0], 0.4, 10)
wall_x_2_list = wall_of_n_spheres_x([-4, -2, 0], 0.4, 10)
    
wall_y_list = wall_of_n_spheres_y([0, -4, 0], 0.4, 3)
wall_y_2_list = wall_of_n_spheres_y([3, -4, 0], 0.4, 3)
wall_y_3_list = wall_of_n_spheres_y([3, 0, 0], 0.4, 5)

    
wall_y_5_list = wall_of_n_spheres_y([3, 9, 0], 0.4, 6)

wall_x_3_list = wall_of_n_spheres_x([6, 2, 0], 0.4, 6)
wall_x_4_list = wall_of_n_spheres_x([6, 6, 0], 0.4, 6)

wall_y_6_list = wall_of_n_spheres_y_right_to_point([0, 2, 0], 16, 0.4)

total_walls = (wall_x_1_list + wall_x_3_list + wall_x_2_list + wall_y_list + wall_y_2_list + wall_y_3_list + wall_x_4_list  + wall_y_5_list
    + wall_y_6_list)

sphere_list_export = [SphereObstacle(name=f"simpleSphere_{i}", content_dict=sphere_i) for i, sphere_i in enumerate(total_walls)]