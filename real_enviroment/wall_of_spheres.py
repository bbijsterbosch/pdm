def wall_of_n_spheres_x(center_pos, radi, n):
    # Center_pos is an array [x_pos, y_pos, z_pos=0], for our application z_pos will be 0
    n_left = n // 2  # Use integer division for the left side
    n_right = n - n_left  # Subtract from n to ensure the total is correct


    #Create center_sphere
    center_sphere = create_sphere(center_pos, radi)
    # Create spheres left of X
    left_spheres = add_sphere_n_spheres_x_left(n_left, center_sphere)
    # Create spheres rifht of X
    right_spheres = add_sphere_n_spheres_x_right(n_right, center_sphere)
    # concatenate lists
    total_list = left_spheres + [center_sphere] + right_spheres 
    return total_list

def wall_of_n_spheres_y(center_pos, radi, n):
    # Center_pos is an array [x_pos, y_pos, z_pos=0], for our application z_pos will be 0
    n_left = round(n/2)
    n_right = round(n/2)

    #Create center_sphere
    center_sphere = create_sphere(center_pos, radi)
    # Create spheres left of X
    left_spheres = add_sphere_n_spheres_y_left(n_left, center_sphere)
    # Create spheres rifht of X
    right_spheres = add_sphere_n_spheres_y_right(n_right, center_sphere)
    # concatenate lists
    total_list = left_spheres + [center_sphere] + right_spheres 
    return total_list

def wall_of_n_spheres_x_right_to_point(begin_point, n, radi):
    #create begin_point
    enter_sphere = create_sphere(begin_point, radi)
    right_spheres = add_sphere_n_spheres_x_right(n - 1, enter_sphere)
    return [enter_sphere] + right_spheres


def wall_of_n_spheres_y_right_to_point(begin_point, n, radi):
    #create begin_point
    enter_sphere = create_sphere(begin_point, radi)
    right_spheres = add_sphere_n_spheres_y_right(n - 1, enter_sphere)
    return [enter_sphere] + right_spheres

def create_sphere(pos_i, radius):
    x_pos = pos_i[0]
    y_pos = pos_i[1]
    z_pos = pos_i[2]


    return {
        "type": "sphere",
        "movable": False,
        "geometry": {"position": [x_pos, y_pos, z_pos], "radius": radius},
    }

def add_sphere_n_spheres_x_left(n, sphere_dict):

    list_of_n_spheres = []

    for i in range(n):
        # Get the infromation from the sphere we wat to place a new sphere behind
        x_pos_sphere_i = sphere_dict['geometry']['position'][0]
        y_pos_sphere_i = sphere_dict['geometry']['position'][1]
        z_pos_sphere_i = sphere_dict['geometry']['position'][2]
        radius_sphere_i = sphere_dict['geometry']['radius']

        # Calculate the new x_pos of the new sphere
        x_new_sphere = x_pos_sphere_i - 2*radius_sphere_i
        # Create the new sphere
        new_sphere_i = create_sphere([x_new_sphere, y_pos_sphere_i, z_pos_sphere_i], radius_sphere_i)
        # Append the sphere to the list of spheres
        list_of_n_spheres.append(new_sphere_i)
        # Set the new sphere dict as sphere
        sphere_dict = new_sphere_i

    return list_of_n_spheres 


   
def add_sphere_n_spheres_x_right(n, sphere_dict):

    list_of_n_spheres = []

    for i in range(n):
        # Get the infromation from the sphere we wat to place a new sphere behind
        x_pos_sphere_i = sphere_dict['geometry']['position'][0]
        y_pos_sphere_i = sphere_dict['geometry']['position'][1]
        z_pos_sphere_i = sphere_dict['geometry']['position'][2]
        radius_sphere_i = sphere_dict['geometry']['radius']

        # Calculate the new x_pos of the new sphere
        x_new_sphere = x_pos_sphere_i + 2*radius_sphere_i
        # Create the new sphere
        new_sphere_i = create_sphere([x_new_sphere, y_pos_sphere_i, z_pos_sphere_i], radius_sphere_i)
        # Append the sphere to the list of spheres
        list_of_n_spheres.append(new_sphere_i)
        # Set the new sphere dict as sphere
        sphere_dict = new_sphere_i

    return list_of_n_spheres  

def add_sphere_n_spheres_y_left(n, sphere_dict):

    list_of_n_spheres = []

    for i in range(n):
        # Get the infromation from the sphere we wat to place a new sphere behind
        x_pos_sphere_i = sphere_dict['geometry']['position'][0]
        y_pos_sphere_i = sphere_dict['geometry']['position'][1]
        z_pos_sphere_i = sphere_dict['geometry']['position'][2]
        radius_sphere_i = sphere_dict['geometry']['radius']

        # Calculate the new x_pos of the new sphere
        y_new_sphere = y_pos_sphere_i - 2*radius_sphere_i
        # Create the new sphere
        new_sphere_i = create_sphere([x_pos_sphere_i, y_new_sphere, z_pos_sphere_i], radius_sphere_i)
        # Append the sphere to the list of spheres
        list_of_n_spheres.append(new_sphere_i)
        # Set the new sphere dict as sphere
        sphere_dict = new_sphere_i

    return list_of_n_spheres 

def add_sphere_n_spheres_y_right(n, sphere_dict):

    list_of_n_spheres = []

    for i in range(n):
        # Get the infromation from the sphere we wat to place a new sphere behind
        x_pos_sphere_i = sphere_dict['geometry']['position'][0]
        y_pos_sphere_i = sphere_dict['geometry']['position'][1]
        z_pos_sphere_i = sphere_dict['geometry']['position'][2]
        radius_sphere_i = sphere_dict['geometry']['radius']

        # Calculate the new x_pos of the new sphere
        y_new_sphere = y_pos_sphere_i + 2*radius_sphere_i
        # Create the new sphere
        new_sphere_i = create_sphere([x_pos_sphere_i, y_new_sphere, z_pos_sphere_i], radius_sphere_i)
        # Append the sphere to the list of spheres
        list_of_n_spheres.append(new_sphere_i)
        # Set the new sphere dict as sphere
        sphere_dict = new_sphere_i

    return list_of_n_spheres 