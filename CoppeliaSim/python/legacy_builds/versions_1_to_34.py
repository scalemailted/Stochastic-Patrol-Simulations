
'''
#[34] - Refactored for nicer codebase

def get(prop):
    data = {
        'cell_size': 0.5,
        'world_length': 5,
        'world_width': 6,
        'quadcopter_speed': 1,
        'enemy_count': 3,
        'enemy_speed': 1
    }
    return data[prop]


def get_axis_ranges(width, length):
    xmax = width/2.0
    xmin = -width/2.0
    ymax = length/2.0
    ymin = -length/2.0
    return (xmin,xmax),(ymin,ymax)


def add_masking_cell(i,j):
    cell_size = get("cell_size")
    floor = sim.getObject("/Floor")
    world_pos = get_world_coordinates([i,j])         
    cell_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cell_handle, floor, [world_pos[0], world_pos[1], 0.15])
    sim.setShapeColor(cell_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cell_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cell_handle

def init_grid():
    cell_size = get('cell_size')
    xaxis, yaxis = get_world_size()
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    grid = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cell_handle = add_masking_cell(i,j)
            grid[i][j] = int(cell_handle)
    return grid

def get_world_coordinates(grid_pos):
    cell_size = get('cell_size')
    (xmin, xmax),(ymin, ymax) = get_world_size()
    x = cell_size * grid_pos[0] + xmin + cell_size / 2.0
    y = cell_size * grid_pos[1] + ymin + cell_size / 2.0
    return x, y

def get_grid_coordinates(world_pos):
    cell_size = get('cell_size')
    (xmin, xmax),(ymin, ymax) = get_world_size()
    i = int((world_pos[0] - xmin) / cell_size)
    j = int((world_pos[1] - ymin) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of cell handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    #If no cells remain then init a new masked grid and select from there
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    # Choose a random unvisited cell as the next waypoint
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, mask2d):
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    speed = get('quadcopter_speed')
    step_size = speed * sim.getSimulationTimeStep()
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)

    def calculate_weight(dst, src):
        penalty = 1
        if np.any(np.all(visited_mask2d == dst, axis=1)):
            penalty = 2 + visited_len
        unvisited_factor = unvisited_len / (visited_len + unvisited_len)
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((dst - src)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents and unvisited_len / (visited_len + unvisited_len) <= 0.05:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j

def add_enemy(i,j):
    cell_size = get('cell_size')
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j])         
    enemy_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(enemy_handle, floor, [world[0], world[1], 0.155])
    sim.setShapeColor(enemy_handle, None, 0, [1, 0, 0])
    sim.setObjectSpecialProperty(enemy_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return enemy_handle

def init_enemies():
    cell_size = get('cell_size')
    xaxis,yaxis = get_world_size()
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    enemy_count = get('enemy_count')
    enemy_handles = []
    for i in range(enemy_count):
        enemy_i = random.randint(0, nrows-1)
        enemy_j = random.randint(0, ncols-1)
        enemy_handle = add_enemy(enemy_i, enemy_j)
        enemy_handles.append(int(enemy_handle))
    return enemy_handles

def move_to_random_adjacent_cell(src_handle, mask2d=None):
    cell_size = get('cell_size')
    xaxis, yaxis = get_world_size()
    speed = get('enemy_speed')
    current_time = sim.getSimulationTime()
    if current_time % 1.0 < sim.getSimulationTimeStep():
        current_pos = sim.getObjectPosition(src_handle, -1)
        step_size = speed * sim.getSimulationTimeStep()

        i, j = get_grid_coordinates(current_pos)
        nrows = int((xaxis[1] - xaxis[0]) / cell_size)
        ncols = int((yaxis[1] - yaxis[0]) / cell_size)

        copter_i, copter_j = get_observer_coordinates()

        while True:
            next_i, next_j = i + random.choice([-1, 0, 1]), j + random.choice([-1, 0, 1])
            if next_i >= 0 and next_i < nrows and next_j >= 0 and next_j < ncols and (next_i != copter_i or next_j != copter_j):
                break

        world = get_world_coordinates([next_i, next_j])
        new_pos = [world[0], world[1], current_pos[2]]
        sim.setObjectPosition(src_handle, -1, new_pos)

        # Check if the next cell is unmasked and change color if it is
        if mask2d is not None and mask2d[next_i][next_j] != -1:
            sim.setShapeColor(src_handle, None, 0, [0, 0, 0])
        else:
            sim.setShapeColor(src_handle, None, 0, [1, 0, 0])

    
def sysCall_thread():
    init_floor2d()
    # Get a list of all unvisited cells
    mask2d = init_grid()
    enemy_handles = init_enemies()
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    caught_enemies = []
    while True:
        for enemy_handle in enemy_handles:
            if enemy_handle not in caught_enemies:
                move_to_random_adjacent_cell(enemy_handle, mask2d)
            if check_waypoint_reached(sim.getObjectPosition(enemy_handle, -1)):
                caught_enemies.append(enemy_handle)
                sim.setShapeColor(enemy_handle, None, 0, [0.5, 0, 0])
                
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()


def hide_default_floor():
    handle = sim.getObjectHandle("/Floor/box")
    sim.setModelProperty(handle,sim.modelproperty_not_visible)


def init_floor2d():
    cell_size = get('cell_size')
    width = get('world_width')
    length = get('world_length')
    xaxis, yaxis = get_axis_ranges(width, length)
    zlevel = 0
    # Hide default floor
    hide_default_floor()
    # Define the size of the floor
    x_size = xaxis[1] - xaxis[0]
    y_size = yaxis[1] - yaxis[0]
    
    # Calculate the number of cells in the x and y directions
    nrows = int(x_size / cell_size)
    ncols = int(y_size / cell_size)
    
    # Create a single large floor surface
    floor_handle = sim.createPureShape(0, 16, [x_size, y_size, 0.01], 0.5, None)
    sim.setObjectPosition(floor_handle, -1, [xaxis[0] + x_size/2, yaxis[0] + y_size/2, zlevel])
    
    # Apply a light grey/grey checkerboard texture to the floor surface
    texture_size = 128
    texture = np.zeros((texture_size * ncols, texture_size * nrows, 3), dtype=np.uint8)
    for i in range(ncols):
        for j in range(nrows):
            color = [255, 255, 255] if (i + j) % 2 == 0 else [192, 192, 192]
            block = np.full((texture_size, texture_size, 3), color, dtype=np.uint8)
            texture[i*texture_size:(i+1)*texture_size, j*texture_size:(j+1)*texture_size, :] = block
    
    with tempfile.NamedTemporaryFile(mode='wb', suffix='.png', delete=False) as f:
        image_filename = f.name
        img = Image.fromarray(texture, 'RGB')
        img.save(f, 'PNG')

    _, texture_id, _ = sim.createTexture(image_filename, 0)
    sim.addLog(0,"Texture: {}".format( texture_id))
    uv = max(x_size, y_size)
    sim.setShapeTexture(floor_handle, texture_id, sim.texturemap_cube, 1, [uv,uv])
    world_handle = sim.getObjectHandle("/Cuboid")
    sim.setObjectName(world_handle, "Worldspace")
    

def get_world_size():
    world_handle = sim.getObjectHandle("Worldspace")
    xmin = sim.getObjectFloatParam(world_handle, sim.objfloatparam_modelbbox_min_x)
    xmax = sim.getObjectFloatParam(world_handle, sim.objfloatparam_modelbbox_max_x)
    ymin = sim.getObjectFloatParam(world_handle, sim.objfloatparam_modelbbox_min_y)
    ymax = sim.getObjectFloatParam(world_handle, sim.objfloatparam_modelbbox_max_y)
    return (xmin,xmax),(ymin,ymax)

'''

'''
#[33] - Floor experiments -> Converted into 1 body with texture!
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + visited_len
        unvisited_factor = unvisited_len / (visited_len + unvisited_len)
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents and unvisited_len / (visited_len + unvisited_len) <= 0.05:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j

def add_enemy(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    enemy_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(enemy_handle, floor, [world[0], world[1], 0.155])
    sim.setShapeColor(enemy_handle, None, 0, [1, 0, 0])
    sim.setObjectSpecialProperty(enemy_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return enemy_handle

def init_enemies(n=3,xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    enemy_handles = []
    for i in range(n):
        enemy_i = random.randint(0, nrows-1)
        enemy_j = random.randint(0, ncols-1)
        enemy_handle = add_enemy(enemy_i, enemy_j, cell_size)
        enemy_handles.append(int(enemy_handle))
    return enemy_handles

def move_to_random_adjacent_cell(src_handle, speed, xaxis, yaxis, cell_size=0.5, mask2d=None):
    current_time = sim.getSimulationTime()
    if current_time % 1.0 < sim.getSimulationTimeStep():
        current_pos = sim.getObjectPosition(src_handle, -1)
        step_size = speed * sim.getSimulationTimeStep()

        i, j = get_grid_coordinates(current_pos, cell_size)
        nrows = int((xaxis[1] - xaxis[0]) / cell_size)
        ncols = int((yaxis[1] - yaxis[0]) / cell_size)

        copter_i, copter_j = get_observer_coordinates()

        while True:
            next_i, next_j = i + random.choice([-1, 0, 1]), j + random.choice([-1, 0, 1])
            if next_i >= 0 and next_i < nrows and next_j >= 0 and next_j < ncols and (next_i != copter_i or next_j != copter_j):
                break

        world = get_world_coordinates([next_i, next_j], cell_size)
        new_pos = [world[0], world[1], current_pos[2]]
        sim.setObjectPosition(src_handle, -1, new_pos)

        # Check if the next cell is unmasked and change color if it is
        if mask2d is not None and mask2d[next_i][next_j] != -1:
            sim.setShapeColor(src_handle, None, 0, [0, 0, 0])
        else:
            sim.setShapeColor(src_handle, None, 0, [1, 0, 0])

    
def sysCall_thread():
    init_floor2d([-3.0,3.0],[-2.5,2.5])
    #return
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    enemy_handles = init_enemies(n=3, xaxis=xaxis, yaxis=yaxis, cell_size=cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    caught_enemies = []
    while True:
        for enemy_handle in enemy_handles:
            if enemy_handle not in caught_enemies:
                move_to_random_adjacent_cell(enemy_handle, speed, xaxis, yaxis, cell_size, mask2d)
            if check_waypoint_reached(sim.getObjectPosition(enemy_handle, -1)):
                caught_enemies.append(enemy_handle)
                sim.setShapeColor(enemy_handle, None, 0, [0.5, 0, 0])
                
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()


def hide_default_floor():
    handle = sim.getObjectHandle("/Floor/box")
    #sim.setObjectInt32Param(handle, sim.modelproperty_not_visible, 0)
    sim.setModelProperty(handle,sim.modelproperty_not_visible)


import tempfile
import numpy as np
from PIL import Image

def init_floor2d(xaxis=[-2.5,2.5], yaxis=[-2.5,2.5], zlevel=0, cell_size=0.5):
    hide_default_floor()
    # Define the size of the floor
    x_size = xaxis[1] - xaxis[0]
    y_size = yaxis[1] - yaxis[0]
    
    # Calculate the number of cells in the x and y directions
    nrows = int(x_size / cell_size)
    ncols = int(y_size / cell_size)
    
    # Create a single large floor surface
    floor_handle = sim.createPureShape(0, 16, [x_size, y_size, 0.01], 0.5, None)
    sim.setObjectPosition(floor_handle, -1, [xaxis[0] + x_size/2, yaxis[0] + y_size/2, zlevel])
    
    # Apply a light grey/grey checkerboard texture to the floor surface
    texture_size = 128
    texture = np.zeros((texture_size * ncols, texture_size * nrows, 3), dtype=np.uint8)
    for i in range(ncols):
        for j in range(nrows):
            color = [255, 255, 255] if (i + j) % 2 == 0 else [192, 192, 192]
            block = np.full((texture_size, texture_size, 3), color, dtype=np.uint8)
            texture[i*texture_size:(i+1)*texture_size, j*texture_size:(j+1)*texture_size, :] = block
    
    with tempfile.NamedTemporaryFile(mode='wb', suffix='.png', delete=False) as f:
        image_filename = f.name
        img = Image.fromarray(texture, 'RGB')
        img.save(f, 'PNG')

    _, texture_id, _ = sim.createTexture(image_filename, 0)
    sim.addLog(0,"Texture: {}".format( texture_id))
    uv = max(x_size, y_size)
    sim.setShapeTexture(floor_handle, texture_id, sim.texturemap_cube, 1, [uv,uv])

'''





'''
#[32] - Floor experiments -> function that programtically creates a new tiled floor (performance heavy!)
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + visited_len
        unvisited_factor = unvisited_len / (visited_len + unvisited_len)
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents and unvisited_len / (visited_len + unvisited_len) <= 0.05:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j

def add_enemy(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    enemy_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(enemy_handle, floor, [world[0], world[1], 0.155])
    sim.setShapeColor(enemy_handle, None, 0, [1, 0, 0])
    sim.setObjectSpecialProperty(enemy_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return enemy_handle

def init_enemies(n=3,xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    enemy_handles = []
    for i in range(n):
        enemy_i = random.randint(0, nrows-1)
        enemy_j = random.randint(0, ncols-1)
        enemy_handle = add_enemy(enemy_i, enemy_j, cell_size)
        enemy_handles.append(int(enemy_handle))
    return enemy_handles

def move_to_random_adjacent_cell(src_handle, speed, xaxis, yaxis, cell_size=0.5, mask2d=None):
    current_time = sim.getSimulationTime()
    if current_time % 1.0 < sim.getSimulationTimeStep():
        current_pos = sim.getObjectPosition(src_handle, -1)
        step_size = speed * sim.getSimulationTimeStep()

        i, j = get_grid_coordinates(current_pos, cell_size)
        nrows = int((xaxis[1] - xaxis[0]) / cell_size)
        ncols = int((yaxis[1] - yaxis[0]) / cell_size)

        copter_i, copter_j = get_observer_coordinates()

        while True:
            next_i, next_j = i + random.choice([-1, 0, 1]), j + random.choice([-1, 0, 1])
            if next_i >= 0 and next_i < nrows and next_j >= 0 and next_j < ncols and (next_i != copter_i or next_j != copter_j):
                break

        world = get_world_coordinates([next_i, next_j], cell_size)
        new_pos = [world[0], world[1], current_pos[2]]
        sim.setObjectPosition(src_handle, -1, new_pos)

        # Check if the next cell is unmasked and change color if it is
        if mask2d is not None and mask2d[next_i][next_j] != -1:
            sim.setShapeColor(src_handle, None, 0, [0, 0, 0])
        else:
            sim.setShapeColor(src_handle, None, 0, [1, 0, 0])

    
def sysCall_thread():
    init_floor2d()
    #return
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    enemy_handles = init_enemies(n=3, xaxis=xaxis, yaxis=yaxis, cell_size=cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    caught_enemies = []
    while True:
        for enemy_handle in enemy_handles:
            if enemy_handle not in caught_enemies:
                move_to_random_adjacent_cell(enemy_handle, speed, xaxis, yaxis, cell_size, mask2d)
            if check_waypoint_reached(sim.getObjectPosition(enemy_handle, -1)):
                caught_enemies.append(enemy_handle)
                sim.setShapeColor(enemy_handle, None, 0, [0.5, 0, 0])
                
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()

def resize_floor(xaxis=[-2.5,2.5], yaxis=[-2.5,2.5]):
    floor_handle = sim.getObjectHandle("/Floor/box")
    xmin = sim.getObjectFloatParam(floor_handle, sim.objfloatparam_modelbbox_min_x)
    xmax = sim.getObjectFloatParam(floor_handle, sim.objfloatparam_modelbbox_max_x)
    sim.addLog(0,"floor x-size: ({},{})" .format(xmin, xmax))
    ymin = sim.getObjectFloatParam(floor_handle, sim.objfloatparam_modelbbox_min_y)
    ymax = sim.getObjectFloatParam(floor_handle, sim.objfloatparam_modelbbox_max_y)
    sim.addLog(0,"floor y-size: ({},{})" .format(ymin, ymax))
    # FAIL - Read only properties, so this doesn't work
    #sim.setObjectFloatParam(floor_handle, sim.objfloatparam_modelbbox_min_x, -10.0)
    #sim.setObjectFloatParam(floor_handle, sim.objfloatparam_modelbbox_max_x, 10.0)
    #sim.setObjectFloatParam(floor_handle, sim.objfloatparam_modelbbox_min_y, -8.0)
    #sim.setObjectFloatParam(floor_handle, sim.objfloatparam_modelbbox_max_y, 10.0)
    # FAIL - Changes data but not the rendered size, plus keeps changes even after simulation 
    #xscale = 1
    #yscale = 1
    #zscale = 1
    #sim.scaleObject(floor_handle,xscale,yscale,zscale, 0)

def init_floor2d(xaxis=[-2.5,2.5], yaxis=[-2.5,2.5], zlevel=0, cell_size=0.5):
    # Define the size of the floor
    x_size = xaxis[1] - xaxis[0]
    y_size = yaxis[1] - yaxis[0]
    
    # Calculate the number of cells in the x and y directions
    nrows = int(x_size / cell_size)
    ncols = int(y_size / cell_size)
    
    # Create a floor made up of square shapes of alternating colors light gray, gray
    for i in range(nrows):
        for j in range(ncols):
            color = [0.8, 0.8, 0.8] if (i + j) % 2 == 0 else [0.7, 0.7, 0.7]
            pos = [xaxis[0] + i * cell_size + cell_size / 2, yaxis[0] + j * cell_size + cell_size / 2, zlevel]
            cell_handle = sim.createPureShape(0, 16, [cell_size, cell_size, 0.01], 0.5, None)
            sim.setObjectPosition(cell_handle, -1, pos)
            sim.setShapeColor(cell_handle, None, 0, color)
'''
    
    

'''

#[31] - Enemies move works fully!
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + visited_len
        unvisited_factor = unvisited_len / (visited_len + unvisited_len)
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents and unvisited_len / (visited_len + unvisited_len) <= 0.05:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j

def add_enemy(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    enemy_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(enemy_handle, floor, [world[0], world[1], 0.155])
    sim.setShapeColor(enemy_handle, None, 0, [1, 0, 0])
    sim.setObjectSpecialProperty(enemy_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return enemy_handle

def init_enemies(n=3,xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    enemy_handles = []
    for i in range(n):
        enemy_i = random.randint(0, nrows-1)
        enemy_j = random.randint(0, ncols-1)
        enemy_handle = add_enemy(enemy_i, enemy_j, cell_size)
        enemy_handles.append(int(enemy_handle))
    return enemy_handles

def move_to_random_adjacent_cell(src_handle, speed, xaxis, yaxis, cell_size=0.5, mask2d=None):
    current_time = sim.getSimulationTime()
    if current_time % 1.0 < sim.getSimulationTimeStep():
        current_pos = sim.getObjectPosition(src_handle, -1)
        step_size = speed * sim.getSimulationTimeStep()

        i, j = get_grid_coordinates(current_pos, cell_size)
        nrows = int((xaxis[1] - xaxis[0]) / cell_size)
        ncols = int((yaxis[1] - yaxis[0]) / cell_size)

        copter_i, copter_j = get_observer_coordinates()

        while True:
            next_i, next_j = i + random.choice([-1, 0, 1]), j + random.choice([-1, 0, 1])
            if next_i >= 0 and next_i < nrows and next_j >= 0 and next_j < ncols and (next_i != copter_i or next_j != copter_j):
                break

        world = get_world_coordinates([next_i, next_j], cell_size)
        new_pos = [world[0], world[1], current_pos[2]]
        sim.setObjectPosition(src_handle, -1, new_pos)

        # Check if the next cell is unmasked and change color if it is
        if mask2d is not None and mask2d[next_i][next_j] != -1:
            sim.setShapeColor(src_handle, None, 0, [0, 0, 0])
        else:
            sim.setShapeColor(src_handle, None, 0, [1, 0, 0])

    
def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    enemy_handles = init_enemies(n=3, xaxis=xaxis, yaxis=yaxis, cell_size=cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    caught_enemies = []
    while True:
        for enemy_handle in enemy_handles:
            if enemy_handle not in caught_enemies:
                move_to_random_adjacent_cell(enemy_handle, speed, xaxis, yaxis, cell_size, mask2d)
            if check_waypoint_reached(sim.getObjectPosition(enemy_handle, -1)):
                caught_enemies.append(enemy_handle)
                sim.setShapeColor(enemy_handle, None, 0, [0.5, 0, 0])
                
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()
'''

'''
#[30Bii] - Enemies move everywhere, but avoid copter cell, making it impossible!
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + visited_len
        unvisited_factor = unvisited_len / (visited_len + unvisited_len)
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents and unvisited_len / (visited_len + unvisited_len) <= 0.05:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j

def add_enemy(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    enemy_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(enemy_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(enemy_handle, None, 0, [1, 0, 0])
    sim.setObjectSpecialProperty(enemy_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return enemy_handle

def init_enemies(n=3,xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    enemy_handles = []
    for i in range(n):
        enemy_i = random.randint(0, nrows-1)
        enemy_j = random.randint(0, ncols-1)
        enemy_handle = add_enemy(enemy_i, enemy_j, cell_size)
        enemy_handles.append(int(enemy_handle))
    return enemy_handles

def move_to_random_adjacent_cell(src_handle, speed, xaxis, yaxis, cell_size=0.5, mask2d=None):
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()

    i, j = get_grid_coordinates(current_pos, cell_size)
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)

    copter_i, copter_j = get_observer_coordinates()

    while True:
        next_i, next_j = i + random.choice([-1, 0, 1]), j + random.choice([-1, 0, 1])
        if next_i >= 0 and next_i < nrows and next_j >= 0 and next_j < ncols and (next_i != copter_i or next_j != copter_j):
            break

    world = get_world_coordinates([next_i, next_j], cell_size)
    new_pos = [world[0], world[1], current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)

    # Check if the next cell is unmasked and change color if it is
    if mask2d is not None and mask2d[next_i][next_j] != -1:
        sim.setShapeColor(src_handle, None, 0, [0, 0, 0])
    else:
        sim.setShapeColor(src_handle, None, 0, [1, 0, 0])


    
def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    enemy_handles = init_enemies(n=3, xaxis=xaxis, yaxis=yaxis, cell_size=cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    caught_enemies = []
    while True:
        for enemy_handle in enemy_handles:
            if enemy_handle not in caught_enemies:
                move_to_random_adjacent_cell(enemy_handle, speed, xaxis, yaxis, cell_size, mask2d)
            if check_waypoint_reached(sim.getObjectPosition(enemy_handle, -1)):
                caught_enemies.append(enemy_handle)
                sim.setShapeColor(enemy_handle, None, 0, [0.5, 0, 0])
                
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()
'''


'''
#[30Bi] - Enemies move only in hidden areas
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + visited_len
        unvisited_factor = unvisited_len / (visited_len + unvisited_len)
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents and unvisited_len / (visited_len + unvisited_len) <= 0.05:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j

def add_enemy(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    enemy_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(enemy_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(enemy_handle, None, 0, [1, 0, 0])
    sim.setObjectSpecialProperty(enemy_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return enemy_handle

def init_enemies(n=3,xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    enemy_handles = []
    for i in range(n):
        enemy_i = random.randint(0, nrows-1)
        enemy_j = random.randint(0, ncols-1)
        enemy_handle = add_enemy(enemy_i, enemy_j, cell_size)
        enemy_handles.append(int(enemy_handle))
    return enemy_handles

def move_to_random_adjacent_cell(src_handle, speed, xaxis, yaxis, cell_size=0.5, mask2d=None):
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep() / 2  # Slow down the enemy

    i, j = get_grid_coordinates(current_pos, cell_size)
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)

    while True:
        next_i, next_j = i + random.choice([-1, 0, 1]), j + random.choice([-1, 0, 1])
        if next_i >= 0 and next_i < nrows and next_j >= 0 and next_j < ncols and (mask2d is None or mask2d[next_i][next_j] != -1):
            break

    world = get_world_coordinates([next_i, next_j], cell_size)
    new_pos = [world[0], world[1], current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)

    # Check if the next cell is unmasked and change color if it is
    if mask2d is not None and mask2d[next_i][next_j] != -1:
        sim.setShapeColor(src_handle, None, 0, [0, 0, 0])
    else:
        sim.setShapeColor(src_handle, None, 0, [1, 0, 0])


    
def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    enemy_handles = init_enemies(n=3, xaxis=xaxis, yaxis=yaxis, cell_size=cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    caught_enemies = []
    while True:
        for enemy_handle in enemy_handles:
            if enemy_handle not in caught_enemies:
                move_to_random_adjacent_cell(enemy_handle, speed, xaxis, yaxis, cell_size, mask2d)
            if check_waypoint_reached(sim.getObjectPosition(enemy_handle, -1)):
                caught_enemies.append(enemy_handle)
                sim.setShapeColor(enemy_handle, None, 0, [0.5, 0, 0])
                
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()
'''

'''
#[30B] - Hide enemies in mask and reveal them in unmasked areas
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + visited_len
        unvisited_factor = unvisited_len / (visited_len + unvisited_len)
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents and unvisited_len / (visited_len + unvisited_len) <= 0.05:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j

def add_enemy(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    enemy_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(enemy_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(enemy_handle, None, 0, [1, 0, 0])
    sim.setObjectSpecialProperty(enemy_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return enemy_handle

def init_enemies(n=3,xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    enemy_handles = []
    for i in range(n):
        enemy_i = random.randint(0, nrows-1)
        enemy_j = random.randint(0, ncols-1)
        enemy_handle = add_enemy(enemy_i, enemy_j, cell_size)
        enemy_handles.append(int(enemy_handle))
    return enemy_handles

def move_to_random_adjacent_cell(src_handle, speed, xaxis, yaxis, cell_size=0.5, mask2d=None):
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()

    i, j = get_grid_coordinates(current_pos, cell_size)
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)

    while True:
        next_i, next_j = i + random.choice([-1, 0, 1]), j + random.choice([-1, 0, 1])
        if next_i >= 0 and next_i < nrows and next_j >= 0 and next_j < ncols:
            break

    world = get_world_coordinates([next_i, next_j], cell_size)
    new_pos = [world[0], world[1], current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)

    # Check if the next cell is unmasked and change color if it is
    if mask2d is not None and mask2d[next_i][next_j] != -1:
        sim.setShapeColor(src_handle, None, 0, [0, 0, 0])
    else:
        sim.setShapeColor(src_handle, None, 0, [1, 0, 0])

    
def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    enemy_handles = init_enemies(n=3, xaxis=xaxis, yaxis=yaxis, cell_size=cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    caught_enemies = []
    while True:
        for enemy_handle in enemy_handles:
            if enemy_handle not in caught_enemies:
                move_to_random_adjacent_cell(enemy_handle, speed, xaxis, yaxis, cell_size, mask2d)
            if check_waypoint_reached(sim.getObjectPosition(enemy_handle, -1)):
                caught_enemies.append(enemy_handle)
                sim.setShapeColor(enemy_handle, None, 0, [0.5, 0, 0])
                
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()
'''


'''
#[30A] - Add monsters - works but they are visible all the time and never hidden
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + visited_len
        unvisited_factor = unvisited_len / (visited_len + unvisited_len)
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents and unvisited_len / (visited_len + unvisited_len) <= 0.05:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j

def add_enemy(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    enemy_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(enemy_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(enemy_handle, None, 0, [1, 0, 0])
    sim.setObjectSpecialProperty(enemy_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return enemy_handle

def init_enemies(n=3,xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    enemy_handles = []
    for i in range(n):
        enemy_i = random.randint(0, nrows-1)
        enemy_j = random.randint(0, ncols-1)
        enemy_handle = add_enemy(enemy_i, enemy_j, cell_size)
        enemy_handles.append(int(enemy_handle))
    return enemy_handles

def move_to_random_adjacent_cell(src_handle, speed, xaxis, yaxis, cell_size=0.5):
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()

    i, j = get_grid_coordinates(current_pos, cell_size)
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)

    while True:
        next_i, next_j = i + random.choice([-1, 0, 1]), j + random.choice([-1, 0, 1])
        if next_i >= 0 and next_i < nrows and next_j >= 0 and next_j < ncols:
            break

    world = get_world_coordinates([next_i, next_j], cell_size)
    new_pos = [world[0], world[1], current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)

    
def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    enemy_handles = init_enemies(n=3, xaxis=xaxis, yaxis=yaxis, cell_size=cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    caught_enemies = []
    while True:
        for enemy_handle in enemy_handles:
            if enemy_handle not in caught_enemies:
                move_to_random_adjacent_cell(enemy_handle, speed, xaxis, yaxis, cell_size)
            if check_waypoint_reached(sim.getObjectPosition(enemy_handle, -1)):
                caught_enemies.append(enemy_handle)
                sim.setShapeColor(enemy_handle, None, 0, [0.5, 0, 0])
                
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()

'''



'''
#[30] - Add monsters - works but they walk off stage and make tiny steps
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + visited_len
        unvisited_factor = unvisited_len / (visited_len + unvisited_len)
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents and unvisited_len / (visited_len + unvisited_len) <= 0.05:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j

def add_enemy(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    enemy_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(enemy_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(enemy_handle, None, 0, [1, 0, 0])
    sim.setObjectSpecialProperty(enemy_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return enemy_handle

def init_enemies(n=3,xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    enemy_handles = []
    for i in range(n):
        enemy_i = random.randint(0, nrows-1)
        enemy_j = random.randint(0, ncols-1)
        enemy_handle = add_enemy(enemy_i, enemy_j, cell_size)
        enemy_handles.append(int(enemy_handle))
    return enemy_handles

def move_to_random_cell(src_handle, speed):
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()

    # Choose a random adjacent cell as the next destination
    angle = random.uniform(0, 2*np.pi)
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)

    
def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    enemy_handles = init_enemies(n=3, xaxis=xaxis, yaxis=yaxis, cell_size=cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    caught_enemies = []
    while True:
        for enemy_handle in enemy_handles:
            if enemy_handle not in caught_enemies:
                move_to_random_cell(enemy_handle, speed)
            i, j = get_grid_coordinates(sim.getObjectPosition(enemy_handle, -1), cell_size)
            if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
                remove_from_grid(i,j, mask2d)
            if check_waypoint_reached(sim.getObjectPosition(enemy_handle, -1)):
                caught_enemies.append(enemy_handle)
                sim.setShapeColor(enemy_handle, None, 0, [0.5, 0, 0])
                
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()

'''




'''
#[27C] - refactored move_to_waypoint to be more performant
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + visited_len
        unvisited_factor = unvisited_len / (visited_len + unvisited_len)
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents and unvisited_len / (visited_len + unvisited_len) <= 0.05:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)



#def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
#    unvisited_mask = np.count_nonzero(mask2d != -1)
#    visited_mask = np.count_nonzero(mask2d == -1)
#    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
#    visited_mask2d = np.array(np.where(mask2d == -1)).T
#    current_pos = sim.getObjectPosition(src_handle, -1)
#    step_size = speed * sim.getSimulationTimeStep()
#
#    def calculate_weight(location, current_location):
#        penalty = 1
#        if np.any(np.all(visited_mask2d == location, axis=1)):
#            penalty = 2 + len(visited_mask2d)
#        unvisited_factor = len(unvisited_mask2d) / (len(visited_mask2d) + len(unvisited_mask2d))
#        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))
#
#    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
#    distances = sorted(distances, key=lambda x: x[2])
#    
#    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
#    
#    if adjacents:
#        distances = adjacents
#
#    # set the destination to the next waypoint
#    destination = next_waypoint
#    
#    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
#    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
#    sim.setObjectPosition(src_handle, -1, new_pos)

def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j
    

def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()
'''

'''
#[27B] - refactored with better pathing! Fixed bug, don't remove if observer off map
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask = np.count_nonzero(mask2d != -1)
    visited_mask = np.count_nonzero(mask2d == -1)
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + len(visited_mask2d)
        unvisited_factor = len(unvisited_mask2d) / (len(visited_mask2d) + len(unvisited_mask2d))
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(mask2d):
    i,j = get_observer_coordinates()
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        remove_from_grid(i,j, mask2d)


def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j
    

def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()
'''

'''
#[27A] - refactored with better pathing! Bug - crashes when leaves world on a pivot
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed, mask2d):
    unvisited_mask = np.count_nonzero(mask2d != -1)
    visited_mask = np.count_nonzero(mask2d == -1)
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()

    def calculate_weight(location, current_location):
        penalty = 1
        if np.any(np.all(visited_mask2d == location, axis=1)):
            penalty = 2 + len(visited_mask2d)
        unvisited_factor = len(unvisited_mask2d) / (len(visited_mask2d) + len(unvisited_mask2d))
        return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

    distances = [np.append(unvisited, calculate_weight(unvisited, current_pos[:2])) for unvisited in unvisited_mask2d]
    distances = sorted(distances, key=lambda x: x[2])
    
    adjacents = [distance[:2] for distance in distances if (np.abs(distance[0] - current_pos[0]) <= 1 and distance[1] == current_pos[1]) or (np.abs(distance[1] - current_pos[1]) <= 1 and distance[0] == current_pos[0])]
    
    if adjacents:
        distances = adjacents

    # set the destination to the next waypoint
    destination = next_waypoint
    
    angle = np.arctan2(destination[1] - current_pos[1], destination[0] - current_pos[0]) + (random.random() - 0.5) * 0.1
    new_pos = [current_pos[0] + np.cos(angle) * step_size, current_pos[1] + np.sin(angle) * step_size, current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)


def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(object_handles):
    i,j = get_observer_coordinates()
    remove_from_grid(i,j, object_handles)

def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j
    

def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed, mask2d)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()
'''

'''
#[28] - improved pathing
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)
   

def get_distance(location, current_location, mask2d):
    penalty = 1
    visited_count = np.count_nonzero(np.all(mask2d == location, axis=1))
    unvisited_count = len(mask2d) - visited_count
    total_count = len(mask2d)
    if visited_count > 0:
        penalty = 2 + visited_count
    unvisited_factor = unvisited_count / total_count
    return penalty + unvisited_factor * -1 + np.sqrt(np.sum((location - current_location)**2))

   

#def get_distance(src, dst, mask2d):
#    penalty = 1
#    visited_count = np.count_nonzero(mask2d == -1)
#    unvisited_count = np.count_nonzero(mask2d != -1)
#    total_count = mask2d.size
#    if np.isin(location, mask2d, axis=0).any():
#        penalty = 2 + visited_count
#    unvisited_factor = unvisited_count / total_count
#    return penalty + unvisited_factor * -1 + math.sqrt((location[0] - current_location[0])**2 + (location[1] - current_location[1])**2)


def get_distances(current_location, mask2d):
    distances = np.zeros((mask2d.shape[0], 3))
    distances[:, 0:2] = mask2d
    distances[:, 2] = np.apply_along_axis(lambda x: get_distance(x, current_location), 1, mask2d)
    return distances

def sort_distances(distances):
    return sorted(distances, key=lambda x: x[2])

def set_target_waypoint(current_location, mask2d):
    distances = get_distances(current_location, mask2d)
    adjacents = [dist for dist in distances if abs(dist[0] - current_location[0]) <= cell_size and dist[1] == current_location[1] or abs(dist[1] - current_location[1]) <= cell_size and dist[0] == current_location[0]]
    if adjacents:
        distances = adjacents
    sorted_distances = sort_distances(distances)
    random_index = 0 if len(unvisited_locations) / total_locations <= 0.05 else int(random.random() * len(sorted_distances))
    target_x, target_y = sorted_distances[random_index][0], sorted_distances[random_index][1]
    return target_x, target_y

def select_random_waypoint(mask2d):
    flat_mask = mask2d[mask2d != -1].tolist()
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    #waypoint_handle = random.choice(flat_mask)
    #next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    #sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    
    current_location = sim.getObjectPosition(sim.getObject("."), -1)
    distances = get_distances(current_location, mask2d)
    target_x, target_y = set_target_waypoint(distances, mask2d)
    
    target_x, target_y = get_world_coordinates([target_x, target_y], 0.5)
    return target_x, target_y, mask2d


def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()
'''



'''
#[27] - refactored initial variable names to be better
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(mask2d):
    # Flatten the 2D array of object handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    
    waypoint_handle = random.choice(flat_mask)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return next_waypoint, mask2d

def move_to_waypoint(src_handle, next_waypoint, speed):
    current_pos = sim.getObjectPosition(src_handle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
               current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
               current_pos[2]]
    sim.setObjectPosition(src_handle, -1, new_pos)

def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(object_handles):
    i,j = get_observer_coordinates()
    remove_from_grid(i,j, object_handles)

def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j
    

def sysCall_thread():
    speed = 1  # Specify the speed of the sphere object in m/s
    cell_size = 0.5
    xaxis = [-2.5, 2.5]
    yaxis = [-2.5, 2.5]
    # Get a list of all unvisited cells
    mask2d = init_grid(xaxis,yaxis,cell_size)
    #handle for src, dst objects
    target_handle = sim.getObject(".")
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, speed)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(mask2d)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()
'''


'''
#[26] - refactored with functional decomposition
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(object_handles):
    # Flatten the 2D array of object handles into a 1D list
    flat_handles = object_handles[object_handles != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_handles) == 0:
        object_handles = init_grid()
        return select_random_waypoint(object_handles)
    
    waypoint_handle = random.choice(flat_handles)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return waypoint_handle, next_waypoint, object_handles

def move_to_waypoint(objHandle, next_waypoint, speed):
    current_pos = sim.getObjectPosition(objHandle, -1)
    step_size = speed * sim.getSimulationTimeStep()
    new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
               current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
               current_pos[2]]
    sim.setObjectPosition(objHandle, -1, new_pos)

def check_waypoint_reached(next_waypoint):
    src_i,src_j = get_observer_coordinates()
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    return src_i == dst_i and src_j == dst_j

def unmask_cell_at_observer(object_handles):
    i,j = get_observer_coordinates()
    remove_from_grid(i,j, object_handles)

def get_observer_coordinates():
    handle = sim.getObject("/Quadcopter")
    world_pos = sim.getObjectPosition( handle, -1)
    i,j = get_grid_coordinates(world_pos)
    return i,j
    

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    # Get a list of all unvisited cells
    object_handles = init_grid()
    grid_size = 0.5
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint, object_handles = select_random_waypoint(object_handles)
        
        # Move the sphere object towards the next waypoint
        move_to_waypoint(objHandle, next_waypoint, speed)
            
        #remove grid cells at quadcopter pos
        unmask_cell_at_observer(object_handles)
        
        # Check if the sphere object has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            next_waypoint = None
        
        sim.switchThread()
'''


'''
#[26] - refactored names & parameters
def add_masking_cube(i,j,cell_size=0.5):
    floor = sim.getObject("/Floor")
    world = get_world_coordinates([i,j], cell_size)         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [world[0], world[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid(xaxis=[-2.5,2.5],yaxis=[-2.5,2.5], cell_size=0.5):
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    object_handles = np.empty((nrows, ncols), dtype=np.int32)
    for i in range(nrows):
        for j in range(ncols):
            cube_handle = add_masking_cube(i,j,cell_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_world_coordinates(grid_pos, cell_size=0.5):
    x = cell_size * grid_pos[0] - 2.5 + cell_size / 2
    y = cell_size * grid_pos[1] - 2.5 + cell_size / 2
    return [x,y]

def get_grid_coordinates(world_pos, cell_size=0.5):
    i = int((world_pos[0] + 2.5) / cell_size)
    j = int((world_pos[1] + 2.5) / cell_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(object_handles):
    # Flatten the 2D array of object handles into a 1D list
    flat_handles = object_handles[object_handles != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_handles) == 0:
        object_handles = init_grid()
        return select_random_waypoint(object_handles)
    
    waypoint_handle = random.choice(flat_handles)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return waypoint_handle, next_waypoint, object_handles

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    # Get a list of all unvisited cells
    object_handles = init_grid()
    grid_size = 0.5
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint, object_handles = select_random_waypoint(object_handles)
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Move the sphere object towards the next waypoint
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
            
        #remove grid cells at quadcopter pos
        quadcopter = sim.getObject("/Quadcopter")
        quad_pos = sim.getObjectPosition(quadcopter, -1)
        i,j = get_grid_coordinates(quad_pos)
        remove_from_grid(i, j, object_handles)
        
        # Check if the sphere object has reached the waypoint
        wi, wj = get_grid_coordinates(next_waypoint)
        at_waypoint = i == wi and j == wj

        # Remove the blue cube after the sphere object reaches the waypoint
        if at_waypoint:
            next_waypoint = None
        
        sim.switchThread()
'''

'''
#[25] refactor to use numpy
def add_masking_cube(i,j,grid_size):
    floor = sim.getObject("/Floor")
    cell = [grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2]         
    cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
    sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
    sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    return cube_handle

def init_grid():
    floor_size = [5, 5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    object_handles = np.empty((num_rows, num_cols), dtype=np.int32)
    for i in range(num_rows):
        for j in range(num_cols):
            cube_handle = add_masking_cube(i,j,grid_size)
            object_handles[i][j] = int(cube_handle)
    return object_handles

def get_grid_coordinates(world_pos):
    floor_size = [5, 5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    i = int((world_pos[0] + 2.5) / grid_size)
    j = int((world_pos[1] + 2.5) / grid_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] != -1:
        object_handle = int(grid[i][j])
        sim.addLog(0, "grid[{}][{}]={}".format(i, j, grid[i][j]))
        grid[i][j] = -1
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(object_handles):
    # Flatten the 2D array of object handles into a 1D list
    flat_handles = object_handles[object_handles != -1].tolist()
    # Choose a random unvisited cell as the next waypoint
    if len(flat_handles) == 0:
        object_handles = init_grid()
        return select_random_waypoint(object_handles)
    
    waypoint_handle = random.choice(flat_handles)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return waypoint_handle, next_waypoint, object_handles

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    # Get a list of all unvisited cells
    object_handles = init_grid()
    grid_size = 0.5
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint, object_handles = select_random_waypoint(object_handles)
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Move the sphere object towards the next waypoint
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
            
        #remove grid cells at quadcopter pos
        quadcopter = sim.getObject("/Quadcopter")
        quad_pos = sim.getObjectPosition(quadcopter, -1)
        i,j = get_grid_coordinates(quad_pos)
        remove_from_grid(i, j, object_handles)
        
        # Check if the sphere object has reached the waypoint
        wi, wj = get_grid_coordinates(next_waypoint)
        at_waypoint = i == wi and j == wj

        # Remove the blue cube after the sphere object reaches the waypoint
        if at_waypoint:
            next_waypoint = None
        
        sim.switchThread()

'''


'''
#[24A] cleanup code v1 + unmasking occurs from quadcopter
def init_grid():
    floor_size = [5, 5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    floor = sim.getObject("/Floor")
    object_handles = [[None for _ in range(num_cols)] for __ in range(num_rows)]
    for i in range(num_rows):
        for j in range(num_cols):
            cell = [grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2]
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
            sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
            sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
            sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
            object_handles[i][j] = cube_handle
    return object_handles


def get_grid_coordinates(world_pos):
    floor_size = [5, 5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    i = int((world_pos[0] + 2.5) / grid_size)
    j = int((world_pos[1] + 2.5) / grid_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] is not None:
        object_handle = grid[i][j]
        sim.addLog(0,"grid[{}][{}]={}".format(i,j,grid[i][j]))
        grid[i][j] = None
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(object_handles):
    # Flatten the 2D array of object handles into a 1D list
    flat_handles = [handle for row in object_handles for handle in row if handle is not None]
    
    # Choose a random unvisited cell as the next waypoint
    if len(flat_handles) == 0:
        #return None, None, None
        object_handles = init_grid()
        return select_random_waypoint(object_handles)
    
    waypoint_handle = random.choice(flat_handles)
    sim.addLog(0,"waypoint: {}".format(waypoint_handle))
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return waypoint_handle, next_waypoint, object_handles

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    # Get a list of all unvisited cells
    object_handles = init_grid()
    grid_size = 0.5
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint, object_handles = select_random_waypoint(object_handles)
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Move the sphere object towards the next waypoint
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
            
        #remove grid cells at quadcopter pos
        quadcopter = sim.getObject("/Quadcopter")
        quad_pos = sim.getObjectPosition(quadcopter, -1)
        i,j = get_grid_coordinates(quad_pos)
        remove_from_grid(i, j, object_handles)
        
        # Check if the sphere object has reached the waypoint
        wi, wj = get_grid_coordinates(next_waypoint)
        at_waypoint = i == wi and j == wj

        # Remove the blue cube after the sphere object reaches the waypoint
        if at_waypoint:
            next_waypoint = None
        
        sim.switchThread()
'''


'''
#[24] cleanup code v1 + unmasking occurs from quadcopter
def init_grid():
    floor_size = [5, 5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    floor = sim.getObject("/Floor")
    object_handles = [[None for _ in range(num_cols)] for __ in range(num_rows)]
    for i in range(num_rows):
        for j in range(num_cols):
            cell = [grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2]
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
            sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
            sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
            sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
            object_handles[i][j] = cube_handle
    return object_handles


def get_grid_coordinates(world_pos):
    floor_size = [5, 5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    i = int((world_pos[0] + 2.5) / grid_size)
    j = int((world_pos[1] + 2.5) / grid_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] is not None:
        object_handle = grid[i][j]
        sim.addLog(0,"grid[{}][{}]={}".format(i,j,grid[i][j]))
        grid[i][j] = None
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)

def select_random_waypoint(object_handles):
    # Flatten the 2D array of object handles into a 1D list
    flat_handles = [handle for row in object_handles for handle in row if handle is not None]
    
    # Choose a random unvisited cell as the next waypoint
    if len(flat_handles) == 0:
        #return None, None, None
        object_handles = init_grid()
        return select_random_waypoint(object_handles)
    
    waypoint_handle = random.choice(flat_handles)
    sim.addLog(0,"waypoint: {}".format(waypoint_handle))
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return waypoint_handle, next_waypoint, object_handles

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    # Get a list of all unvisited cells
    object_handles = init_grid()
    grid_size = 0.5
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint, object_handles = select_random_waypoint(object_handles)
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Move the sphere object towards the next waypoint
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
            
        # Check if the sphere object has reached the waypoint
        #result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        #at_waypoint = distanceData[2] <= 0.4
        #i,j = get_grid_coordinates(new_pos)
        quadcopter = sim.getObject("/Quadcopter")
        quad_pos = sim.getObjectPosition(quadcopter, -1)
        i,j = get_grid_coordinates(quad_pos)
        remove_from_grid(i, j, object_handles)
        wi, wj = get_grid_coordinates(next_waypoint)
        at_waypoint = i == wi and j == wj

 
        
            
        # Remove the blue cube after the sphere object reaches the waypoint
        if at_waypoint:
            #sim.removeObject(waypoint_handle)
            #waypoint_pos = sim.getObjectPosition(waypoint_handle, -1)
            #i,j = get_grid_coordinates(waypoint_pos)
            #remove_from_grid(i, j, object_handles)
            next_waypoint = None
        
        sim.switchThread()
'''

'''
#[23] works for multiple patrols
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    grid_coords = []
    for i in range(num_rows):
        for j in range(num_cols):
            grid_coords.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return grid_coords

def get_grid_coordinates(world_pos):
    floor_size = [5, 5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    i = int((world_pos[0] + 2.5) / grid_size)
    j = int((world_pos[1] + 2.5) / grid_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] is not None:
        object_handle = grid[i][j]
        sim.addLog(0,"grid[{}][{}]={}".format(i,j,grid[i][j]))
        grid[i][j] = None
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)
    

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    object_handles = [[None for _ in range(num_cols)] for __ in range(num_rows)]
    for i in range(num_rows):
        for j in range(num_cols):
            cell = cells[i * num_cols + j]
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
            sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
            sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
            sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
            unvisited_cells.append(cube_handle)
            object_handles[i][j] = cube_handle
    return unvisited_cells, object_handles

def select_random_waypoint(object_handles):
    # Flatten the 2D array of object handles into a 1D list
    flat_handles = [handle for row in object_handles for handle in row if handle is not None]
    
    # Choose a random unvisited cell as the next waypoint
    if len(flat_handles) == 0:
        #return None, None, None
        _, object_handles = init_unvisited()
        return select_random_waypoint(object_handles)
    
    waypoint_handle = random.choice(flat_handles)
    sim.addLog(0,"waypoint: {}".format(waypoint_handle))
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return waypoint_handle, next_waypoint, object_handles

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    # Get a list of all unvisited cells
    unvisited_cells, object_handles = init_unvisited()
    grid_size = 0.5
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        if len(object_handles) == 0:
            unvisited_cells, object_handles = init_unvisited()
            unvisted_cells = object_handles
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint, unvisited_cells = select_random_waypoint(object_handles)
            object_handles = unvisited_cells
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Move the sphere object towards the next waypoint
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
        i,j = get_grid_coordinates(new_pos)
        remove_from_grid(i, j, object_handles)
        #if not sim.isHandle(waypoint_handle):
        #    next_waypoint = None
        sim.switchThread()
            
        # Check if the sphere object has reached the waypoint
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        at_waypoint = distanceData[2] <= 0.4
            
        # Remove the blue cube after the sphere object reaches the waypoint
        if at_waypoint:
            #sim.removeObject(waypoint_handle)
            waypoint_pos = sim.getObjectPosition(waypoint_handle, -1)
            i,j = get_grid_coordinates(waypoint_pos)
            remove_from_grid(i, j, object_handles)
            next_waypoint = None
'''

'''
#[23] works but breaks after one patrol
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    grid_coords = []
    for i in range(num_rows):
        for j in range(num_cols):
            grid_coords.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return grid_coords

def get_grid_coordinates(world_pos):
    floor_size = [5, 5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    i = int((world_pos[0] + 2.5) / grid_size)
    j = int((world_pos[1] + 2.5) / grid_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] is not None:
        object_handle = grid[i][j]
        sim.addLog(0,"grid[{}][{}]={}".format(i,j,grid[i][j]))
        grid[i][j] = None
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)
    

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    object_handles = [[None for _ in range(num_cols)] for __ in range(num_rows)]
    for i in range(num_rows):
        for j in range(num_cols):
            cell = cells[i * num_cols + j]
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
            sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
            sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
            sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
            unvisited_cells.append(cube_handle)
            object_handles[i][j] = cube_handle
    return unvisited_cells, object_handles

def select_random_waypoint(object_handles):
    # Flatten the 2D array of object handles into a 1D list
    flat_handles = [handle for row in object_handles for handle in row if handle is not None]
    
    # Choose a random unvisited cell as the next waypoint
    if len(flat_handles) == 0:
        return None, None, None
    
    waypoint_handle = random.choice(flat_handles)
    sim.addLog(0,"waypoint: {}".format(waypoint_handle))
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return waypoint_handle, next_waypoint, object_handles

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    # Get a list of all unvisited cells
    unvisited_cells, object_handles = init_unvisited()
    grid_size = 0.5
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint, unvisited_cells = select_random_waypoint(object_handles)
            object_handles = unvisited_cells
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Move the sphere object towards the next waypoint
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
        i,j = get_grid_coordinates(new_pos)
        remove_from_grid(i, j, object_handles)
        #if not sim.isHandle(waypoint_handle):
        #    next_waypoint = None
        sim.switchThread()
            
        # Check if the sphere object has reached the waypoint
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        at_waypoint = distanceData[2] <= 0.4
            
        # Remove the blue cube after the sphere object reaches the waypoint
        if at_waypoint:
            #sim.removeObject(waypoint_handle)
            waypoint_pos = sim.getObjectPosition(waypoint_handle, -1)
            i,j = get_grid_coordinates(waypoint_pos)
            remove_from_grid(i, j, object_handles)
            next_waypoint = None

'''


'''
#[22] works but breaks, need to unify the list and grid into one management of object handles
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    grid_coords = []
    for i in range(num_rows):
        for j in range(num_cols):
            grid_coords.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return grid_coords

def get_grid_coordinates(world_pos):
    floor_size = [5, 5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    i = int((world_pos[0] + 2.5) / grid_size)
    j = int((world_pos[1] + 2.5) / grid_size)
    return i, j

def remove_from_grid(i, j, grid):
    if grid[i][j] is not None:
        object_handle = grid[i][j]
        sim.addLog(0,"grid[{}][{}]={}".format(i,j,grid[i][j]))
        grid[i][j] = None
        if sim.isHandle(object_handle):
            sim.removeObject(object_handle)
    

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    object_handles = [[None for _ in range(num_cols)] for __ in range(num_rows)]
    for i in range(num_rows):
        for j in range(num_cols):
            cell = cells[i * num_cols + j]
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
            sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
            sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
            sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
            unvisited_cells.append(cube_handle)
            object_handles[i][j] = cube_handle
    return unvisited_cells, object_handles

def select_random_waypoint(unvisited_cells, object_handles):
    # Choose a random unvisited cell as the next waypoint
    if len(unvisited_cells) == 0:
        unvisited_cells, object_handles = init_grid()
    
    waypoint_handle = random.choice(unvisited_cells)
    unvisited_cells.remove(waypoint_handle)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return (waypoint_handle, next_waypoint, unvisited_cells, object_handles)

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    # Get a list of all unvisited cells
    unvisited_cells, object_handles = init_unvisited()
    grid_size = 0.5
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint, unvisited_cells, object_handles = select_random_waypoint(unvisited_cells, object_handles)
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Move the sphere object towards the next waypoint
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
        i,j = get_grid_coordinates(new_pos)
        remove_from_grid(i, j, object_handles)
        sim.switchThread()
            
        # Check if the sphere object has reached the waypoint
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        at_waypoint = distanceData[2] <= 0.4
            
        # Remove the blue cube after the sphere object reaches the waypoint
        if at_waypoint:
            sim.removeObject(waypoint_handle)
            next_waypoint = None
'''


'''
#[21] converted unvisisted into 2d array
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    unvisited_cells = []
    for i in range(num_rows):
        for j in range(num_cols):
            unvisited_cells.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return unvisited_cells

def remove_unvisited(object_handle, grid_size, num_rows, num_cols, object_handles):
    position = sim.getObjectPosition(object_handle, -1)
    i = int((position[0] + 2.5) / grid_size)
    j = int((position[1] + 2.5) / grid_size)
    object_handles[i][j] = None
    sim.removeObject(object_handle)

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    object_handles = [[None for _ in range(num_cols)] for __ in range(num_rows)]
    for i in range(num_rows):
        for j in range(num_cols):
            cell = cells[i * num_cols + j]
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
            sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
            sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
            sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
            unvisited_cells.append(cube_handle)
            object_handles[i][j] = cube_handle
    return unvisited_cells, object_handles

def select_random_waypoint(unvisited_cells, object_handles):
    # Choose a random unvisited cell as the next waypoint
    if len(unvisited_cells) == 0:
        unvisited_cells, object_handles = init_unvisited()
    
    waypoint_handle = random.choice(unvisited_cells)
    unvisited_cells.remove(waypoint_handle)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return (waypoint_handle, next_waypoint, unvisited_cells, object_handles)

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    # Get a list of all unvisited cells
    unvisited_cells, object_handles = init_unvisited()
    grid_size = 0.5
    floor_size = [5, 5]
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint, unvisited_cells, object_handles = select_random_waypoint(unvisited_cells, object_handles)
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Move the sphere object towards the next waypoint
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
        sim.switchThread()
            
        # Check if the sphere object has reached the waypoint
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        at_waypoint = distanceData[2] <= 0.4
            
        # Remove the blue cube after the sphere object reaches the waypoint
        if at_waypoint:
            remove_unvisited(waypoint_handle, grid_size, num_rows, num_cols, object_handles)
            next_waypoint = None

'''




'''
#[20] - touch cells before making a new waypoint, need to remove all cells that are passed over
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    grid_coords = []
    for i in range(num_rows):
        for j in range(num_cols):
            grid_coords.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return grid_coords

def init_unvisited():
    grid_coords = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    for grid_coord in grid_coords:
        cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
        sim.setObjectPosition(cube_handle, floor, [grid_coord[0], grid_coord[1], 0.15])
        sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
        sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
        unvisited_cells.append(cube_handle)
    return unvisited_cells

def select_random_waypoint(unvisited_cells):
    # Choose a random unvisited cell as the next waypoint
    if len(unvisited_cells) == 0:
        unvisited_cells = init_unvisited()
    
    waypoint_handle = random.choice(unvisited_cells)
    unvisited_cells.remove(waypoint_handle)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return (waypoint_handle, next_waypoint)

def convert_to_grid_coords(world_coords, grid_size):
    return [int(world_coords[0] / grid_size), int(world_coords[1] / grid_size)]

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    grid_size = 0.5
    # Get a list of all unvisited cells
    unvisited_cells = init_unvisited()
    # Set the thread to be synchronized
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint = select_random_waypoint(unvisited_cells)
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Move the sphere object towards the next waypoint
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
        sim.switchThread()
            
        # Check if the sphere object has reached the waypoint
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        at_waypoint = distanceData[2] <= 0.4
            
        # Remove the blue cube after the sphere object reaches the waypoint
        if at_waypoint:
            sim.removeObject(waypoint_handle)
            if waypoint_handle in unvisited_cells:
                unvisited_cells.remove(waypoint_handle)
            next_waypoint = None

'''



'''
#[19A] - Broken - refactored with boolean 2d, doesn't work
def get_cell_count(width=5,height=5,cell_size=0.5):
    num_rows = int(width / cell_size)
    num_cols = int(height / cell_size)
    return num_rows, num_cols

# Create a 2D array with n rows and n columns filled with False values
def init_visited_matrix2d(nrows, ncols):
    mask = np.full((nrows, ncols), False, dtype=bool)
    return mask

def get_world_position(pos, nrows, ncols, cell_size):
    x = pos[0] * cell_size - 2.5 + cell_size / 2
    y = pos[1] * cell_size - 2.5 + cell_size / 2
    return x, y

def get_grid_position(pos, nrows, ncols, cell_size):
    i = int((pos[0] + 2.5) / cell_size)
    j = int((pos[1] + 2.5) / cell_size)
    return i, j


def toggle_mask_cell(mask, pos, cell_size):
    i, j = get_grid_position(pos, mask.shape[0], mask.shape[1], cell_size)
    mask[i, j] = True

def select_random_waypoint(visited_matrix2d):
    indices = np.where(visited_matrix2d == False)
    if indices[0].size == 0:
        visited_matrix2d = np.full((num_rows, num_cols), False, dtype=bool)
        indices = np.where(visited_matrix2d == False)
    idx = np.random.choice(indices[0].size)
    i, j = indices[0][idx], indices[1][idx]
    return i,j

def draw_mask(mask,cell_size=0.5):
    mask_handles = []
    floor = sim.getObject("/Floor")
    for i in range(num_rows):
        for j in range(num_cols):
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
            world_pos = [cell_size * i - 2.5 + cell_size / 2, cell_size * j - 2.5 + cell_size / 2, 0.15]
            sim.setObjectPosition(cube_handle, floor, world_pos)
            sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
            sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
            mask_handles.append(cube_handle)
    return mask_handles

def get_path(src, dst, grid):
    path = []
    num_rows, num_cols = grid.shape
    # Check if src and dst are within the bounds of the grid
    if src[0] < 0 or src[0] >= num_rows or src[1] < 0 or src[1] >= num_cols:
        return path
    if dst[0] < 0 or dst[0] >= num_rows or dst[1] < 0 or dst[1] >= num_cols:
        return path
    # Check if src and dst are the same cell
    if src == dst:
        return path
    # Use Bresenham's line algorithm to get cells between src and dst
    x0, y0 = src
    x1, y1 = dst
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        path.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return path


def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 0.5  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    visited_matrix2d = init_visited_matrix2d(5, 5)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None: #np.all(visited_matrix2d):
            visited_matrix2d = init_visited_matrix2d(5, 5)
            next_waypoint = select_random_waypoint(visited_matrix2d)
        
        
        #get waypoint world coordinates
        x,y = get_world_position(next_waypoint, visited_matrix2d.shape[0], visited_matrix2d.shape[1], 0.5)
        next_waypoint = [x, y, 0.5]
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
                
        src_cell = get_grid_position(current_pos, visited_matrix2d.shape[0], visited_matrix2d.shape[1], 0.5)
        dst_cell = next_waypoint
        
        path = get_path(src_cell, dst_cell, visited_matrix2d)
        #check location 
        while (path):
            next_step = path.pop()
            at_cell = False
            while not at_cell:
                #move the sphere towards the next cell until it reaches the cell
                obj_pos = sim.getObjectPosition(objHandle, -1)
                if obj_pos[0] >= next_step_world[0]-0.5 and obj_pos[0] <= next_step_world[0]+0.5 and obj_pos[1] >= next_step_world[1]-0.5 and obj_pos[1] <= next_step_world[1]+0.5:
                    at_cell = True

            #set this location to true
            toggle_mask_cell(visited_matrix2d, next_step, 0.5)
            draw_mask(visited_matrix2d, 0.5)
'''



'''
#[19] - Broken - refactored with boolean 2d, doesn't work
def get_cell_count(width=5,height=5,cell_size=0.5):
    num_rows = int(width / grid_size)
    num_cols = int(height / grid_size)
    return num_rows, num_cols

# Create a 2D array with n rows and n columns filled with False values
def init_visited_matrix2d(nrows, ncols):
    mask = np.full((nrows, ncols), False, dtype=bool)
    return mask

def get_world_position(pos, nrows, ncols, cell_size):
    x = pos[0] * cell_size - 2.5 + cell_size / 2
    y = pos[1] * cell_size - 2.5 + cell_size / 2
    return x, y

def get_grid_position(pos, nrows, ncols, cell_size):
    i = int((pos[0] + 2.5) / cell_size)
    j = int((pos[1] + 2.5) / cell_size)
    return i, j


def toggle_mask_cell(mask, pos, cell_size):
    i, j = get_grid_location(pos, mask.shape[0], mask.shape[1], cell_size)
    mask[i, j] = True

def select_random_waypoint(visited_matrix2d):
    indices = np.where(visited_matrix2d == False)
    if indices[0].size == 0:
        visited_matrix2d = np.full((num_rows, num_cols), False, dtype=bool)
        indices = np.where(visited_matrix2d == False)
    idx = np.random.choice(indices[0].size)
    i, j = indices[0][idx], indices[1][idx]
    return i,j

def draw_mask(mask,cell_size=0.5):
    mask_handles = 90
    floor = sim.getObject("/Floor")
    for i in range(num_rows):
        for j in range(num_cols):
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
            world_pos = [cell_size * i - 2.5 + cell_size / 2, cell_size * j - 2.5 + cell_size / 2, 0.15]
            sim.setObjectPosition(cube_handle, floor, world_pos)
            sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
            sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
            mask_handles = []
    return mask_handles

def get_path(src, dst, grid):
    path = []
    num_rows, num_cols = grid.shape
    # Check if src and dst are within the bounds of the grid
    if src[0] < 0 or src[0] >= num_rows or src[1] < 0 or src[1] >= num_cols:
        return path
    if dst[0] < 0 or dst[0] >= num_rows or dst[1] < 0 or dst[1] >= num_cols:
        return path
    # Check if src and dst are the same cell
    if src == dst:
        return path
    # Use Bresenham's line algorithm to get cells between src and dst
    x0, y0 = src
    x1, y1 = dst
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        path.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
    return path


def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 0.5  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    visited_matrix2d = init_visited_matrix2d(5, 5)
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None: #np.all(visited_matrix2d):
            visited_matrix2d = init_visited_matrix2d(5, 5)
            next_waypoint = select_random_waypoint(visited_matrix2d)
        
        
        #get waypoint world coordinates
        x,y = get_world_position(next_waypoint, visited_matrix2d.shape[0], visited_matrix2d.shape[1], 0.5)
        next_waypoint = [x, y, 0.5]
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        src_cell = get_grid_position(current_pos)
        dst_cell = next_waypoint
        
        path = get_path(src_cell, dst_cell, visited_matrix2d)
        #check location 
        while (path):
            next_step = path.pop()
            at_cell = False
            while not at_cell:
                #move the sphere towards the next cell until it reaches the cell
                obj_pos = sim.getObjectPosition(objHandle, -1)
                next_step_world = get_world_position(next_step, visited_matrix2d.shape[0], visited_matrix2d.shape[1], 0.5)
                direction = [next_step_world[0] - obj_pos[0], next_step_world[1] - obj_pos[1], 0]
                sim.setObjectPosition(objHandle, -1, [obj_pos[0]+speed*direction[0], obj_pos[1]+speed*direction[1], obj_pos[2]+speed*direction[2]])
                #check if object is in next_step cell
                if obj_pos[0] >= next_step_world[0]-0.5 and obj_pos[0] <= next_step_world[0]+0.5 and obj_pos[1] >= next_step_world[1]-0.5 and obj_pos[1] <= next_step_world[1]+0.5:
                    at_cell = True
            
            #set this location to true
            toggle_mask_cell(visited_matrix2d, next_step, 0.5)
            draw_mask(visited_matrix2d, 0.5)
'''


            
        
       
'''
#[18] - touch cells before making a new waypoint, need to remove all cells that are passed over
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    unvisited_cells = []
    for i in range(num_rows):
        for j in range(num_cols):
            unvisited_cells.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return unvisited_cells

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    for cell in cells:
        cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
        sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
        sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
        sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
        unvisited_cells.append(cube_handle)
    return unvisited_cells

def select_random_waypoint(unvisited_cells):
    # Choose a random unvisited cell as the next waypoint
    if len(unvisited_cells) == 0:
        unvisited_cells = init_unvisited()
    
    waypoint_handle = random.choice(unvisited_cells)
    unvisited_cells.remove(waypoint_handle)
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Toggle mask cell into waypoint cell as blue 
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    return (waypoint_handle, next_waypoint)

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 0.5  # Specify the speed of the sphere object in m/s
    waypoint_handle, next_waypoint = None, None
    # Get a list of all unvisited cells
    unvisited_cells = init_unvisited()
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if next_waypoint is None:
            waypoint_handle, next_waypoint = select_random_waypoint(unvisited_cells)
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(time_to_waypoint / sim.getSimulationTimeStep())
        
        # Move the sphere object towards the next waypoint in steps
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
        sim.switchThread()
            
        # Check if the sphere object has reached the waypoint
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        at_waypoint = distanceData[2] <= 0.4
        sim.addLog(0, "distance: {}".format(distanceData[2]))
        # Check if the sphere object has reached the waypoint
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        at_waypoint = distanceData[2] <= 0.4
        sim.addLog(0, "distance: {}".format(distanceData[2]))
            
        # Remove the blue cube after the sphere object reaches the waypoint
        if at_waypoint:
            sim.removeObject(waypoint_handle)
            next_waypoint = None
'''



'''
#[17] Works, but its jerky in speed
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    unvisited_cells = []
    for i in range(num_rows):
        for j in range(num_cols):
            unvisited_cells.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return unvisited_cells

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    for cell in cells:
        cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
        sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
        sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
        sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
        unvisited_cells.append(cube_handle)
    return unvisited_cells

def move_to_waypoint(objHandle, next_waypoint, speed, step_size, steps):
    for i in range(steps):
        current_pos = sim.getObjectPosition(objHandle, -1)
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
        sim.switchThread()

def select_random_waypoint(unvisited_cells):
        # Choose a random unvisited cell as the next waypoint
        if len(unvisited_cells) == 0:
            unvisited_cells = init_unvisited()
            
        waypoint_handle = random.choice(unvisited_cells)
        unvisited_cells.remove(waypoint_handle)
        next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
        # Toggle mask cell into waypoint cell as blue 
        sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
        return (waypoint_handle, next_waypoint)

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 0.5  # Specify the speed of the sphere object in m/s
    # Get a list of all unvisited cells
    unvisited_cells = init_unvisited()
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        waypoint_handle, next_waypoint = select_random_waypoint(unvisited_cells)
        
        at_waypoint = False 
        while not at_waypoint:
            # Get the current position of the sphere object
            current_pos = sim.getObjectPosition(objHandle, -1)
            
            result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
            distance = distanceData[2]
            
            # Calculate the time it takes to reach the next waypoint
            time_to_waypoint = distance / speed
            
            # Calculate the step size for each simulation step
            step_size = speed * sim.getSimulationTimeStep()
            
            # Calculate the number of steps required to reach the next waypoint
            steps = int(time_to_waypoint / sim.getSimulationTimeStep())
            
            # Move the sphere object towards the next waypoint in steps
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) * step_size,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) * step_size,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
            
            # Check if the sphere object has reached the waypoint
            result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
            at_waypoint = distanceData[2] <= 0.4
            sim.addLog(0, "distance: {}".format(distanceData[2]))
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)
'''

'''
#[16] - functional decomposition for select_random_waypoint
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    unvisited_cells = []
    for i in range(num_rows):
        for j in range(num_cols):
            unvisited_cells.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return unvisited_cells

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    for cell in cells:
        cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
        sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
        sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
        sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
        unvisited_cells.append(cube_handle)
    return unvisited_cells

def move_to_waypoint(objHandle, next_waypoint, speed, step_size, steps):
    for i in range(steps):
        current_pos = sim.getObjectPosition(objHandle, -1)
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
        sim.switchThread()

def select_random_waypoint(unvisited_cells):
        # Choose a random unvisited cell as the next waypoint
        if len(unvisited_cells) == 0:
            unvisited_cells = init_unvisited()
            
        waypoint_handle = random.choice(unvisited_cells)
        unvisited_cells.remove(waypoint_handle)
        next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
        # Toggle mask cell into waypoint cell as blue 
        sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
        return (waypoint_handle, next_waypoint)

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 0.5  # Specify the speed of the sphere object in m/s
    # Get a list of all unvisited cells
    unvisited_cells = init_unvisited()
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        # Choose a random unvisited cell as the next waypoint
        waypoint_handle, next_waypoint = select_random_waypoint(unvisited_cells)
            
        result, distanceData, objectHandlePair = sim.checkDistance(objHandle, waypoint_handle, 0)
        distance = distanceData[2]
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)
'''

'''
#[15] - functional decomposition 
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    unvisited_cells = []
    for i in range(num_rows):
        for j in range(num_cols):
            unvisited_cells.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return unvisited_cells

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    for cell in cells:
        cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
        sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
        sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
        sim.setObjectSpecialProperty(cube_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
        unvisited_cells.append(cube_handle)
    return unvisited_cells

def move_to_waypoint(objHandle, next_waypoint, speed, step_size, steps):
    for i in range(steps):
        current_pos = sim.getObjectPosition(objHandle, -1)
        new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                   current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                   current_pos[2]]
        sim.setObjectPosition(objHandle, -1, new_pos)
        sim.switchThread()

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1.5  # Specify the speed of the sphere object in m/s
    # Get a list of all unvisited cells
    unvisited_cells = init_unvisited()
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Choose a random unvisited cell as the next waypoint
        if len(unvisited_cells) == 0:
            unvisited_cells = init_unvisited()
            
        waypoint_handle = random.choice(unvisited_cells)
        unvisited_cells.remove(waypoint_handle)
        next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
            
        # Toggle mask cell into waypoint cell as blue 
        sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
        
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        move_to_waypoint(objHandle, next_waypoint, speed, step_size, steps)
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)
'''





'''
#[14] Bug: Target doesn't touch waypoint before moving on to next waypoint
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    unvisited_cells = []
    for i in range(num_rows):
        for j in range(num_cols):
            unvisited_cells.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return unvisited_cells

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    for cell in cells:
        cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
        sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
        sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
        unvisited_cells.append(cube_handle)
    return unvisited_cells

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1.5  # Specify the speed of the sphere object in m/s
    # Get a list of all unvisited cells
    unvisited_cells = init_unvisited()
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        # Choose a random unvisited cell as the next waypoint
        if len(unvisited_cells) == 0:
            unvisited_cells = init_unvisited()
            
        waypoint_handle = random.choice(unvisited_cells)
        unvisited_cells.remove(waypoint_handle)
        next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
            
        # Toggle mask cell into waypoint cell as blue 
        sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)
'''

'''
#[13]
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    unvisited_cells = []
    for i in range(num_rows):
        for j in range(num_cols):
            unvisited_cells.append([grid_size * i - 2.5 + grid_size / 2, grid_size * j - 2.5 + grid_size / 2])
    return unvisited_cells

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    for cell in cells:
        cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
        sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
        sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
        unvisited_cells.append(cube_handle)
    return unvisited_cells

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1.5  # Specify the speed of the sphere object in m/s
    # Get a list of all unvisited cells
    unvisited_cells = init_unvisited()
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        # Choose a random unvisited cell as the next waypoint
        if len(unvisited_cells) == 0:
            unvisited_cells = init_unvisited()
            
        waypoint_handle = random.choice(unvisited_cells)
        unvisited_cells.remove(waypoint_handle)
        next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
            
        # Toggle mask cell into waypoint cell as blue 
        sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()

        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)
'''

'''
#[12]
def init_grid():
    floor_size = [5,5]
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    unvisited_cells = []
    for i in range(num_rows):
        for j in range(num_cols):
            unvisited_cells.append([grid_size * i - 2.25 + grid_size / 2, grid_size * j - 2.25 + grid_size / 2])
    return unvisited_cells

def init_unvisited():
    cells = init_grid()
    floor = sim.getObject("/Floor")
    grid_size = 0.5
    unvisited_cells = []
    for cell in cells:
        cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
        sim.setObjectPosition(cube_handle, floor, [cell[0], cell[1], 0.15])
        sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
        unvisited_cells.append(cube_handle)
    return unvisited_cells

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1.5  # Specify the speed of the sphere object in m/s
    # Get a list of all unvisited cells
    unvisited_cells = init_unvisited()
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        # Choose a random unvisited cell as the next waypoint
        if len(unvisited_cells) == 0:
            unvisited_cells = init_unvisited()
            
        waypoint_handle = random.choice(unvisited_cells)
        next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
        unvisited_cells.remove(waypoint_handle)
            
        # Toggle mask cell into waypoint cell as blue 
        
        sim.setObjectPosition(waypoint_handle, objHandle, [next_waypoint[0], next_waypoint[1], 0])
        sim.setShapeColor(waypoint_handle, None,0,[0, 0, 1])
        sim.setObjectSpecialProperty(waypoint_handle, 0)
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)
'''



'''
[11]
def mask_unvisited():
    floor = sim.getObject("/Floor")
    floor_size = [5,5] #sim.getObjectFloatParameter(floor, 15)
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    cells = []
    for i in range(num_rows):
        for j in range(num_cols):
            cells.append([grid_size * i-2.25, grid_size * j-2.25])
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
            sim.setObjectPosition(cube_handle, floor, [grid_size * i-2.25, grid_size * j-2.25, 0.15])
            sim.setShapeColor(cube_handle, None, 0, [0.1, 0.1, 0.1])
    return cells

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1.5  # Specify the speed of the sphere object in m/s
    
    # Draw the grid of primitive cuboids on the floor
    cells = mask_unvisited()
    
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        sim.addLog(0, "drone    - x: {} y: {}".format(current_pos[0], current_pos[1]))
        
        # Generate a random x, y position for the next waypoint
        next_waypoint = random.choice(cells)
        
        # Add a blue cube at the target location of the waypoint
        waypoint_handle =  sim.createPrimitiveShape (sim.primitiveshape_cuboid, [0.5, 0.5, 0],1)
        
        sim.setObjectPosition(waypoint_handle, objHandle, [next_waypoint[0], next_waypoint[1], 0])
        sim.setShapeColor(waypoint_handle, None,0,[0, 0, 1])
        sim.setObjectSpecialProperty(waypoint_handle, 0)
        sim.addLog(0, "waypoint - handle: {}".format(waypoint_handle))
        sim.addLog(0, "waypoint - x: {} y: {}".format(next_waypoint[0], next_waypoint[1]))
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
                # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)
'''

'''
[10]
def mask_unvisited():
    floor = sim.getObject("/Floor")
    floor_size = [5,5] #sim.getObjectFloatParameter(floor, 15)
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    for i in range(num_rows):
        for j in range(num_cols):
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
            sim.setObjectPosition(cube_handle, floor, [grid_size * i-2.25, grid_size * j-2.25, 0.15])
            sim.setShapeColor(cube_handle, None, 0, [1, 1, 1])

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1.5  # Specify the speed of the sphere object in m/s
    
    # Draw the grid of primitive cuboids on the floor
    mask_unvisited()
    
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        sim.addLog(0, "drone    - x: {} y: {}".format(current_pos[0], current_pos[1]))
        
        # Generate a random x, y position for the next waypoint
        next_waypoint = [random.uniform(-2.5, 2.5), random.uniform(-2.5, 2.5)]
        
        # Add a blue cube at the target location of the waypoint
        waypoint_handle =  sim.createPrimitiveShape (sim.primitiveshape_cuboid, [0.5, 0.5, 0],1)
        
        sim.setObjectPosition(waypoint_handle, objHandle, [next_waypoint[0], next_waypoint[1], 0])
        sim.setShapeColor(waypoint_handle, None,0,[0, 0, 1])
        sim.setObjectSpecialProperty(waypoint_handle, 0)
        sim.addLog(0, "waypoint - handle: {}".format(waypoint_handle))
        sim.addLog(0, "waypoint - x: {} y: {}".format(next_waypoint[0], next_waypoint[1]))
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
                # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)
'''

'''
[9]
def draw_unvisited():
    floor = sim.getObject("/Floor")
    floor_size = [5,5] #sim.getObjectFloatParameter(floor, 15)
    grid_size = 0.5
    num_rows = int(floor_size[0] / grid_size)
    num_cols = int(floor_size[1] / grid_size)
    for i in range(num_rows):
        for j in range(num_cols):
            cube_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [grid_size, grid_size, 0], 1)
            sim.setObjectPosition(cube_handle, floor, [grid_size * i, grid_size * j, 0.5])
            sim.setShapeColor(cube_handle, None, 0, [1, 1, 1])

def sysCall_thread():
    objHandle = sim.getObject(".")
    speed = 1.5  # Specify the speed of the sphere object in m/s
    
    # Draw the grid of primitive cuboids on the floor
    draw_unvisited()
    
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        sim.addLog(0, "drone    - x: {} y: {}".format(current_pos[0], current_pos[1]))
        
        # Generate a random x, y position for the next waypoint
        next_waypoint = [random.uniform(-2.5, 2.5), random.uniform(-2.5, 2.5)]
        
        # Add a blue cube at the target location of the waypoint
        waypoint_handle =  sim.createPrimitiveShape (sim.primitiveshape_cuboid, [0.5, 0.5, 0],1)
        
        sim.setObjectPosition(waypoint_handle, objHandle, [next_waypoint[0], next_waypoint[1], 0])
        sim.setShapeColor(waypoint_handle, None,0,[0, 0, 1])
        sim.setObjectSpecialProperty(waypoint_handle, 0)
        sim.addLog(0, "waypoint - handle: {}".format(waypoint_handle))
        sim.addLog(0, "waypoint - x: {} y: {}".format(next_waypoint[0], next_waypoint[1]))
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
                # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)

'''


'''
[8]
def sysCall_thread():
    objHandle = sim.getObject(".")
    floor = sim.getObject("/Floor")
    speed = 1.5  # Specify the speed of the sphere object in m/s
    floor_size = sim.getObjectFloatParameter(floor, 15)
    
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        sim.addLog(0, "drone    - x: {} y: {}".format(current_pos[0], current_pos[1]))
        
        # Generate a random x, y position for the next waypoint
        next_waypoint = [random.uniform(-2.5, 2.5), random.uniform(-2.5, 2.5)]
        
        # Add a blue cube at the target location of the waypoint
        #waypoint_handle = sim.createPureShape(0, 0, [0.5, 0.5, 0], 0, [0, 0, 0])
        waypoint_handle =  sim.createPrimitiveShape (sim.primitiveshape_cuboid, [0.5, 0.5, 0],1)
        
        sim.setObjectPosition(waypoint_handle, objHandle, [next_waypoint[0], next_waypoint[1], 0])
        sim.setShapeColor(waypoint_handle, None,0,[0, 0, 1])
        sim.setObjectSpecialProperty(waypoint_handle, 0)
        sim.addLog(0, "waypoint - handle: {}".format(waypoint_handle))
        sim.addLog(0, "waypoint - x: {} y: {}".format(next_waypoint[0], next_waypoint[1]))
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)
'''

'''
[7]
def sysCall_thread():
    objHandle = sim.getObject(".")
    floor = sim.getObject("/Floor")
    speed = 1  # Specify the speed of the sphere object in m/s
    floor_size = sim.getObjectFloatParameter(floor, 15)
    
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        sim.addLog(0, "drone    - x: {} y: {}".format(current_pos[0], current_pos[1]))
        
        # Generate a random x, y position for the next waypoint
        next_waypoint = [random.uniform(-2.5, 2.5), random.uniform(-2.5, 2.5)]
        
        # Add a blue cube at the target location of the waypoint
        waypoint_handle = sim.createPureShape(0, 0, [0.5, 0.5, 0], 0, [0, 0, 0])
        sim.setObjectPosition(waypoint_handle, objHandle, [next_waypoint[0], next_waypoint[1], 0])
        sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
        sim.addLog(0, "waypoint - handle: {}".format(waypoint_handle))
        sim.addLog(0, "waypoint - x: {} y: {}".format(next_waypoint[0], next_waypoint[1]))
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)

'''

'''
[6]
def sysCall_thread():
    objHandle = sim.getObject(".")
    floor = sim.getObject("/Floor")
    speed = 1.5  # Specify the speed of the sphere object in m/s
    floor_size = sim.getObjectFloatParameter(floor, 15)
    
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        sim.addLog(0, "drone    - x: {} y: {}".format(current_pos[0], current_pos[1]))
        
        # Generate a random x, y position for the next waypoint
        next_waypoint = [random.uniform(0, 3), random.uniform(0, 3)]
        
        # Add a blue cube at the target location of the waypoint
        waypoint_handle = sim.createPureShape(0, 0, [0.5, 0.5, 0], 0, None, [0, 0, 0])
        sim.setObjectPosition(waypoint_handle, objHandle, [next_waypoint[0], next_waypoint[1], 0])
        #sim.setShapeColor(waypoint_handle, None, [0, 0, 1], None)
        sim.addLog(0, "waypoint - handle: {}".format(waypoint_handle))
        sim.addLog(0, "waypoint - x: {} y: {}".format(next_waypoint[0], next_waypoint[1]))
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)

'''

'''
[5]
def sysCall_thread():
    objHandle = sim.getObject(".")
    floor = sim.getObject("/Floor")
    speed = 1  # Specify the speed of the sphere object in m/s
    floor_size = sim.getObjectFloatParameter(floor, 15)
    
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        sim.addLog(0, "drone    - x: {} y: {}".format(current_pos[0], current_pos[1]))
        
        # Generate a random x, y position for the next waypoint
        next_waypoint = [random.uniform(-2.5, 2.5), random.uniform(-2.5, 2.5)]
        
        # Create a blue cube at the target location of the waypoint
        waypoint_handle = sim.createPureShape(0, 0, [0.2, 0.2, 0.2], 0, [0, 0, 1])
        sim.setObjectPosition(waypoint_handle, objHandle, [next_waypoint[0], next_waypoint[1], 0])
        sim.addLog(0, "waypoint - handle: {}".format(waypoint_handle))
        sim.addLog(0, "waypoint - x: {} y: {}".format(next_waypoint[0], next_waypoint[1]))
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue cube after the sphere object reaches the waypoint
        sim.removeObject(waypoint_handle)
'''

'''
#[4] blue object for target waypoint cell
def sysCall_thread():
    objHandle = sim.getObject(".")
    floor = sim.getObject("/Floor")
    speed = 1  # Specify the speed of the sphere object in m/s
    floor_size = sim.getObjectFloatParameter(floor, 15)
    
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        sim.addLog(0, "drone    - x: {} y: {}".format(current_pos[0], current_pos[1]))
        
        # Generate a random x, y position for the next waypoint
        next_waypoint = [random.uniform(-2.5, 2.5), random.uniform(-2.5, 2.5)]
        
        # Add a blue square at the target location of the waypoint
        waypoint_handle = sim.addDrawingObject(0, 0.2, 0, -1, 1, [0, 0, 1])
        sim.setObjectPosition(waypoint_handle, objHandle, [next_waypoint[0], next_waypoint[1], 0])
        sim.addLog(0, "waypoint - handle: {}".format(waypoint_handle))
        sim.addLog(0, "waypoint - x: {} y: {}".format(next_waypoint[0], next_waypoint[1]))
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
        
        # Remove the blue square after the sphere object reaches the waypoint
        sim.removeDrawingObject(waypoint_handle)

'''


'''
#[3] floor-based zone
def sysCall_thread():
    objHandle = sim.getObject(".")
    floor = sim.getObject("/Floor")
    speed = 1 #0.5  # Specify the speed of the sphere object in m/s
    floor_size = sim.getObjectFloatParameter(floor, 15)
    sim.addLog(0,str(floor_size[0]))
    sim.addLog(0,str(floor_size[1]))
    
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        # Generate a random x, y position for the next waypoint
        next_waypoint = [random.uniform(-3.5, 3.5), random.uniform(-3.5, 3.5)]
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()
'''
'''
#[2] continuous move 
def sysCall_thread():
    objHandle = sim.getObject(".")
    floor = sim.getObject("/Floor")
    speed = 0.5  # Specify the speed of the sphere object in m/s
    
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    while True:
        # Get the current position of the sphere object
        current_pos = sim.getObjectPosition(objHandle, -1)
        
        # Generate a random x, y position for the next waypoint
        next_waypoint = [random.uniform(0, 2), random.uniform(0, 2)]
        
        # Calculate the distance to the next waypoint
        distance = ((current_pos[0] - next_waypoint[0]) ** 2 + (current_pos[1] - next_waypoint[1]) ** 2) ** 0.5
        
        # Calculate the time it takes to reach the next waypoint
        time_to_waypoint = distance / speed
        
        # Calculate the step size for each simulation step
        step_size = speed * sim.getSimulationTimeStep()
        
        # Calculate the number of steps required to reach the next waypoint
        steps = int(distance / step_size)
        
        # Move the sphere object towards the next waypoint in steps
        for i in range(steps):
            current_pos = sim.getObjectPosition(objHandle, -1)
            new_pos = [current_pos[0] + (next_waypoint[0] - current_pos[0]) / steps,
                       current_pos[1] + (next_waypoint[1] - current_pos[1]) / steps,
                       current_pos[2]]
            sim.setObjectPosition(objHandle, -1, new_pos)
            sim.switchThread()

'''

'''
#[1] teleport
def sysCall_thread():
    objHandle=sim.getObject(".")
    floor = sim.getObject("/Floor")
    sim.addLog(0,"hello")
    
    # e.g. non-synchronized loop:
    sim.setThreadAutomaticSwitch(True)
    while True:
        p=sim.getObjectPosition(objHandle,-1)
        p[0]=  random.uniform(0, 2) #p[0]+0.001
        p[1]=  random.uniform(0, 2) #p[1]+0.001
        sim.setObjectPosition(objHandle,-1,p)
        time.sleep(1)
        
        
    # e.g. synchronized loop:
    # sim.setThreadAutomaticSwitch(False)
    # while True:
    #     p=sim.getObjectPosition(objHandle,-1)
    #     p[0]=p[0]+0.001
    #     sim.setObjectPosition(objHandle,-1,p)
    #     sim.switchThread() # resume in next simulation step
    #pass
'''


# See the user manual or the available code snippets for additional callback functions and details
