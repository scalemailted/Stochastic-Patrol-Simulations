#python
import random
import time
import numpy as np
import tempfile
from PIL import Image


#########################################################################
# Global datastore for simulation features
#
# Parameters:
#   prop (str): the property to retrieve
#
# Returns:
#   float or int: the value of the requested property
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


#########################################################################
# Main Thread for Simulation
#
# Initializes the grid, generates the enemies, moves the quadcopter and enemies
# checks if the quadcopter touched either the enemies or the waypoint.
    
def sysCall_thread():
    # Initialize the world floor
    init_floor2d()
    # Get a list of all unvisited cells in the grid
    mask2d = init_grid()
    # Generate the enemies in the simulation
    enemy_handles = init_enemies()
    # Get the target object for the quadcopter
    target_handle = sim.getObject(".")
    # Initialize control variables
    waypoint_handle, next_waypoint = None, None
    # Set the thread to be synchronized
    sim.setThreadAutomaticSwitch(False)
    # Initialize a list to store the handles of caught enemies
    caught_enemies = []
    # Run the loop
    while True:
        # Iterate through each enemy
        for enemy_handle in enemy_handles:
            # Check if the enemy has not already been caught
            if enemy_handle not in caught_enemies:
                # Move the enemy to a random adjacent cell
                move_to_random_adjacent_cell(enemy_handle, mask2d)
            # Check if the enemy has been caught by the observer
            if check_waypoint_reached(sim.getObjectPosition(enemy_handle, -1)):
                # Add any caught enemy to the list
                caught_enemies.append(enemy_handle)
                # Change the color of caught enemies to dark red (dead)
                sim.setShapeColor(enemy_handle, None, 0, [0.5, 0, 0])
                
        # Check status on whether there is a next waypoint
        if next_waypoint is None:
            # Select a random unvisited cell as the next waypoint
            next_waypoint, mask2d = select_random_waypoint(mask2d)
        
        # Move the observer towards the next waypoint
        move_to_waypoint(target_handle, next_waypoint, mask2d)
            
        # Remove the cell at the observer's position from the list of unvisited cells
        unmask_cell_at_observer(mask2d)
        
        # Check if the observer has reached the waypoint
        if check_waypoint_reached(next_waypoint):
            # Reset the next waypoint
            next_waypoint = None
        
        # Switch to the next thread
        sim.switchThread()


#########################################################################
# Calculates the ranges of the x and y axis of the world
#
# Parameters:
#   width (float): the width of the world
#   length (float): the length of the world
#
# Returns:
#   tuple: the ranges of the x and y axis of the world
def get_axis_ranges(width, length):
    # Midpoint of world x-axis is 0, xmax is half the width
    xmax = width/2.0   
    # xmin is negative half the width
    xmin = -width/2.0
    # Midpoint of world y-axis is 0, ymax is half the length
    ymax = length/2.0
    # ymin is negative half the length
    ymin = -length/2.0
    return (xmin,xmax),(ymin,ymax)


#########################################################################
# Adds a masking cell to the simulation at the given indices
#
# Parameters:
#   i (int): The row index of the cell to be added
#   j (int): The column index of the cell to be added
#
# Returns:
#   cell_handle (int): The handle of the created cell

def add_masking_cell(i,j):
    # Get the cell size from the "cell_size" property
    cell_size = get("cell_size")
    # Get the Floor object from the simulation
    floor = sim.getObject("/Floor")
    # Calculate the world coordinates for the given cell index
    world_pos = get_world_coordinates([i,j])         
    # Create a cuboid shape with the cell size
    cell_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    # Set the position of the mask cell to be on the floor at the calculated world position
    sim.setObjectPosition(cell_handle, floor, [world_pos[0], world_pos[1], 0.15])
    # Set the color of the mask cell to a dark gray
    sim.setShapeColor(cell_handle, None, 0, [0.1, 0.1, 0.1])
    # Set the mask cell to be collidable and measurable
    sim.setObjectSpecialProperty(cell_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    # Return the handle of the mask cell 
    return cell_handle


#########################################################################
# Initialize the grid of cells in the simulation environment.
#
# Returns:
#   grid (np.array): 2D array of ints representing the handles of the mask cells in the grid.

def init_grid():
    # Get the cell size from the global datastore
    cell_size = get('cell_size')
    # Get the world size in terms of x-axis and y-axis ranges
    xaxis, yaxis = get_world_size()
    # Calculate the number of rows and columns in the grid based on the world size and cell size
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    # Initialize a 2D grid using the number of rows and columns as its dims
    grid = np.empty((nrows, ncols), dtype=np.int32)
    # Iterate over each row and column in the grid
    for i in range(nrows):
        for j in range(ncols):
            # Add a masking cell at each position in the grid
            cell_handle = add_masking_cell(i,j)
            # Store the handle of the cell object in the 2d grid
            grid[i][j] = int(cell_handle)
    # Return the grid
    return grid


#########################################################################
#Convert grid position to world coordinates.
#Note: The grid position is left aligned at 0, while the world coordinates are center aligned on objects.
#    
# Parameters:
# grid_pos ([int, int]): Position in the grid represented as (row, column) indices.
#
# Returns:
# [float, float]: The world coordinates corresponding to the grid position.

def get_world_coordinates(grid_pos):
    cell_size = get('cell_size')
    (xmin, xmax),(ymin, ymax) = get_world_size()
    # Shift axis from leftmost point (grid) to center point (world) and left-shift range into negatives
    x = cell_size * grid_pos[0] + xmin + cell_size / 2.0
    y = cell_size * grid_pos[1] + ymin + cell_size / 2.0
    return x, y


#########################################################################
# Convert world coordinates to grid position.
# Note: The grid position is left aligned at 0, while the world coordinates are center aligned on objects.
# 
# Parameters:
# world_pos ([float, float]): Position in the world represented as (x, y) coordinates.
#
# Returns:
# [int, int]: The grid position corresponding to the world coordinates.

def get_grid_coordinates(world_pos):
    cell_size = get('cell_size')
    (xmin, xmax),(ymin, ymax) = get_world_size()
    # Shift axis from center point (world) to leftmost point (grid) and rightshift range to 0
    i = int((world_pos[0] - xmin) / cell_size)
    j = int((world_pos[1] - ymin) / cell_size)
    return i, j


#########################################################################
# Remove Object from Grid Cell
# 
# Parameters:
# i (int): Row index of the cell in the grid.
# j (int): Column index of the cell in the grid.
# grid (np.array): 2D grid containing the handles of objects in each cell.

def remove_from_grid(i, j, grid):
    # Check if there is an object in the grid cell
    if grid[i][j] != -1:
        # Get the handle of the object in the cell
        object_handle = int(grid[i][j])
        # Mark the cell as empty in the grid
        grid[i][j] = -1
        # Check if the handle is valid
        if sim.isHandle(object_handle):
            # Remove the object from the simulation
            sim.removeObject(object_handle)


#########################################################################
# Select a random unvisited cell from the given masked grid as the next waypoint.
#
# Parameters:
# mask2d (np.ndarray): 2D array of cell handles representing the masked grid.
#
# Returns:
# [np.ndarray, np.ndarray]: The world position of the selected waypoint and the updated masked grid.

def select_random_waypoint(mask2d):
    # Flatten the 2D array of cell handles into a 1D list
    flat_mask = mask2d[mask2d != -1].tolist()
    # If there are no cells remaining, initialize a new masked grid and choose a waypoint from there
    if len(flat_mask) == 0:
        mask2d = init_grid()
        return select_random_waypoint(mask2d)
    # Select a random unvisited cell as the next waypoint
    waypoint_handle = random.choice(flat_mask)
    # Get the position of the waypoint in the world
    next_waypoint = sim.getObjectPosition(waypoint_handle, -1)
    # Set the color of the waypoint to blue
    sim.setShapeColor(waypoint_handle, None, 0, [0, 0, 1])
    # Return the world position of the waypoint and the updated mask grid
    return next_waypoint, mask2d


#########################################################################
# Calculates the weight (importance) of an unvisited cell
#
# Parameters:
#   unvisited (np.array): the unvisited cell to calculate the weight for
#   src (np.array): the source position
#   visited_mask2d (np.array): the 2D mask indicating which cells have been visited
#   visited_len (int): the length of the visited mask
#   unvisited_len (int): the length of the unvisited mask
#
# Returns:
#   float: the weight of the unvisited cell

def calculate_weight(unvisited, src, visited_mask2d, visited_len, unvisited_len):
    # Set the initial penalty value to 1
    penalty = 1
    # If the unvisited cell has already been visited, increase the penalty
    if np.any(np.all(visited_mask2d == unvisited, axis=1)):
        penalty = 2 + visited_len
    # Calculate the unvisited factor based on the proportion of unvisited cells
    unvisited_factor = unvisited_len / (visited_len + unvisited_len)
    # Calculate the distance between the unvisited cell and source
    distance = np.sqrt(np.sum((unvisited - src)**2))
    # Return the sum of the penalty, unvisited factor, and distance
    return penalty + unvisited_factor * -1 + distance


#########################################################################
# Calculates the weight of all the unvisited cells.
#
# Parameters:
#   unvisited_mask2d (np.array): the 2D mask indicating the positions of unvisited cells
#   current_pos (np.array): the current position of the agent
#   visited_mask2d (np.array): the 2D mask indicating which cells have been visited
#   visited_len (int): the length of the visited mask
#   unvisited_len (int): the length of the unvisited mask
#
# Returns:
#   list of np.array: the unvisited cells sorted by their weight, consisting of the x, y, and weight

def get_weights(unvisited_mask2d, current_pos, visited_mask2d, visited_len, unvisited_len):
    distances = []
    # loop through all unvisited cells
    for unvisited in unvisited_mask2d:
        # calculate the weight of the cell
        weight = calculate_weight(unvisited, current_pos[:2], visited_mask2d, visited_len, unvisited_len)
        # append the unvisited cell and its weight to the distances list
        distances.append(np.append(unvisited, weight))
    # sort the distances based on the weight
    return sorted(distances, key=lambda x: x[2])


#########################################################################
# Gets the adjacent cells to the current position
#
# Parameters:
#   distances (list of np.array): the distances of the unvisited cells from the current position
#   current_pos (np.array): the current position of the agent
#
# Returns:
#   list of np.array: the adjacent cells, consisting of the x, y coordinates

def get_adjacents(distances, current_pos):
    adjacents = []
    for distance in distances:
        # Check if the x-coordinate of the distance is within 1 of the current position's x-coordinate
        if np.abs(distance[0] - current_pos[0]) <= 1:
            # Check if the y-coordinate of the distance is equal to the current position's y-coordinate
            if distance[1] == current_pos[1]:
                # If both conditions are met, the distance is adjacent
                adjacents.append(distance[:2])
        # Check if the y-coordinate of the distance is within 1 of the current position's y-coordinate
        elif np.abs(distance[1] - current_pos[1]) <= 1:
            # Check if the x-coordinate of the distance is equal to the current position's x-coordinate
            if distance[0] == current_pos[0]:
                # If both conditions are met, the distance is adjacent
                adjacents.append(distance[:2])
    return adjacents


#########################################################################
# Determines the destination for the agent to move to
#
# Parameters:
#   adjacents (list of np.array): the adjacent cells to the current position
#   next_waypoint (np.array): the next waypoint for the agent to reach
#   unvisited_len (int): the length of the unvisited mask
#   visited_len (int): the length of the visited mask
#
# Returns:
#   np.array or list of np.array: the destination for the agent to move to, either a single np.array representing the next waypoint or a list of np.array representing the adjacent cells

def get_destination(adjacents, next_waypoint, unvisited_len, visited_len):
    # Check if the ratio of unvisited to visited is less than or equal to 0.05
    if unvisited_len / (visited_len + unvisited_len) <= 0.05:
        # If there are adjacents and the ratio meets the requirement, return adjacents
        if adjacents:
            return adjacents
    # If the ratio does not meet the requirement or there are no adjacents, return next_waypoint
    return next_waypoint


#########################################################################
# Calculates the angle between the current position and the destination
#
# Parameters:
#   destination (np.array): the position of the destination
#   current_pos (np.array): the position of the current position
#
# Returns:
#   float: the angle between the current position and destination, with a small random offset added

def get_angle(destination, current_pos):
    # Calculate the difference in x and y positions
    x_diff = destination[1] - current_pos[1]
    y_diff = destination[0] - current_pos[0]
    # Calculate the angle between the destination and current position
    angle = np.arctan2(x_diff, y_diff)
    # Add a small random offset to the angle
    random_offset = (random.random() - 0.5) * 0.1
    # Return the final angle
    return angle + random_offset


#########################################################################
# Calculates the new position of the agent based on the angle and step size
#
# Parameters:
#   angle (float): the angle between the current position and the destination
#   step_size (float): the size of the step taken in each time step
#   current_pos (list): the current position of the agent
#
# Returns:
#   list: the new position of the agent as a list of x, y, and z coordinates

def get_new_pos(angle, step_size, current_pos):
    # Calculate new x position using cosine of angle and step size
    new_x = current_pos[0] + np.cos(angle) * step_size
    # Calculate new y position using sine of angle and step size
    new_y = current_pos[1] + np.sin(angle) * step_size
    # Keep the current z position
    new_z = current_pos[2]
    # Return the new position as a list
    return [new_x, new_y, new_z]


#########################################################################
# Move the quadcopter towards the next waypoint
#
# Parameters:
#   src_handle (int): the handle of the source object to be moved
#   next_waypoint (np.array): the next target position for the source object
#   mask2d (np.array): the 2D mask indicating the visited and unvisited cells

def move_to_waypoint(src_handle, next_waypoint, mask2d):
    # Get the positions of unvisited and visited points in the mask2d
    unvisited_mask2d = np.array(np.where(mask2d != -1)).T
    visited_mask2d = np.array(np.where(mask2d == -1)).T
    # Get the current position of the source handle
    current_pos = sim.getObjectPosition(src_handle, -1)
    # Get the speed of the quadcopter
    speed = get('quadcopter_speed')
    # Calculate the step size based on the speed and simulation time step
    step_size = speed * sim.getSimulationTimeStep()
    # Get the number of visited and unvisited points
    visited_len = len(visited_mask2d)
    unvisited_len = len(unvisited_mask2d)
    # Get the distances of unvisited points from the current position
    distances = get_weights(unvisited_mask2d, current_pos, visited_mask2d, visited_len, unvisited_len)
    # Get the adjacent points to the current position
    adjacents = get_adjacents(distances, current_pos)
    # Get the final destination based on the adjacents and next waypoint
    destination = get_destination(adjacents, next_waypoint, unvisited_len, visited_len)
    # Calculate the angle to move towards the destination
    angle = get_angle(destination, current_pos)
    # Calculate the new position based on the angle and step size
    new_pos = get_new_pos(angle, step_size, current_pos)
    # Set the position of the source handle to the new position
    sim.setObjectPosition(src_handle, -1, new_pos)



#########################################################################
# Determines if the source has reached the next waypoint (or enemy)
#
# Parameters:
#   next_waypoint (np.array): the next waypoint position
#
# Returns:
#   bool: True if the source has reached the next waypoint, False otherwise

def check_waypoint_reached(next_waypoint):
    # Get the source's row and column position in the grid
    src_i,src_j = get_observer_coordinates()
    # Get the next waypoint's row and column position in the grid
    dst_i, dst_j = get_grid_coordinates(next_waypoint)
    # Check if the source and next waypoint have the same row and column position in the grid
    return src_i == dst_i and src_j == dst_j


#########################################################################
# Unmasks the cell at the observer's position in the grid
#
# Parameters:
#   mask2d (list of list): the 2D mask indicating which cells in the grid have been visited

def unmask_cell_at_observer(mask2d):
    # Get the observer's row and column position in the grid
    i,j = get_observer_coordinates()
    # Check if the observer's position is within the bounds of the grid
    if i >= 0 and i < len(mask2d) and j >= 0 and j < len(mask2d[0]):
        # Remove the cell at the observer's position from the grid
        remove_from_grid(i,j, mask2d)


#########################################################################
# Queries the row and column position of the observer (quadcopter) in the grid
#
# Returns:
#   tuple (int, int): the row and column position of the observer in the grid

def get_observer_coordinates():
    # Get the handle of the quadcopter
    handle = sim.getObject("/Quadcopter")
    # Get the world position of the quadcopter
    world_pos = sim.getObjectPosition( handle, -1)
    # Get the grid coordinates of the quadcopter's position
    i,j = get_grid_coordinates(world_pos)
    # Return the row and column position of the observer in the grid
    return i,j


#########################################################################
# Adds an enemy to the simulation at the given grid cell
#
# Parameters:
#   i (int): the row position of the cell
#   j (int): the column position of the cell
#
# Returns:
#   int: the handle of the created enemy object

def add_enemy(i,j):
    # Get the size of a cell in the grid
    cell_size = get('cell_size')
    # Get the handle of the floor
    floor = sim.getObject("/Floor")
    # Get the world coordinates of the cell at the given row and column position
    world = get_world_coordinates([i,j])         
    # Create a red cube with the same size as a cell in the grid
    enemy_handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [cell_size, cell_size, 0], 1)
    # Set the position of the enemy to the world coordinates of the given cell
    sim.setObjectPosition(enemy_handle, floor, [world[0], world[1], 0.155])
    # Set the color of the enemy to red
    sim.setShapeColor(enemy_handle, None, 0, [1, 0, 0])
    # Make the enemy collidable and measurable
    sim.setObjectSpecialProperty(enemy_handle, sim.objectspecialproperty_collidable + sim.objectspecialproperty_measurable)
    # Return the handle of the enemy
    return enemy_handle


#########################################################################
# Initialize the enemies in the simulation
#
# Returns:
#   List of int: the handles of the created enemy objects

def init_enemies():
    # Get the size of a cell in the grid
    cell_size = get('cell_size')
    # Get the size of the world in the x and y directions
    xaxis,yaxis = get_world_size()
    # Calculate the number of rows and columns in the grid
    nrows = int((xaxis[1] - xaxis[0]) / cell_size)
    ncols = int((yaxis[1] - yaxis[0]) / cell_size)
    # Get the number of enemies to generate
    enemy_count = get('enemy_count')
    # Initialize a list to store the handles of the enemies
    enemy_handles = []
    # Generate the specified number of enemies
    for i in range(enemy_count):
        # Randomly select a row and column position for the enemy
        enemy_i = random.randint(0, nrows-1)
        enemy_j = random.randint(0, ncols-1)
        # Add an enemy at the selected position
        enemy_handle = add_enemy(enemy_i, enemy_j)
        # Append the handle of the enemy to the list of enemy handles
        enemy_handles.append(int(enemy_handle))
    # Return the list of enemy handles
    return enemy_handles


#########################################################################
# Move an enemy to a random adjacent cell
#
# Parameters:
#   src_handle (int): the handle of the enemy object to move
#   mask2d (2D list, optional): tracks masked cells, and hides enemy 

def move_to_random_adjacent_cell(src_handle, mask2d=None):
    # Get the cell size and world size
    cell_size = get('cell_size')
    xaxis, yaxis = get_world_size()
    # Get the speed of the enemy
    speed = get('enemy_speed')
    # Get the current simulation time
    current_time = sim.getSimulationTime()
    # Check if it's time to move the enemy
    if current_time % 1.0 < sim.getSimulationTimeStep():
        # Get the current position of the enemy
        current_pos = sim.getObjectPosition(src_handle, -1)
        # Calculate the step size for this time step
        step_size = speed * sim.getSimulationTimeStep()

        # Get the grid coordinates of the enemy's current position
        i, j = get_grid_coordinates(current_pos)
        # Calculate the number of rows and columns in the grid
        nrows = int((xaxis[1] - xaxis[0]) / cell_size)
        ncols = int((yaxis[1] - yaxis[0]) / cell_size)

        # Get the observer's grid coordinates
        copter_i, copter_j = get_observer_coordinates()

        # Choose a random adjacent cell for the enemy to move to
        while True:
            next_i, next_j = i + random.choice([-1, 0, 1]), j + random.choice([-1, 0, 1])
            # Check if the cell is within the bounds of the grid and not the observer's current cell
            if next_i >= 0 and next_i < nrows and next_j >= 0 and next_j < ncols and (next_i != copter_i or next_j != copter_j):
                break

        # Get the world coordinates of the chosen cell
        world = get_world_coordinates([next_i, next_j])
        # Calculate the new position of the enemy
        new_pos = [world[0], world[1], current_pos[2]]
        # Move the enemy to the new position
        sim.setObjectPosition(src_handle, -1, new_pos)

        # Change the color of the enemy based on whether the next cell is unmasked or not
        if mask2d is not None and mask2d[next_i][next_j] != -1:
            sim.setShapeColor(src_handle, None, 0, [0, 0, 0])
        else:
            sim.setShapeColor(src_handle, None, 0, [1, 0, 0])



#########################################################################
# Hides the default floor in the simulation

def hide_default_floor():
    # Get the handle of the default floor
    handle = sim.getObjectHandle("/Floor/box")
    # Set the floor to be invisible
    sim.setModelProperty(handle,sim.modelproperty_not_visible)


#########################################################################
# Initialize the 2D floor in the simulation
#
# The approach used is to texture a single cube face into a grid to be more performant by reducing geometry

def init_floor2d():
    # Get the cell size and world size
    cell_size = get('cell_size')
    width = get('world_width')
    length = get('world_length')
    xaxis, yaxis = get_axis_ranges(width, length)
    # Set the elevation of the floor
    zlevel = 0
    # Hide the default floor
    hide_default_floor()
    # Calculate the size of the floor in the x and y directions
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
    # Generate the texture
    for i in range(ncols):
        for j in range(nrows):
            color = [255, 255, 255] if (i + j) % 2 == 0 else [192, 192, 192]
            block = np.full((texture_size, texture_size, 3), color, dtype=np.uint8)
            texture[i*texture_size:(i+1)*texture_size, j*texture_size:(j+1)*texture_size, :] = block
    # Create a temporary file to store the texture
    with tempfile.NamedTemporaryFile(mode='wb', suffix='.png', delete=False) as f:
        image_filename = f.name
        img = Image.fromarray(texture, 'RGB')
        img.save(f, 'PNG')
    # Create the texture and apply it to the floor surface
    _, texture_id, _ = sim.createTexture(image_filename, 0)
    sim.addLog(0,"Texture: {}".format( texture_id))
    uv = max(x_size, y_size)
    sim.setShapeTexture(floor_handle, texture_id, sim.texturemap_cube, 1, [uv,uv])
    # Get the handle of the world object
    world_handle = sim.getObjectHandle("/Cuboid")
    # Rename the world object to "Worldspace"
    sim.setObjectName(world_handle, "Worldspace")


#########################################################################    
# Queries the ranges of the x and y values of the world model box
#
# Returns:
#   tuple: the ranges of the x and y values of the world model box

def get_world_size():
    # Get the handle of the world object
    world_handle = sim.getObjectHandle("Worldspace")
    # Get the minimum x value of the world model
    xmin = sim.getObjectFloatParam(world_handle, sim.objfloatparam_modelbbox_min_x)
    # Get the maximum x value of the world model
    xmax = sim.getObjectFloatParam(world_handle, sim.objfloatparam_modelbbox_max_x)
    # Get the minimum y value of the world model
    ymin = sim.getObjectFloatParam(world_handle, sim.objfloatparam_modelbbox_min_y)
    # Get the maximum y value of the world model
    ymax = sim.getObjectFloatParam(world_handle, sim.objfloatparam_modelbbox_max_y)
    # Return the ranges of the x and y values of the world 
    return (xmin,xmax),(ymin,ymax)