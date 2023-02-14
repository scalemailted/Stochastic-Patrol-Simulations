
# API Documentation

---

## `add_masking_cell`

Adds a masking cell to the simulation at the given indices

### Parameters

- `i` (int): The row index of the cell to be added
- `j` (int): The column index of the cell to be added

### Returns

- `cell_handle` (int): The handle of the created cell

---

## `calculate_weight`

Calculates the weight (importance) of an unvisited cell

### Parameters

- `unvisited` (np.array): the unvisited cell to calculate the weight for
- `src` (np.array): the source position
- `visited_mask2d` (np.array): the 2D mask indicating which cells have been visited
- `visited_len` (int): the length of the visited mask
- `unvisited_len` (int): the length of the unvisited mask

### Returns

- `float`: the weight of the unvisited cell

---

## `check_waypoint_reached`
Determines if the source has reached the next waypoint (or enemy)

#### Parameters:
- `next_waypoint` (np.array): the next waypoint position

#### Returns:
- bool: True

---

## `get`

Global datastore for simulation features

### Parameters

- `prop` (str): the property to retrieve

### Returns

- `float` or `int`: the value of the requested property

---

## `get_adjacents`
Gets the adjacent cells to the current position

#### Parameters:
- `distances` (list of np.array): the distances of the unvisited cells from the current position
- `current_pos` (np.array): the current position of the agent

#### Returns:
- list of np.array: the adjacent cells, consisting of the x, y coordinates

---

## `get_angle`
Calculates the angle between the current position and the destination

#### Parameters:
- `destination` (np.array): the position of the destination
- `current_pos` (np.array): the position of the current position

#### Returns:
- float: the angle between the current position and destination, with a small random offset added

---

## `get_axis_ranges`

Calculates the ranges of the x and y axis of the world

### Parameters

- `width` (float): the width of the world
- `length` (float): the length of the world

### Returns

- `tuple`: the ranges of the x and y axis of the world

---

## `get_destination`
Determines the destination for the agent to move to

#### Parameters:
- `adjacents` (list of np.array): the adjacent cells to the current position
- `next_waypoint` (np.array): the next waypoint for the agent to reach
- `unvisited_len` (int): the length of the unvisited mask
- `visited_len` (int): the length of the visited mask

#### Returns:
- np.array or list of np.array: the destination for the agent to move to, either a single np.array representing the next waypoint or a list of np.array representing the adjacent cells

--- 

## `get_grid_coordinates`

Convert world coordinates to grid position. Note: The grid position is left aligned at 0, while the world coordinates are center aligned on objects.

### Parameters

- `world_pos` ([float, float]): Position in the world represented as (x, y) coordinates.

### Returns

- `[int, int]`: The grid position corresponding to the world coordinates.

---

## `get_new_pos`
Calculates the new position of the agent based on the angle and step size

#### Parameters:
- `angle` (float): the angle between the current position and the destination
- `step_size` (float): the size of the step taken in each time step
- `current_pos` (list): the current position of the agent

#### Returns:
- list: the new position of the agent as a list of x, y, and z coordinates

---

## `get_weights`
Calculates the weight of all the unvisited cells.

#### Parameters:
- `unvisited_mask2d` (np.array): the 2D mask indicating the positions of unvisited cells
- `current_pos` (np.array): the current position of the agent
- `visited_mask2d` (np.array): the 2D mask indicating which cells have been visited
- `visited_len` (int): the length of the visited mask
- `unvisited_len` (int): the length of the unvisited mask

#### Returns:
- list of np.array: the unvisited cells sorted by their weight, consisting of the x, y, and weight

---

## `get_world_coordinates`

Convert grid position to world coordinates. Note: The grid position is left aligned at 0, while the world coordinates are center aligned on objects.

### Parameters

- `grid_pos` ([int, int]): Position in the grid represented as (row, column) indices.

### Returns

- `[float, float]`: The world coordinates corresponding to the grid position.

---

## `init_grid`

Initialize the grid of cells in the simulation environment.

### Returns

- `grid` (np.array): 2D array of ints representing the handles of the mask cells in the grid.

---

## `move_to_waypoint`
Move the quadcopter towards the next waypoint

#### Parameters:
- `src_handle` (int): the handle of the source object to be moved
- `next_waypoint` (np.array): the next target position for the source object
- `mask2d` (np.array): the 2D mask indicating the visited and unvisited cells

---

## `remove_from_grid`

Remove Object from Grid Cell

### Parameters

- `i` (int): Row index of the cell in the grid.
- `j` (int): Column index of the cell in the grid.
- `grid` (np.array): 2D grid containing the handles of objects in each cell.

---

## `select_random_waypoint`

Select a random unvisited cell from the given masked grid as the next waypoint.

### Parameters

- `mask2d` (np.ndarray): 2D array of cell handles representing the masked grid.

### Returns

- `[np.ndarray, np.ndarray]`: The world position of the selected waypoint and the updated masked grid.

---

## `sysCall_thread`

Main Thread for Simulation. Initializes the grid, generates the enemies, moves the quadcopter and enemies, and checks if the quadcopter touched either the enemies or the waypoint.

---

