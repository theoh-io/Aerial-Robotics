import numpy as np
import math

# Global variables
START_POS_X = 0
START_POS_Y = 0

## A* star or global nav variables
start = (START_POS_X, START_POS_Y) # to get before the start of the drone
goal = (9,4) # to get from the zranger detection, to get when landing on base done
len_x, len_y = (10, 5)

###################################################################################################################
# Define all types of movements between two points:
def type_deplacement(ptA,ptB):
    x = ptA[0]-ptB[0]
    y = ptA[1]-ptB[1]
    deplacement =''
    if(x == 1 & y == 1):
        deplacement = 'NE'
    elif (x == 1 & y == 0):
        deplacement = 'E'
    elif (x == 1 & y == -1):
        deplacement = 'SE'
    elif (x == 0 & y == -1):
        deplacement = 'S'
    elif (x == -1 & y == -1):
        deplacement = 'SO'
    elif (x == -1 & y == 0):
        deplacement = 'O'
    elif (x == -1 & y == 1):
        deplacement = 'NO'
    elif (x == 0 & y == 1):
        deplacement = 'N'
    return deplacement

# # Transform list of points in segments and checkpoints
# # This helps to get a smoother path, removing "zigzag"
# def define_checkpoints(path):
#     ### First transform the path to get checkpoints, for each segment
#     checkpoints = []
#     deplacement1 = ' ' # first point defined as a checkpoint
#     for i in range (1,len(path[0])):
#         pos1 = path[:,i-1]
#         pos2 = path[:,i]
#         deplacement2 = type_deplacement(pos2,pos1)
#         if (deplacement1 != deplacement2):
#             checkpoints.append(pos1)
#         deplacement1 = deplacement2
        
#     checkpoints.append(pos2) # last point defined as a checkpoint
#     smooth_cp = []
    
#     ### Then remove the very small changes ("zigzags")
#     smooth_cp.append(checkpoints[0]) # last point defined as a checkpoint

#     for i in range (0,len(checkpoints)-1):
#         pos1 = checkpoints[i]
#         pos2 = checkpoints[i+1]
#         if (abs(checkpoints[i][0]-checkpoints[i+1][0])>1)or(abs(checkpoints[i][1]-checkpoints[i+1][1])>1):
#             smooth_cp.append(checkpoints[i+1])
#     smooth_cp.append(checkpoints[len(checkpoints)-1])
#     return smooth_cp

###################################################################################################################

#possible movements in an 8 connected grid:  (up, down, left, right and the 4 diagonals)
def _get_movements_8n():
    """
    Equivalent to get_movements_in_radius(1)
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]

###################################################################################################################
# A* :

#Recurrently reconstructs the path from start node to the current node 
def reconstruct_path(cameFrom, current):
    """
    :param cameFrom: map (dictionary) containing for each node n the node immediately 
                     preceding it on the cheapest path from start to n currently known.
    :param current: current node (x, y)
    :return: list of nodes from start to current node
    """
    total_path = [current]
    while current in cameFrom.keys():
        # Add where the current node came from to the start of the list
        total_path.insert(0, cameFrom[current]) 
        current=cameFrom[current]
    return total_path

###################################################################################################################
# A* for 2D rectangle occupancy grid. Finds a path from start to goal:
def A_Star(start, goal, h, coords, occupancy_grid, len_x, len_y, movement_type="8N"):
    """
    :param start: start node, tuple (x, y)
    :param goal: goal node, tuple (x, y)
    :param h:the heuristic function. h(n) estimates the cost to reach goal from node n.
    :param coords:list of all coordinates in the grid
    :param occupancy_grid: numpy array containing the map with the obstacles. At each position, 
                           you either have the number 1 (occupied) or 0 (free)
    :param movement: string, 8-connectivity ('8N')
    :return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
    """
    # Check if the start and goal are within the boundaries of the map
    for point in [start, goal]:
        #the start and goal position are already transposed in the vision module
        assert point[0]>=0 and point[1]>=0 and point[0]<len_x and point[1]< len_y, "start|end goal not contained in the map"
    
    # check if start and goal nodes correspond to free spaces
    if occupancy_grid[start[0], start[1]]:
        raise Exception('Start node is not traversable')

    if occupancy_grid[goal[0], goal[1]]:
        raise Exception('Goal node is not traversable')
    
    # get the possible movements corresponding to 8N connectivity
    if movement_type == '8N':
        movements = _get_movements_8n()
    else:
        raise ValueError('Unknown movement')

    # The set of visited nodes that need to be (re-)expanded, i.e. for which the neighbors need to be explored
    # Initially, only the start node is known.
    openSet = [start]
    
    # The set of visited nodes that no longer need to be expanded.
    closedSet = []

    # For node n, cameFrom[n] is the node immediately preceding it on the cheapest path from start to n currently known.
    cameFrom = dict()

    # For node n, gScore[n] is the cost of the cheapest path from start to n currently known.
    gScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    gScore[start] = 0

    # For node n, fScore[n] := gScore[n] + h(n). map with default value of Infinity
    fScore = dict(zip(coords, [np.inf for x in range(len(coords))]))
    fScore[start] = h[start]

    # while there are still elements to investigate
    while openSet != []:
        
        #the node in openSet having the lowest fScore[] value
        fScore_openSet = {key:val for (key,val) in fScore.items() if key in openSet}
        current = min(fScore_openSet, key=fScore_openSet.get)
        del fScore_openSet
        
        #If the goal is reached, reconstruct and return the obtained path
        if current == goal:
            return reconstruct_path(cameFrom, current), closedSet

        openSet.remove(current)
        closedSet.append(current)
        
        #for each neighbor of current:
        for dx, dy, deltacost in movements:
            
            neighbor = (current[0]+dx, current[1]+dy)
            
            # if the node is not in the map, skip
            if (neighbor[0] >= occupancy_grid.shape[0]) or (neighbor[1] >= occupancy_grid.shape[1]) or (neighbor[0] < 0) or (neighbor[1] < 0):
                continue
            
            # if the node is occupied or has already been visited, skip
            if (occupancy_grid[neighbor[0], neighbor[1]]) or (neighbor in closedSet): 
                continue
                
            # d(current,neighbor) is the weight of the edge from current to neighbor
            # tentative_gScore is the distance from start to the neighbor through current
            tentative_gScore = gScore[current] + deltacost
            
            if neighbor not in openSet:
                openSet.append(neighbor)
                
            if tentative_gScore < gScore[neighbor]:
                # This path to neighbor is better than any previous one. Record it!
                cameFrom[neighbor] = current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + h[neighbor]

    # Open set is empty but goal was never reached
    print("No path found to goal")
    return [], closedSet

###################################################################################################################
# Combine several fonctions of the library to ease the external use of the module
# Plot the result of the A_star

def find_path(start, goal, occupancy_grid, len_x, len_y, explored_list):
    start[0] 
    # List of all coordinates in the grid
    x,y = np.mgrid[0:len_y:1, 0:len_x:1]
    pos = np.empty(x.shape + (2,))
    pos[:, :, 0] = x; pos[:, :, 1] = y
    pos = np.reshape(pos, (x.shape[0]*x.shape[1], 2))
    # unzip dans une liste les valeurs de pos, = toutes les coordonnées de la grid
    # pos et coords sont les mêmes infos, toutes les coordonnées de la grille mais dans un autre format
    coords = list([(int(x[1]), int(x[0])) for x in pos])

    # Heuristic =  Euclidian distance between the starting point and the goal, ignoring obstacles
    h = np.linalg.norm(pos - goal, axis=-1)
    h = dict(zip(coords, h))
    for coord in explored_list:
        h[coord] = h[coord]+2

    # Run the A* algorithm
    path, visitedNodes = A_Star(start, goal, h, coords, occupancy_grid, len_x, len_y, movement_type="8N")
    path = np.array(path).reshape(-1, 2).transpose()
    visitedNodes = np.array(visitedNodes).reshape(-1, 2).transpose()
    return path

#np array
occupancy_grid = np.zeros((len_x,len_y))

#occupancy_grid[2,1] = 1
#occupancy_grid[3,1] = 1
# occupancy_grid[4,1] = 1
# occupancy_grid[5,1] = 1
# occupancy_grid[6,1] = 1
# occupancy_grid[7,1] = 1
# occupancy_grid[8,1] = 1
# occupancy_grid[6,3] = 1
# occupancy_grid[7,3] = 1
# occupancy_grid[8,3] = 1
# occupancy_grid[9,3] = 1

# dict of tuple
#explored_list = [(0,1), (0,2), (0,3), (0,4), (1,1), (1,2), (1,3), (1,4), (2,2), (2,3), (2,4),(3,2), (3,3), (3,4), (4,3), (4,4), (5,3), (5,4)]

path=find_path(start, goal, occupancy_grid, len_x+1, len_y+1, explored_list)
print(path)