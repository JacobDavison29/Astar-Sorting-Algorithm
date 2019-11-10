import numpy as np
import time
import pygame as pg


"""
Define constants related to Grid size
"""
X_Length = 10
Y_Length = 10

"""
Used to calculate how long the program takes to finish
"""
Start_time = time.time()
pg.init()

"""
Used for obstacles that will be implemented 
"""
Number_of_Walls = 10
Wall_Length = 6

"""
Create a node class which stores atributes that are required to perform the Astar sorting algorithm.

TODO:
- Change the parent attribute tobe a node type instead of a list
"""
class Node:

    def __init__(self):

        Location = []
        g = 0
        f = 0
        h = 0
        Parent = []


"""
Creates a nxn grid and defines the algorithms start and end location
"""
def Grid():

    grid = np.zeros((X_Length,Y_Length))
    Start_node = Node()
    End_node = Node()


    #Starting Location
    Start_X = 0
    Start_Y = 0

    #Pick Random End Location
    End_X = np.random.randint(0,X_Length)
    End_Y = np.random.randint(0,Y_Length)


    for i in range(0, X_Length):
        for j in range(0, Y_Length):
            if(Start_X == i and Start_Y == j):
                grid[i][j] = -1
                Start_node.Location = [i,j]

            if(End_X == i and End_Y == j):
                grid[i][j] = -2
                End_node.Location = [i,j]

            if(Start_X == End_X and Start_Y == End_Y):
                End_X = np.random.randint(0, X_Length)
                End_Y = np.random.randint(0, Y_Length)


    '''
    TODO:
    - creating obstacles for the grid
    
    for m in range(0, Number_of_Walls):
        for n in range(0, Wall_Length):
            random_x = np.random.randint(0, X_Length)
            random_y = np.random.randint(0, Y_Length)
            if (random_x != Start_X and random_x != End_X):
                grid[random_x][random_y] = 1
    '''

    PathBoi = A_Star(Start_node, End_node)

    for i in PathBoi:
        if(i != [0,0]):
            grid[i[0]][i[1]] = 500

    print("Run Time ", time.time() - Start_time)
    print(grid)

"""
The Astar function keeps track of which nodes have and have not been visited by the algorithm and chooses which node to visit next based on the nodes f cost.

Parameter:
- Start_node: List representing the coordinates of the starting node.
- End_node: List representing the coordinates of the ending node 

"""
def A_Star(Start_node, End_node):

    Open_Nodes = []
    Open_Nodes.append(Start_node)
    Closed_Nodes = []
    Start_node.g = 0
    Start_node.f = Start_node.g + Calculate_H_Values(Start_node,End_node)
    while (len(Open_Nodes) != 0):
        Open_Nodes.sort(key=lambda x: x.f)
        Current_node = Open_Nodes[0]
        if(Current_node.Location == End_node.Location):
            return Create_Path(Current_node,Closed_Nodes)

        Closed_Nodes.append(Current_node)
        Open_Nodes.pop(0)


        for neighbor in Neighbor_Nodes(Current_node):
            if(neighbor not in Closed_Nodes):
                neighbor.f = neighbor.g + Calculate_H_Values(neighbor,End_node)
                if(neighbor not in Open_Nodes):
                    Open_Nodes.append(neighbor)
                else:
                    Open_neighbor = Open_Nodes[Open_Nodes.index(neighbor)]
                    if(neighbor.g < Open_neighbor.g):
                        Open_neighbor.g = neighbor.g
                        Open_neighbor.Parent = neighbor.Parent

    return False

"""
Calculate_H_Values calculates and returns the h value for the Current_node.

Parameter:
- Current_node: List representing the coordinates of the current node.
- End_node: List representing the coordinates of the ending node 

"""
def Calculate_H_Values(Current_node, End_node):


    Distance_X = np.absolute(Current_node.Location[0] - End_node.Location[0])
    Distance_Y = np.absolute(Current_node.Location[1]-End_node.Location[1])
    Smallest_Distance = min(Distance_X,Distance_Y)
    Current_node.h = (10 * (Distance_X + Distance_Y) + (14 - 2 * 10) * Smallest_Distance)

    return Current_node.h

"""
Neighbor_Nodes calculates the g value for each of the nodes surrounding the Current node, this includes diagonals but excludes those that are out of bounds. 
Returns a list of nodes surrounding the current node

Parameter:
- Current_node: List representing the coordinates of the current node.
"""
def Neighbor_Nodes(Current_node):

    g=[]
    G_Cost_Index_X = []
    G_Cost_Index_Y = []
    neighbors = []

    for i in range(0, X_Length):
        for j in range(0, Y_Length):
            G_Cost_Index_X.append(i)
            G_Cost_Index_Y.append(j)
            t = np.absolute(Current_node.Location[0] - i)
            r = np.absolute(Current_node.Location[1] - j)
            m = t + r
            g.append(m)

    for i in range(len(g)):
        if (g[i] == 1):
            neighbor_node = Node()
            neighbor_node.g = 10
            neighbor_node.Parent = Current_node.Location
            neighbor_node.Location = [G_Cost_Index_X[i],G_Cost_Index_Y[i]]
            neighbors.append(neighbor_node)

            # middle of board
            try:
                # middle of board
                if (g[i + 1] == 2 and g[i - 1] == 2):
                    neighbor_node = Node()
                    neighbor_node.g = 14
                    neighbor_node.Parent = Current_node.Location
                    neighbor_node.Location = [G_Cost_Index_X[i-1], G_Cost_Index_Y[i-1]]
                    neighbors.append(neighbor_node)

                    neighbor_node = Node()
                    neighbor_node.g = 14
                    neighbor_node.Parent = Current_node.Location
                    neighbor_node.Location = [G_Cost_Index_X[i + 1], G_Cost_Index_Y[i + 1]]
                    neighbors.append(neighbor_node)

                # Top left of board also sides
                elif (g[i + 1] == 2 and g[i - 1] != 0):
                    neighbor_node = Node()
                    neighbor_node.g = 14
                    neighbor_node.Parent = Current_node.Location
                    neighbor_node.Location = [G_Cost_Index_X[i + 1], G_Cost_Index_Y[i + 1]]
                    neighbors.append(neighbor_node)

                # top right of board also sides
                elif (g[i - 1] == 2 and g[i + 1] != 0):
                    neighbor_node = Node()
                    neighbor_node.g = 14
                    neighbor_node.Parent = Current_node.Location
                    neighbor_node.Location = [G_Cost_Index_X[i - 1], G_Cost_Index_Y[i - 1]]
                    neighbors.append(neighbor_node)

            except IndexError:
                pass



    return neighbors

"""
Create_Path takes the current node and traces back to the start based off of the current nodes parent.
Returns a path from the start to end node 

Parameters:
- Current_node: List representing the coordinates of the current node.
- Closed_Nodes: List containing nodes that makeup the shortest path to the end node from the start 
"""
def Create_Path(Current_node,Closed_Nodes):
    path = []

    while len(Closed_Nodes) != 0:
        path.append(Current_node.Parent)
        Current_node = Closed_Nodes[-1]
        Closed_Nodes.pop()

    return path



Grid()