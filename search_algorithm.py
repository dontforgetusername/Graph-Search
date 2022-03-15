import pygame
import graphUI
from node_color import white, yellow, black, red, blue, purple, orange, green
from math import sqrt

"""
Feel free print graph, edges to console to get more understand input.
Do not change input parameters
Create new function/file if necessary
"""


def BFS(graph, edges, edge_id, start, goal):
    """
    BFS search
    """
    # TODO: your code
    #first case
    if (goal == start):
        graph[start][3] = orange
        graphUI.updateUI()
        graph[start][3] = purple
        graphUI.updateUI()
        path = [start]
        return path

    #initialize queue, discovered, path and previous_node
    queue = []
    discovered = []
    path = []
    previous_node = []
    for i in range(len(graph)):
        discovered.append(False)
        previous_node.append(-1)
    
    #add starting vertex to the queue and label it as discovered
    queue.append(start)
    graph[start][3] = orange
    graphUI.updateUI()

    while queue:
        current_node = queue.pop(0)

        #update current vertex color
        graph[current_node][3] = yellow
        graphUI.updateUI()

        #update discovered edges and following vertices
        for adjacency_node in graph[current_node][1]:
            if (not discovered[adjacency_node]) and (adjacency_node not in queue):
                previous_node[adjacency_node] = current_node

                #update color
                edges[edge_id(current_node, adjacency_node)][1] = white
                graph[adjacency_node][3] = red
                graphUI.updateUI()

                #reach the goal
                if (adjacency_node == goal):
                    temp = goal
                    path.append(temp)
                    while (previous_node[temp] != -1):
                        temp = previous_node[temp]
                        path.append(temp)
                    
                    #update path color
                    for i in range(len(path) - 1):
                        edges[edge_id(path[i], path[i + 1])][1] = green
                    graph[goal][3] = purple
                    graph[start][3] = orange
                    graphUI.updateUI()
                    return path     
            
                #add adjacency vertex to the queue
                queue.append(adjacency_node)

        #update current vertex as discovered
        discovered[current_node] = True
        graph[current_node][3] = blue
        graphUI.updateUI()

    print("Can't find the path")
    return None



    print("Implement BFS algorithm.")
    pass


def DFS(graph, edges, edge_id, start, goal):
    """
    DFS search
    """
    # TODO: your code
    #first case
    if (goal == start):
        graph[start][3] = orange
        graphUI.updateUI()
        graph[start][3] = purple
        graphUI.updateUI()
        path = [start]
        return path

    #initialize stack, discovered, path and previous_node
    stack = []
    discovered = []
    path = []
    previous_node = []
    for i in range(len(graph)):
        discovered.append(False)
        previous_node.append(-1)
    
    #add starting vertex to the stack and label it as discovered
    stack.append(start)
    graph[start][3] = orange
    graphUI.updateUI()

    while stack:
        current_node = stack.pop()

        #update current vertex color
        graph[current_node][3] = yellow
        graphUI.updateUI()

        #update discovered edges and following vertices
        for adjacency_node in graph[current_node][1]:
            if (not discovered[adjacency_node]) and (adjacency_node not in stack):
                previous_node[adjacency_node] = current_node

                #update color
                edges[edge_id(current_node, adjacency_node)][1] = white
                graph[adjacency_node][3] = red
                graphUI.updateUI()

                #reach the goal
                if (adjacency_node == goal):
                    temp = goal
                    path.append(temp)
                    while (previous_node[temp] != -1):
                        temp = previous_node[temp]
                        path.append(temp)
                    
                    #update path color
                    for i in range(len(path) - 1):
                        edges[edge_id(path[i], path[i + 1])][1] = green
                    graph[goal][3] = purple
                    graph[start][3] = orange
                    graphUI.updateUI()
                    return path
            
                #add adjacency vertex to the stack
                stack.append(adjacency_node)

        #update current vertex as discovered
        discovered[current_node] = True
        graph[current_node][3] = blue
        graphUI.updateUI()

    print("Can't find the path")
    return None
    print("Implement DFS algorithm.")
    pass

#calculate distance between two coordinates
def distance(graph, vertex1, vertex2):
    coord1 = graph[vertex1][0]
    coord2 = graph[vertex2][0]
    dist = pow(coord1[0] - coord2[0], 2) + pow(coord1[1] - coord2[1], 2)
    return sqrt(dist)

def UCS(graph, edges, edge_id, start, goal):
    """
    Uniform Cost Search search
    """
    # TODO: your code
    #first case
    if (goal == start):
        graph[start][3] = orange
        graphUI.updateUI()
        graph[start][3] = purple
        graphUI.updateUI()
        path = [start]
        return path

    #initialize queue, discovered, path, cost and previous_node
    queue = []
    discovered = []
    path = []
    previous_node = []
    cost = []
    for i in range(len(graph)):
        discovered.append(False)
        previous_node.append(-1)
        cost.append(float('inf'))
    
    #add starting vertex to the queue
    cost[start] = 0
    queue.append((start, cost[start]))
    graph[start][3] = orange
    graphUI.updateUI()

    while queue:
        #get the min cost from the queue
        current_node, current_cost = min(queue, key = lambda t: t[1])

        #reach the goal
        if (current_node == goal):
            temp = goal

            #find the path
            path.append(temp)
            while (previous_node[temp] != -1):
                temp = previous_node[temp]
                path.append(temp)
            
            #update path color
            for i in range(len(path) - 1):
                edges[edge_id(path[i], path[i + 1])][1] = green
            graph[goal][3] = purple
            graph[start][3] = orange
            graphUI.updateUI()
            return path

        #remove min cost from the queue
        queue.remove((current_node, current_cost))

        #list of nodes in the queue
        node_queue = [x[0] for x in queue]
        
        #update current vertex color
        graph[current_node][3] = yellow
        graphUI.updateUI()

        #update discovered edges and following vertices
        for adjacency_node in graph[current_node][1]:
            #visit undiscovered vertex
            if (not discovered[adjacency_node]):
                #add adjacency vertex to the queue
                if (adjacency_node not in node_queue):
                    cost[adjacency_node] = current_cost + distance(graph, current_node, adjacency_node)
                    queue.append((adjacency_node, cost[adjacency_node]))
                    previous_node[adjacency_node] = current_node
                else:
                    #update cost value if adjacency vertex has been in the queue
                    for node in node_queue:
                        if (node == adjacency_node and cost[adjacency_node] > current_cost + distance(graph, current_node, adjacency_node)):
                            queue.remove((adjacency_node, cost[adjacency_node]))
                            cost[adjacency_node] = current_cost + distance(graph, current_node, adjacency_node)
                            queue.append((adjacency_node, cost[adjacency_node]))
                            previous_node[adjacency_node] = current_node
                            break

                #update color
                edges[edge_id(current_node, adjacency_node)][1] = white
                graph[adjacency_node][3] = red
                graphUI.updateUI()

        #update current vertex as discovered
        discovered[current_node] = True
        graph[current_node][3] = blue
        graphUI.updateUI()

    print("Can't find the path")
    return None
    print("Implement Uniform Cost Search algorithm.")
    pass


def AStar(graph, edges, edge_id, start, goal):
    """
    A star search
    """
    # TODO: your code
    if (goal == start):
        graph[start][3] = orange
        graphUI.updateUI()
        graph[start][3] = purple
        graphUI.updateUI()
        path = [start]
        return path

    #initialize queue, discovered, path, cost and previous_node
    queue = []
    discovered = []
    path = []
    previous_node = []
    heuristic = []
    cost_from_start = []
    for i in range(len(graph)):
        discovered.append(False)
        previous_node.append(-1)
        heuristic.append(distance(graph, i, goal))
        cost_from_start.append(float('inf'))
    
    #add starting vertex to the queue
    cost_from_start[start] = 0
    queue.append((start, cost_from_start[start] + heuristic[start]))
    graph[start][3] = orange
    graphUI.updateUI()

    while queue:
        #get the min cost from the queue
        current_node, current_cost = min(queue, key = lambda t: t[1])

        #reach the goal
        if (current_node == goal):
            temp = goal

            #find the path
            path.append(temp)
            while (previous_node[temp] != -1):
                temp = previous_node[temp]
                path.append(temp)
            
            #update path color
            for i in range(len(path) - 1):
                edges[edge_id(path[i], path[i + 1])][1] = green
            graph[goal][3] = purple
            graphUI.updateUI()
            return path

        #remove min cost from the queue
        queue.remove((current_node, current_cost))

        #list of nodes in the queue
        node_queue = [x[0] for x in queue]
        
        #update current vertex color
        graph[current_node][3] = yellow
        graphUI.updateUI()

        #update discovered edges and following vertices
        for adjacency_node in graph[current_node][1]:
            #visit undiscovered vertex
            if (not discovered[adjacency_node]):
                #add adjacency vertex to the queue
                if (adjacency_node not in node_queue):
                    cost_from_start[adjacency_node] = cost_from_start[current_node] + distance(graph, current_node, adjacency_node)
                    queue.append((adjacency_node, cost_from_start[adjacency_node] + heuristic[adjacency_node]))
                    previous_node[adjacency_node] = current_node
                else:
                    #update cost value if adjacency vertex has been in the queue
                    for node in node_queue:
                        if (node == adjacency_node and cost_from_start[adjacency_node] > cost_from_start[current_node] + distance(graph, current_node, adjacency_node)):
                            queue.remove((adjacency_node, cost_from_start[adjacency_node] + heuristic[adjacency_node]))
                            cost_from_start[adjacency_node] = cost_from_start[current_node] + distance(graph, current_node, adjacency_node)
                            queue.append((adjacency_node, cost_from_start[adjacency_node] + heuristic[adjacency_node]))
                            previous_node[adjacency_node] = current_node
                            break

                #update color
                edges[edge_id(current_node, adjacency_node)][1] = white
                graph[adjacency_node][3] = red
                graphUI.updateUI()

                #reach the goal
                if (adjacency_node == goal):
                    temp = goal
                    path.append(temp)
                    while (previous_node[temp] != -1):
                        temp = previous_node[temp]
                        path.append(temp)
                    
                    #update path color
                    for i in range(len(path) - 1):
                        edges[edge_id(path[i], path[i + 1])][1] = green
                    graph[goal][3] = purple
                    graph[start][3] = orange
                    graphUI.updateUI()
                    return path

        #update current vertex as discovered
        discovered[current_node] = True
        graph[current_node][3] = blue
        graphUI.updateUI()

    print("Can't find the path")
    return None
    print("Implement A* algorithm.")
    pass


def example_func(graph, edges, edge_id, start, goal):
    """
    This function is just show some basic feature that you can use your project.
    @param graph: list - contain information of graph (same value as global_graph)
                    list of object:
                     [0] : (x,y) coordinate in UI
                     [1] : adjacent node indexes
                     [2] : node edge color
                     [3] : node fill color
                Ex: graph = [
                                [
                                    (139, 140),             # position of node when draw on UI
                                    [1, 2],                 # list of adjacent node
                                    (100, 100, 100),        # grey - node edged color
                                    (0, 0, 0)               # black - node fill color
                                ],
                                [(312, 224), [0, 4, 2, 3], (100, 100, 100), (0, 0, 0)],
                                ...
                            ]
                It means this graph has Node 0 links to Node 1 and Node 2.
                Node 1 links to Node 0,2,3 and 4.
    @param edges: dict - dictionary of edge_id: [(n1,n2), color]. Ex: edges[edge_id(0,1)] = [(0,1), (0,0,0)] : set color
                    of edge from Node 0 to Node 1 is black.
    @param edge_id: id of each edge between two nodes. Ex: edge_id(0, 1) : id edge of two Node 0 and Node 1
    @param start: int - start vertices/node
    @param goal: int - vertices/node to search
    @return:
    """

    # Ex1: Set all edge from Node 1 to Adjacency node of Node 1 is green edges.
    node_1 = graph[1]
    for adjacency_node in node_1[1]:
        edges[edge_id(1, adjacency_node)][1] = green
    graphUI.updateUI()

    # Ex2: Set color of Node 2 is Red
    graph[2][3] = red
    graphUI.updateUI()

    # Ex3: Set all edge between node in a array.
    path = [4, 7, 9]  # -> set edge from 4-7, 7-9 is blue
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = blue
    graphUI.updateUI()
