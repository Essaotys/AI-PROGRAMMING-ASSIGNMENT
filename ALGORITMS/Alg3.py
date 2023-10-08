from collections import deque
import heapq
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'C': 3, 'S': 1, 'A': 2},
    'C': {'A': 2, 'D': 4, 'B': 3, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'D': 1, 'C': 4}
}   
print ('BREADTH FIRST SEARCH')
def breadth_first_search(graph, start, goal):
    queue = deque([(start, [start])])  
    visited = set()

    while queue:
        current_node, path = queue.popleft()

        if current_node == goal:
            return path

        if current_node not in visited:
            visited.add(current_node)
            for neighbor in graph.get(current_node, []):
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

    return None  




start_node = 'S'
goal_node = 'G'


bfs_path = breadth_first_search(graph, start_node, goal_node)

if bfs_path:
    print(" Path 'S' to 'G':", ' -> '.join(bfs_path))
else:
    print("No valid path ")
print ('DEPTH FIRST SEARCH')
def dfs(graph, node, visited=None, path=None):
    if visited is None:
        visited = set()
    if path is None:
        path = []

    visited.add(node)
    path.append(node)

    for neighbor in graph.get(node, {}):
        if neighbor not in visited:
            dfs(graph, neighbor, visited, path)

    return path


dfs_path = dfs(graph, 'S')

if dfs_path:
    print("Path 'S' to 'G':", ' -> '.join(dfs_path))
else:
    print("No valid path .")

print ('UNIFORM COST SEARCH')
def dijkstra(graph, start, goal):
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    previous_nodes = {node: None for node in graph}
    priority_queue = [(0, start)]

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_node == goal:
            path = []
            while previous_nodes[current_node] is not None:
                path.insert(0, current_node)
                current_node = previous_nodes[current_node]
            path.insert(0, start)  
            return distances[goal], path

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node].items():
            total_cost = current_distance + weight
            if total_cost < distances[neighbor]:
                distances[neighbor] = total_cost
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (total_cost, neighbor))

    return float('infinity'), []  
least_uniform_cost_to_G, path_to_G = dijkstra(graph, 'S', 'G')

#print(" Uniform Cost from 'S' to 'G':", least_uniform_cost_to_G)
print("Path 'S' to 'G':", ' -> '.join(path_to_G))
print ('THE GREEDY SEARCH')
def greedy_search(graph, start, goal, heuristic):
    visited = set()
    path = []
    current_node = start
    
    while current_node != goal:
        if current_node in visited:
            return None  
        path.append(current_node)
        visited.add(current_node)
        neighbors = graph[current_node]
        if not neighbors:
            return None  
        current_node = min(neighbors, key=lambda node: neighbors[node] + heuristic[node])  
        
    path.append(goal)
    return path
     
heuristic_values = {'S': 7, 'B': 7, 'A': 5, 'C': 4, 'D': 1, 'G': 0}

path = greedy_search(graph, 'S', 'G', heuristic_values)

if path:
    print(" Path from 'S' to 'G':", ' -> '.join(path))
else:
    print("No valid path found from 'S' to 'G' using Greedy Search.")
print ('A* SEARCH')
 
def astar_search(graph, start, goal, heuristic):
    open_set = [(0, start)]  
    g_values = {node: float('infinity') for node in graph}
    g_values[start] = 0
    came_from = {} 

    while open_set:
        f, current_node = heapq.heappop(open_set)

        if current_node == goal:
           
            path = []
            while current_node != start:
                path.insert(0, current_node)
                current_node = came_from[current_node]
            path.insert(0, start)
            return path

        for neighbor, weight in graph.get(current_node, {}).items():
            tentative_g = g_values[current_node] + weight
            if tentative_g < g_values.get(neighbor, float('infinity')):
                
                g_values[neighbor] = tentative_g
                f = tentative_g + heuristic.get(neighbor, 0)  
                heapq.heappush(open_set, (f, neighbor))
                came_from[neighbor] = current_node

    return None  # No path found

heuristic_values = {
    'S': 7,
    'B': 7,
    'A': 5,
    'C': 4,
    'D': 1,
    'G': 0
}


start_node = 'S'
goal_node = 'G'


path = astar_search(graph, start_node, goal_node, heuristic_values)

if path:
    print("Path 'S' to 'G' :", ' -> '.join(path))
else:
    print("No valid path found ")
