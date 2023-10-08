from collections import deque
import heapq


weighted_graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'B': 1, 'A': 2, 'C': 3},
    'C': {'A': 2, 'B': 3, 'D': 4, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'C': 4, 'D': 1}
}


heuristics = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}


def bfs(graph, start, goal):
    visited = set()
    queue = deque([(start, [start])])
    expanded_states = []

    while queue:
        node, path = queue.popleft()
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states, [] 
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    queue.append((neighbor, path + [neighbor]))

    return None, expanded_states, [] 

def dfs(graph, start, goal):
    visited = set()
    stack = [(start, [start])]
    expanded_states = []

    while stack:
        node, path = stack.pop()
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states, [] 
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in reversed(neighbors):
                if neighbor not in visited:
                    stack.append((neighbor, path + [neighbor]))

    return None, expanded_states, []  



def ucs(graph, start, goal):
    visited = set()
    priority_queue = [(0, start, [start])]
    expanded_states = []

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states, []  
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    new_cost = cost + graph[node][neighbor]
                    heapq.heappush(priority_queue, (new_cost, neighbor, path + [neighbor]))

    return None, expanded_states, []  



def astar(graph, start, goal, heuristics):
    visited = set()
    priority_queue = [(heuristics[start], start, [start])]
    expanded_states = []

    while priority_queue:
        if len(priority_queue) == 0:
            break 
        _, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states

        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    cost = path_cost(path) + graph[node][neighbor]
                    heapq.heappush(priority_queue, (cost + heuristics[neighbor], neighbor, path + [neighbor]))

    return None, expanded_states

def greedy(graph, start, goal, heuristics):
    visited = set()
    priority_queue = [(heuristics[start], start, [start])]
    expanded_states = []

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return path, expanded_states
        if node not in visited:
            visited.add(node)
            neighbors = sorted(graph[node].keys())
            for neighbor in neighbors:
                if neighbor not in visited:
                    heapq.heappush(priority_queue, (heuristics[neighbor], neighbor, path + [neighbor]))

    unexpanded_states = [node for node in graph if node not in visited]
    return None, expanded_states, unexpanded_states

def path_cost(path):
    cost = 0
    for i in range(len(path) - 1):
        cost += weighted_graph[path[i]][path[i + 1]]
    return cost


start_node = 'S'
goal_node = 'G'

bfs_result = bfs(weighted_graph, start_node, goal_node)
dfs_result = dfs(weighted_graph, start_node, goal_node)
ucs_result = ucs(weighted_graph, start_node, goal_node)
astar_result = astar(weighted_graph, start_node, goal_node, heuristics)
greedy_result = greedy(weighted_graph, start_node, goal_node, heuristics)



bfs_path, bfs_expanded, bfs_unexpanded = bfs(weighted_graph, start_node, goal_node)
if bfs_path is not None:
    print("Breadth First Search Path:", bfs_path)
else:
    print("Breadth First Search did not find a path.")
print("Breadth First Search Expanded States:", bfs_expanded)
print("Breadth First Search Unexpanded States:", bfs_unexpanded)

dfs_path, dfs_expanded, dfs_unexpanded = dfs(weighted_graph, start_node, goal_node)
if dfs_path is not None:
    print("Depth First Search Path:", dfs_path)
else:
    print("Depth First Search did not find a path.")
print("Depth First Search Expanded States:", dfs_expanded)
print("Depth First Search Unexpanded States:", dfs_unexpanded)


ucs_path, ucs_expanded, ucs_unexpanded = ucs(weighted_graph, start_node, goal_node)
if ucs_path is not None:
    print("Uniform Cost Search Path:", ucs_path)
else:
    print("Uniform Cost Search did not find a path.")
print("Uniform Cost Search Expanded States:", ucs_expanded)
print("Uniform Cost Search Unexpanded States:", ucs_unexpanded)


astar_path, astar_expanded = astar(weighted_graph, start_node, goal_node, heuristics)


if astar_path is not None:
    print("A* Search Path:", astar_path)
    print("A* Search Expanded States:", astar_expanded)
else:
    print("A* Search did not find a path.")



greedy_path, greedy_expanded = greedy(weighted_graph, start_node, goal_node, heuristics)

if greedy_path is not None:
    print("Greedy Search Path:", greedy_path)
    print("Greedy Search Expanded States:", greedy_expanded)
else:
    print("Greedy Search did not find a path.")




