import heapq


weighted_graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'C': 3, 'S': 1, 'A': 2},
    'C': {'A': 2, 'D': 4, 'B': 3, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'D': 1, 'C': 4}
}

def depth_first_search(graph, start, goal):
    stack = [(start, [start])]
    while stack:
        (node, path) = stack.pop()
        for neighbor in set(graph[node].keys()) - set(path):
            if neighbor == goal:
                yield path + [neighbor]
            else:
                stack.append((neighbor, path + [neighbor]))

def breadth_first_search(graph, start, goal):
    queue = [(start, [start])]
    while queue:
        (node, path) = queue.pop(0)
        for neighbor in set(graph[node].keys()) - set(path):
            if neighbor == goal:
                yield path + [neighbor]
            else:
                queue.append((neighbor, path + [neighbor]))

def uniform_cost_search(graph, start, goal):
    priority_queue = [(0, start, [start])]
    while priority_queue:
        (cost, node, path) = heapq.heappop(priority_queue)
        for neighbor in set(graph[node].keys()) - set(path):
            if neighbor == goal:
                yield path + [neighbor], cost + graph[node][neighbor]
            else:
                new_cost = cost + graph[node][neighbor]
                heapq.heappush(priority_queue, (new_cost, neighbor, path + [neighbor]))

def greedy_search(graph, start, goal, heuristic):
    priority_queue = [(heuristic[start], start, [start])]
    while priority_queue:
        (_, node, path) = heapq.heappop(priority_queue)
        for neighbor in set(graph[node].keys()) - set(path):
            if neighbor == goal:
                yield path + [neighbor]
            else:
                heapq.heappush(priority_queue, (heuristic[neighbor], neighbor, path + [neighbor]))

def a_star_search(graph, start, goal, heuristic):
    priority_queue = [(heuristic[start], 0, start, [start])]
    while priority_queue:
        (_, cost, node, path) = heapq.heappop(priority_queue)
        for neighbor in set(graph[node].keys()) - set(path):
            if neighbor == goal:
                yield path + [neighbor], cost + graph[node][neighbor]
            else:
                new_cost = cost + graph[node][neighbor]
                heapq.heappush(priority_queue, (new_cost + heuristic[neighbor], new_cost, neighbor, path + [neighbor]))

start_node = 'S'
goal_node = 'G'

heuristic = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

dfs_path = list(depth_first_search(weighted_graph, start_node, goal_node))[0]
bfs_path = list(breadth_first_search(weighted_graph, start_node, goal_node))[0]
ucs_path, ucs_cost = next(uniform_cost_search(weighted_graph, start_node, goal_node))
greedy_search_path = list(greedy_search(weighted_graph, start_node, goal_node, heuristic))[0]
a_star_search_path, a_star_cost = next(a_star_search(weighted_graph, start_node, goal_node, heuristic))


def format_path(path):
    return " -> ".join(path)

print("DFS Path:", format_path(dfs_path))
print("BFS Path:", format_path(bfs_path))
print("UCS Path:", format_path(ucs_path))
print("Greedy Search Path:", format_path(greedy_search_path))
print("A* Search Path:", format_path(a_star_search_path))
