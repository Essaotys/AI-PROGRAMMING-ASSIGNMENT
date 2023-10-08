import heapq
weighted_graph = {
    'S': {'neighbors': {'A': 3, 'B': 1}, 'heuristic': 7},
    'A': {'neighbors': {'B': 2, 'C': 2}, 'heuristic': 5},
    'B': {'neighbors': {'C': 3}, 'heuristic': 7},
    'C': {'neighbors': {'D': 4, 'G': 4}, 'heuristic': 4},
    'D': {'neighbors': {'G': 1}, 'heuristic': 1},
    'G': {'neighbors': {}, 'heuristic': 0}
}

# Depth-First Search 
def depth_first_search_tree(graph, start, goal):
    stack = [(start, [])]
    visited = set()
    expanded_states = []

    while stack:
        node, path = stack.pop()
        expanded_states.append(node)
        if node == goal:
            return (
                "Depth-First Search:",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)
            neighbors = [neighbor for neighbor in graph[node]['neighbors']]
            stack.extend((neighbor, path + [node]) for neighbor in reversed(neighbors) if neighbor not in visited)

# Breadth-First Search 
def breadth_first_search_tree(graph, start, goal):
    queue = [(start, [])]
    visited = set()
    expanded_states = []

    while queue:
        node, path = queue.pop(0)
        expanded_states.append(node)
        if node == goal:
            return (
                "Breadth-First Search :",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)
            neighbors = [neighbor for neighbor in graph[node]['neighbors']]
            queue.extend((neighbor, path + [node]) for neighbor in neighbors if neighbor not in visited)

# Uniform Cost Search 
def uniform_cost_search_tree(graph, start, goal):
    priority_queue = [(0, start, [])]
    visited = set()
    expanded_states = []

    while priority_queue:
        cost, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return (
                "Uniform Cost Search:",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)
            neighbors = [(neighbor, cost + graph[node]['neighbors'][neighbor]) for neighbor in graph[node]['neighbors']]
            for neighbor, new_cost in neighbors:
                heapq.heappush(priority_queue, (new_cost, neighbor, path + [node]))

# Greedy Search
def greedy_search_tree(graph, start, goal):
    priority_queue = [(graph[start]['heuristic'], start, [])]
    visited = set()
    expanded_states = []

    while priority_queue:
        _, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return (
                "Greedy Search :",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)
            neighbors = [neighbor for neighbor in graph[node]['neighbors']]
            priority_queue.extend((graph[neighbor]['heuristic'], neighbor, path + [node]) for neighbor in neighbors if neighbor not in visited)

# A* Search 
def a_star_search_tree(graph, start, goal):
    priority_queue = [(graph[start]['heuristic'], 0, start, [])]
    visited = set()
    expanded_states = []

    while priority_queue:
        _, cost_so_far, node, path = heapq.heappop(priority_queue)
        expanded_states.append(node)
        if node == goal:
            return (
                "A* Search :",
                expanded_states,
                path + [node],
                list(set(graph.keys()) - set(expanded_states))
            )
        if node not in visited:
            visited.add(node)
            neighbors = [(neighbor, cost_so_far + graph[node]['neighbors'][neighbor]) for neighbor in graph[node]['neighbors']]
            for neighbor, new_cost in neighbors:
                heapq.heappush(priority_queue, (new_cost + graph[neighbor]['heuristic'], new_cost, neighbor, path + [node]))

# Format and print results
def print_search_results(result):
    search_name, expanded_states, path, not_expanded = result
    print(f"{search_name}\nOrder: {expanded_states}\nPath: {path}\nNot Expanded: {not_expanded}\n\n")

# Test the search algorithms for Tree Search
goal_state = 'G'
dfs_result = depth_first_search_tree(weighted_graph, 'S', goal_state)
bfs_result = breadth_first_search_tree(weighted_graph, 'S', goal_state)
ucs_result = uniform_cost_search_tree(weighted_graph, 'S', goal_state)
greedy_result = greedy_search_tree(weighted_graph, 'S', goal_state)
a_result = a_star_search_tree(weighted_graph, 'S', goal_state)

# Print results
print_search_results(dfs_result)
print_search_results(bfs_result)
print_search_results(ucs_result)
print_search_results(greedy_result)
print_search_results(a_result)
