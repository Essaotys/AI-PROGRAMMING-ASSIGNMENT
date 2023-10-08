
weighted_graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'S': 3, 'B': 2, 'C': 2},
    'B': {'C': 3, 'S': 1, 'A': 2},
    'C': {'A': 2, 'D': 4, 'B': 3, 'G': 4},
    'D': {'C': 4, 'G': 1},
    'G': {'D': 1, 'C': 4}
}

def extract_graph_info(graph):
    nodes = set(graph.keys())
    edges = []
    for node, neighbors in graph.items():
        for neighbor, weight in neighbors.items():
            edges.append((node, neighbor, weight))
    return nodes, edges


nodes, edges = extract_graph_info(weighted_graph)

print("Nodes:", nodes)
print("Edges with weights:")
for edge in edges:
    print(edge)
