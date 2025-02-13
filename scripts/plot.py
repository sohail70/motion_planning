import matplotlib.pyplot as plt

# Read data from the text file
nodes = []
edges = []
with open('../build/tree_nodes.txt', 'r') as file:
    for line in file:
        data = line.strip().split(", ")
        node_index = int(data[0])
        parent_index = int(data[1])
        x, y = float(data[2]), float(data[3])
        nodes.append((node_index, parent_index, x, y))
        if parent_index != -1:  # Ignore the root node, which has no parent
            edges.append((node_index, parent_index))  # Record the edge

# Create a figure for plotting
plt.figure(figsize=(10, 10))

# Plot the nodes
for node in nodes:
    node_index, parent_index, x, y = node
    plt.plot(x, y, 'bo', markersize=5)  # Plot the node as a blue point
    # plt.text(x, y, f"{node_index}", fontsize=12, ha='right', color='blue')

# Plot the edges (connect nodes to their parents)
for edge in edges:
    node_index, parent_index = edge
    parent_node = next(node for node in nodes if node[0] == parent_index)  # Find the parent node
    x1, y1 = next((node[2], node[3]) for node in nodes if node[0] == node_index)  # Current node position
    x2, y2 = parent_node[2], parent_node[3]  # Parent node position

    # Plot an edge between the node and its parent
    plt.plot([x1, x2], [y1, y2], 'k-', lw=1)  # Black line for the edge

# Labels and plot aesthetics
plt.title("Tree Structure")
plt.xlabel("X")
plt.ylabel("Y")
plt.grid(True)
plt.show()
