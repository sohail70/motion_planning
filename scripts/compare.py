import pandas as pd
import networkx as nx
import matplotlib.pyplot as plt
import numpy as np
import os
import time 



def load_graph_from_csv(filename: str) -> tuple[nx.DiGraph | None, int | None, int]:
    """
    Loads a graph from a CSV file into a networkx DiGraph.
    The CSV should have columns: node_id, x0, x1, ..., parent_id.
    Node coordinates are stored as a 'coords' attribute (tuple).
    Returns the graph, the root node's ID, and the dimension of coordinates.
    """
    try:
        df = pd.read_csv(filename)
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        return None, None, 0
    except pd.errors.EmptyDataError:
        print(f"Error: File '{filename}' is empty.")
        return None, None, 0
    except Exception as e:
        print(f"An unexpected error occurred while reading '{filename}': {e}")
        return None, None, 0

    if df.empty:
        print(f"Warning: CSV file '{filename}' is empty. No graph to load.")
        return nx.DiGraph(), None, 0

    graph = nx.DiGraph()
    coord_cols = sorted(
        [col for col in df.columns if col.startswith('x') and col[1:].isdigit()],
        key=lambda col_name: int(col_name[1:])
    )
    dimension = len(coord_cols)
    
    if dimension == 0 and 'node_id' in df.columns and not df['node_id'].empty:
        print(f"Warning: No coordinate columns (x0, x1, ...) found in '{filename}'. Nodes will be added without positions.")
        root_node_id_fallback = None
        for _, row in df.iterrows():
            node_id = int(row['node_id'])
            graph.add_node(node_id, coords=None)
            if 'parent_id' in row:
                parent_id_val = row['parent_id']
                if pd.notna(parent_id_val):
                    parent_id = int(parent_id_val)
                    if parent_id != -1:
                        graph.add_edge(parent_id, node_id)
                    else:
                        if root_node_id_fallback is not None and node_id != root_node_id_fallback:
                             # print(f"Warning: Multiple roots detected in '{filename}' during fallback. Found {node_id}, already had {root_node_id_fallback}.") # Silenced
                             pass
                        elif root_node_id_fallback is None:
                            root_node_id_fallback = node_id
            else:
                 print(f"Warning: 'parent_id' column missing in '{filename}' during fallback for node {node_id}.")
        return graph, root_node_id_fallback, 0
    elif dimension == 0:
        print(f"Warning: No coordinate columns (x0, x1, ...) found or no data in '{filename}'.")
        return graph, None, 0

    root_node_id = None

    if not all(col in df.columns for col in ['node_id', 'parent_id']):
        print(f"Error: Essential columns ('node_id', 'parent_id') are missing from '{filename}'.")
        return None, None, 0

    for _, row in df.iterrows():
        try:
            node_id = int(row['node_id'])
            coords = tuple(row[coord_cols].astype(float).values) if dimension > 0 else None
            graph.add_node(node_id, coords=coords) 

            parent_id_val = row['parent_id']
            if pd.isna(parent_id_val):
                if root_node_id is None: root_node_id = node_id 
                continue

            parent_id = int(parent_id_val)
            if parent_id != -1:
                if parent_id not in graph:
                     graph.add_node(parent_id, coords=None)
                     # print(f"Warning: Parent node {parent_id} for node {node_id} not yet defined. Added parent provisionally.") # Silenced
                graph.add_edge(parent_id, node_id)
            else: 
                if root_node_id is not None and node_id != root_node_id :
                    # print(f"Warning: Multiple roots (parent_id == -1) detected in '{filename}'. Using the first one encountered: {root_node_id}.") # Silenced
                    pass
                elif root_node_id is None:
                     root_node_id = node_id
        except ValueError as e:
            print(f"Error processing row data in '{filename}': {row.to_dict()}. Error: {e}. Skipping row.")
            continue
        except Exception as e:
            print(f"Unexpected error processing row in '{filename}': {row.to_dict()}. Error: {e}. Skipping row.")
            continue
    
    if root_node_id is None and graph.number_of_nodes() > 0:
         print(f"Warning: No root node (parent_id == -1) found in '{filename}', though graph has nodes.")

    return graph, root_node_id, dimension


def compare_node_coords_match(attrs1, attrs2, tolerance=1e-7):
    coords1 = attrs1.get('coords')
    coords2 = attrs2.get('coords')
    if coords1 is None or coords2 is None: return False
    if len(coords1) != len(coords2): return False
    # Ensure coords are numpy arrays for allclose
    c1_arr = np.asarray(coords1)
    c2_arr = np.asarray(coords2)
    if c1_arr.shape != c2_arr.shape: return False # Should be caught by len check, but good practice
    return np.allclose(c1_arr, c2_arr, atol=tolerance)

def analyze_graph_similarity(graph1, graph2, tolerance=1e-7, calculate_detailed_scores=False):
    """
    Checks for isomorphism and calculates similarity metrics.
    The detailed_scores (node overlap, parent agreement) can be slow.
    """
    print("\n--- Graph Similarity Analysis ---")

    if graph1 is None or graph2 is None or graph1.number_of_nodes() == 0 or graph2.number_of_nodes() == 0:
        print("One or both graphs are empty or could not be loaded. Cannot perform similarity analysis.")
        return False # Indicate isomorphism check failed or N/A

    # Isomorphism Check (structure and node coordinates)
    print("Checking for perfect isomorphism (structure & node coordinates)...")
    is_isomorphic_with_coords = nx.is_isomorphic(
        graph1, graph2, 
        node_match=lambda n1_attrs, n2_attrs: compare_node_coords_match(n1_attrs, n2_attrs, tolerance)
    )

    if is_isomorphic_with_coords:
        print("Result: Graphs ARE isomorphic (identical structure and node coordinates within tolerance).")
        print("Similarity: 100%")
        return True
    else:
        print("Result: Graphs are NOT perfectly isomorphic (considering structure and node coordinates).")

        # Basic Count Differences
        nodes1, nodes2 = graph1.number_of_nodes(), graph2.number_of_nodes()
        edges1, edges2 = graph1.number_of_edges(), graph2.number_of_edges()
        print(f"\nNode Counts: G1={nodes1}, G2={nodes2} (Difference: {abs(nodes1 - nodes2)})")
        print(f"Edge Counts: G1={edges1}, G2={edges2} (Difference: {abs(edges1 - edges2)})")

        if calculate_detailed_scores:
            print("\nCalculating detailed similarity scores (this may take time)...")
            
            # Node Coordinate Overlap Score
            coords_list1_with_ids = {node_id: data['coords'] for node_id, data in graph1.nodes(data=True) if data.get('coords') is not None}
            coords_list2_with_ids = {node_id: data['coords'] for node_id, data in graph2.nodes(data=True) if data.get('coords') is not None}

            if not coords_list1_with_ids or not coords_list2_with_ids:
                print("One or both graphs lack coordinate data for nodes; cannot compute detailed scores.")
            else:
                print("  Calculating Node Coordinate Overlap Score...")
                overlap1_in_2 = 0
                g1_to_g2_coord_matches = {} # Store n1_id -> n2_id for matched nodes

                for n1_id, c1 in coords_list1_with_ids.items():
                    for n2_id, c2 in coords_list2_with_ids.items():
                        if compare_node_coords_match({'coords':c1}, {'coords':c2}, tolerance):
                            overlap1_in_2 += 1
                            g1_to_g2_coord_matches[n1_id] = n2_id
                            break 
                
                overlap2_in_1 = 0 # For symmetry, though g1_to_g2_coord_matches could be used
                for n2_id, c2 in coords_list2_with_ids.items():
                    for n1_id, c1 in coords_list1_with_ids.items():
                         if compare_node_coords_match({'coords':c1}, {'coords':c2}, tolerance):
                            overlap2_in_1 += 1
                            break
                
                score1 = (overlap1_in_2 / len(coords_list1_with_ids)) * 100 if coords_list1_with_ids else 0
                score2 = (overlap2_in_1 / len(coords_list2_with_ids)) * 100 if coords_list2_with_ids else 0
                
                print(f"    Node Coordinate Overlap:")
                print(f"      {score1:.2f}% of G1 nodes have a coordinate match in G2.")
                print(f"      {score2:.2f}% of G2 nodes have a coordinate match in G1.")
                print(f"      Average Coordinate Overlap: {((score1 + score2) / 2):.2f}%")

                # Parent Agreement Score (using g1_to_g2_coord_matches)
                print("  Calculating Parent Agreement Score...")
                parent_agreements = 0
                nodes_with_parents_and_match_in_g1 = 0
                
                g1_parents_map = {n: list(graph1.predecessors(n)) for n in graph1.nodes()}
                g2_parents_map = {n: list(graph2.predecessors(n)) for n in graph2.nodes()}

                for n1_id, matched_n2_id in g1_to_g2_coord_matches.items():
                    p1_list = g1_parents_map.get(n1_id, [])
                    p2_list = g2_parents_map.get(matched_n2_id, [])

                    if p1_list and p2_list: # Both n1 and its matched n2 have parents
                        nodes_with_parents_and_match_in_g1 += 1
                        p1_id = p1_list[0] # Assuming single parent for tree nodes
                        p2_id = p2_list[0]

                        p1_attrs = graph1.nodes.get(p1_id, {})
                        p2_attrs = graph2.nodes.get(p2_id, {})

                        if compare_node_coords_match(p1_attrs, p2_attrs, tolerance):
                            parent_agreements += 1
                
                if nodes_with_parents_and_match_in_g1 > 0:
                    parent_agreement_score = (parent_agreements / nodes_with_parents_and_match_in_g1) * 100
                    print(f"    Parent Agreement Score: {parent_agreement_score:.2f}%")
                    print(f"      (Of G1 nodes that coord-match in G2 & both have parents, this % have parents that also coord-match)")
                else:
                    print("    Could not calculate Parent Agreement Score (no G1 nodes with matches in G2 and parents found).")
        else:
            print("\nDetailed similarity scores calculation skipped for speed.")
        
        print("\nFor more advanced structural similarity (can be slow for large graphs):")
        print("  - Consider nx.graph_edit_distance(graph1, graph2, node_match=..., ...)")
        return False


def plot_overlaid_graphs(file1: str, file2: str, graph1: nx.DiGraph, graph2: nx.DiGraph, root1_id, root2_id, dim1, dim2, output_filename: str | None = "overlaid_graphs.png"):
    """
    Plots two loaded graphs overlaid.
    Assumes 2D coordinates for plotting (uses first two coordinate columns, e.g., x0, x1).
    Takes loaded graph objects as input.
    """
    g1_empty = graph1.number_of_nodes() == 0
    g2_empty = graph2.number_of_nodes() == 0

    if g1_empty and g2_empty: return 
    if g1_empty or g2_empty: return 

    has_coords1 = any(data.get('coords') and len(data['coords']) >= 2 for _, data in graph1.nodes(data=True) if data)
    has_coords2 = any(data.get('coords') and len(data['coords']) >= 2 for _, data in graph2.nodes(data=True) if data)

    if not (dim1 >= 2 and has_coords1):
        print(f"Graph from '{file1}' does not have valid 2D coordinate data for plotting.")
        return 
    if not (dim2 >= 2 and has_coords2):
        print(f"Graph from '{file2}' does not have valid 2D coordinate data for plotting.")
        return

    plt.figure(figsize=(14, 12))
    ax = plt.gca()

    pos1 = {node: (data['coords'][0], data['coords'][1]) for node, data in graph1.nodes(data=True) if data.get('coords') and len(data['coords']) >= 2}
    pos2 = {node: (data['coords'][0], data['coords'][1]) for node, data in graph2.nodes(data=True) if data.get('coords') and len(data['coords']) >= 2}

    if graph1.number_of_nodes() > 0 and pos1:
        nx.draw_networkx_nodes(graph1, pos1, ax=ax, node_size=35, node_color='blue', alpha=0.7, label="Graph 1 Nodes")
        nx.draw_networkx_edges(graph1, pos1, ax=ax, edge_color='blue', alpha=0.4, width=1.2, label="Graph 1 Edges")
        if root1_id in pos1:
             nx.draw_networkx_nodes(graph1, pos1, nodelist=[root1_id], node_size=120, node_color='cyan', edgecolors='black', linewidths=1.5, label="Graph 1 Root")

    if graph2.number_of_nodes() > 0 and pos2:
        nx.draw_networkx_nodes(graph2, pos2, ax=ax, node_size=25, node_color='red', alpha=0.7, label="Graph 2 Nodes", node_shape='s')
        nx.draw_networkx_edges(graph2, pos2, ax=ax, edge_color='red', alpha=0.4, width=1.2, style='dashed', label="Graph 2 Edges")
        if root2_id in pos2:
            nx.draw_networkx_nodes(graph2, pos2, nodelist=[root2_id], node_size=120, node_color='magenta', edgecolors='black', linewidths=1.5, node_shape='s', label="Graph 2 Root")

    plt.title(f"Overlay of {os.path.basename(file1)} (Blue/Circle) and {os.path.basename(file2)} (Red/Square)", fontsize=14)
    plt.xlabel("X0 Coordinate", fontsize=12)
    plt.ylabel("X1 Coordinate", fontsize=12)
    ax.tick_params(axis='both', which='major', labelsize=10, left=True, bottom=True, labelleft=True, labelbottom=True)
    plt.axis('on') 
    ax.set_aspect('equal', adjustable='box') 
    
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = {}
    if handles and labels:
        for handle, label in zip(handles, labels):
            if label not in by_label: by_label[label] = handle
    
    if by_label:
        plt.legend(by_label.values(), by_label.keys(), loc='best', fontsize='medium', frameon=True, fancybox=True, shadow=True, borderpad=1)
    
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.tight_layout()
    
    if output_filename:
        try:
            plt.savefig(output_filename, dpi=300, bbox_inches='tight')
            print(f"Plot saved to {output_filename}")
        except Exception as e:
            print(f"Error saving plot to '{output_filename}': {e}")
    plt.show()


if __name__ == "__main__":
    file_path1 = "../build/tree_run_rrtx.csv"
    file_path2 = "../build/tree_run_fmtx.csv"
    output_plot_filename = "actual_overlay.png"
    tolerance_val = 1e-7 # Tolerance for coordinate comparison
    
    # --- Control whether to run the slow detailed similarity scores ---
    run_detailed_similarity_scores = True  # Set to True to calculate Node Overlap and Parent Agreement
                                           # Set to False for faster execution (skips these detailed scores)

    load_start_time = time.time()
    print(f"Loading Graph 1 from: {file_path1}")
    graph1, root1_id, dim1 = load_graph_from_csv(file_path1)
    print(f"Loading Graph 2 from: {file_path2}")
    graph2, root2_id, dim2 = load_graph_from_csv(file_path2)
    load_end_time = time.time()
    print(f"Time to load graphs: {load_end_time - load_start_time:.2f} seconds")

    if graph1 and graph2:
        analysis_start_time = time.time()
        analyze_graph_similarity(graph1, graph2, 
                                 tolerance=tolerance_val, 
                                 calculate_detailed_scores=run_detailed_similarity_scores) # Pass the flag
        analysis_end_time = time.time()
        print(f"Time for similarity analysis: {analysis_end_time - analysis_start_time:.2f} seconds")
        
        if graph1.number_of_nodes() > 0 and graph2.number_of_nodes() > 0:
            print(f"\nAttempting to plot overlay for '{file_path1}' and '{file_path2}'")
            plot_start_time = time.time()
            plot_overlaid_graphs(file_path1, file_path2, graph1, graph2, root1_id, root2_id, dim1, dim2, output_plot_filename)
            plot_end_time = time.time()
            print(f"Time for plotting function: {plot_end_time - plot_start_time:.2f} seconds")
        else:
            print("Skipping plotting as one or both loaded graphs are effectively empty.")
    else:
        print("Skipping analysis and plotting as one or both graphs failed to load properly.")
