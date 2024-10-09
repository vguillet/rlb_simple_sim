import os
from rlb_simple_sim.rlb_simple_sim.Scenario import Scenario
from graph_env.graph_env.graph_generator import *
from maaf_tools.maaf_tools.tools import dumps

# -> Iterate through every file in the directory
path = "/home/vguillet/ros2_ws/src/rlb_simple_sim/rlb_simple_sim/Configs"
shortest_paths_path = "/home/vguillet/ros2_ws/src/rlb_simple_sim/rlb_simple_sim/Parsed_maps"

for file in os.listdir(path):
    # -> Shortest path file name
    shortest_path_file = f"{file.split('_')[1]}_{file.split('_')[2]}_shortest_paths.json"

    # -> Check if the file exists
    if os.path.exists(shortest_paths_path + "/" + shortest_path_file):
        print(f"File exists: {shortest_path_file}")
        continue

    print(f"File does not exist: {shortest_path_file}")

    scenario = Scenario(scenario_id=file, load_only=True)

    # -> Get the launch parameters
    graph_type = scenario.environment_type
    num_nodes = scenario.env_size * scenario.env_size
    connectivity_percent = scenario.env_connectivity
    num_branches = 0  # TODO: Cleanup
    graph_seed = scenario.seed

    # -> Generate the graph
    if graph_type == "grid":
        graph, pos = generate_grid_layout(
            num_nodes=num_nodes,
            connectivity_percent=connectivity_percent
        )

    elif graph_type == "star":
        graph, pos = generate_star_layout(
            num_nodes=num_nodes,
            num_branches=num_branches
        )

    elif graph_type == "random":
        graph, pos = generate_random_layout(
            num_nodes=num_nodes
        )

    elif graph_type == "MAPF":
        graph, pos = generate_benchmark_layout(
            map_file=scenario.environment_path
        )

    # ----- Compute shortest paths from all to tasks
    # -> Retrieve path from scenario ID
    shortest_paths_path = f"/home/vguillet/ros2_ws/src/rlb_simple_sim/rlb_simple_sim/Parsed_maps/{scenario.scenario_id.split('_')[1]}_{scenario.scenario_id.split('_')[2]}_shortest_paths.json"

    # -> List all task nodes from scenario
    task_node_locs = [(goto_task["instructions"]["x"], goto_task["instructions"]["y"]) for goto_task in
                      scenario.goto_tasks]

    # -> Compute all shortest paths from all nodes to all task nodes
    matching_nodes = [node for node, position in pos.items() if position in task_node_locs]

    print(f"\nMatching nodes ({len(matching_nodes)}/{len(scenario.goto_tasks)}):", matching_nodes)

    # > Print missing nodes
    missing_nodes = [task_node_loc for task_node_loc in task_node_locs if
                     task_node_loc not in pos.values()]
    print(f"Missing nodes  ({len(missing_nodes)}/{len(scenario.goto_tasks)}):", missing_nodes)

    # -> Compute all shortest paths from all nodes to all task nodes
    print("\nComputing all shortest paths from all nodes to all task nodes...")
    all_pairs_shortest_paths = {}

    for i, task_node_loc in enumerate(task_node_locs):
        print(f"> Computing shortest paths - {i + 1}/{len(task_node_locs)}")

        # > Find node corresponding to task
        task_node = [node for node, position in pos.items() if position == task_node_loc][0]

        # > Find paths from node to all other nodes
        paths = dict(nx.single_source_shortest_path(G=graph, source=task_node))

        # > Add paths from each node to the task node
        for source_node, path in paths.items():
            # > Invert path
            path.reverse()

            # > Record path from source to task node
            if source_node not in all_pairs_shortest_paths.keys():
                all_pairs_shortest_paths[source_node] = {}

            all_pairs_shortest_paths[source_node][task_node] = path

    # -> Cache results to file
    with open(shortest_paths_path, "w") as f:
        all_pairs_shortest_paths = {str(k): {str(k2): v2 for k2, v2 in v.items()} for k, v in
                                    all_pairs_shortest_paths.items()}
        f.write(dumps(all_pairs_shortest_paths))

        print(f"Saved shortest paths to file: {scenario.scenario_id}")