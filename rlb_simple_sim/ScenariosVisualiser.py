
from json import load
import os
from tabulate import tabulate


# -> Load all scenarios jsons
scenarios = []

for config in os.listdir("/home/vguillet/ros2_ws/src/rlb_simple_sim/rlb_simple_sim/Configs"):
    with open(f"/home/vguillet/ros2_ws/src/rlb_simple_sim/rlb_simple_sim/Configs/{config}", "r") as f:
        scenarios.append(load(f))

print(f">> Loaded {len(scenarios)} scenarios")

# -> Sort results in by scenario_id
scenarios.sort(key=lambda x: x["scenario_id"])

settings = [
    "seed",
    "env_size",
    "env_connectivity",
    "goto_tasks_count",
    "tasks_types_ratios",
    "no_tasks_count",
    "action_1_tasks_count",
    "action_2_tasks_count",
    "initial_tasks_announcement",
    "release_max_epoch",
    "agent_lst",
    "fleet_skillsets",
    "fleet_bids_mechanisms",
    "recompute_bids_on_state_change",
    "with_interceding",
    "intercession_targets",
]

# -> Sort results in by scenario_id
scenarios.sort(key=lambda x: x["scenario_id"])

config_table = []

for config in scenarios:
    row = []

    for setting in settings:
        row.append(config[setting])

    config_table.append(row)

print(tabulate(config_table, headers=settings))