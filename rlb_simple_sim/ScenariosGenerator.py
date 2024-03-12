
##################################################################################################################

"""
Parent class for the CAF framework. To use, the following must be defined in the child class:
MAF:
    Optional:
    - on_set_state

CAF:
    - message_to_publish(self) (property)
    - process_msg(self, msg)
    - next_state(self) (property)
    # - action_to_take(self) (property)
    # - process_step(self, obs, reward, done, infos)

    Optional:
    - on_new_task(self, task_id, task_data)
"""

# Built-in/Generic Imports
import os
from abc import abstractmethod
from typing import List, Optional
from datetime import datetime, timedelta
from json import dumps, loads
from pprint import pprint, pformat
import warnings
from copy import deepcopy
from math import ceil
import random

# Libs
import numpy as np

# Suppress FutureWarning messages

# ROS2 Imports

# Local Imports

######################################################################################################


SOLO_GEN = 0
PAIRS_GEN = 1


class ScenariosGenerator:
    def __init__(self):
        seed = 0
        self.rng = np.random.RandomState(seed=seed)

        self.scenarios_count = 10

        # ----- Allocation protocol
        self.recompute_bids_on_state_change = True
        self.with_interceding = False

        # ----- Environment
        self.env_connectivity_range = [0.55, 0.85]
        self.env_size_range = [16, 19]
        
        # ----- Tasks
        self.goto_tasks_count_range = [50, 150]
        
        self.no_action_tasks_fraction_range = [1, 3]
        self.action1_tasks_fraction_range = [1, 3]
        self.action2_tasks_fraction_range = [1, 3]

        self.initial_tasks_announcement_fraction_range = [0.1, 0.5]
        self.release_max_epoch_range = [100, 200]

        # ----- Agents
        self.agent_lst = ["Turtle_1", "Turtle_2", "Turtle_3", "Turtle_4"]
        self.skills = ["GOTO", "ACTION_1", "ACTION_2"]

        self.action_1_freq_range = [1, 3]
        self.action_2_freq_range = [1, 3]
        self.no_action_freq_range = [1, 3]
        
        self.fleets_skillsets = [
            {
                "Turtle_1": ["GOTO", "ACTION_1"],
                "Turtle_2": ["GOTO", "ACTION_1"],
                "Turtle_3": ["GOTO", "ACTION_2"],
                "Turtle_4": ["GOTO", "ACTION_2"],
            },
            {
                "Turtle_1": ["GOTO", "ACTION_1"],
                "Turtle_2": ["GOTO", "ACTION_2"],
                "Turtle_3": ["GOTO", "ACTION_2"],
                "Turtle_4": ["GOTO", "ACTION_2"],
            },
            {
                "Turtle_1": ["GOTO", "ACTION_1"],
                "Turtle_2": ["GOTO"],
                "Turtle_3": ["GOTO", "ACTION_2"],
                "Turtle_4": ["GOTO", "ACTION_2"],
            },
            {
                "Turtle_1": ["GOTO", "ACTION_1"],
                "Turtle_2": ["GOTO"],
                "Turtle_3": ["GOTO"],
                "Turtle_4": ["GOTO", "ACTION_2"],
            }
        ]
        
        self.bids_functions = [
            "anticipated_action_task_interceding_agent",
            "graph_weighted_manhattan_distance_bid"
        ]
        
        self.fleet_bids_mechanisms = [
            {
                "Turtle_1": "anticipated_action_task_interceding_agent",
                "Turtle_2": "graph_weighted_manhattan_distance_bid",
                "Turtle_3": "graph_weighted_manhattan_distance_bid",
                "Turtle_4": "graph_weighted_manhattan_distance_bid",
            },
            {
                "Turtle_1": "graph_weighted_manhattan_distance_bid",
                "Turtle_2": "graph_weighted_manhattan_distance_bid",
                "Turtle_3": "graph_weighted_manhattan_distance_bid",
                "Turtle_4": "graph_weighted_manhattan_distance_bid",
            }
        ]

    def gen_scenarios_config(self,
                            gen_type: int = SOLO_GEN,
                            save_to_file: bool = True):
        # -> Generate configurations parameters
        # ----- Environment
        env_connectivity_configs = np.linspace(self.env_connectivity_range[0], self.env_connectivity_range[1], self.scenarios_count, dtype=float)
        self.rng.shuffle(env_connectivity_configs)
        
        env_size_configs = np.linspace(self.env_size_range[0], self.env_size_range[1], self.scenarios_count, dtype=int)
        self.rng.shuffle(env_size_configs)
        
        # ----- Tasks
        goto_tasks_count_configs = np.linspace(self.goto_tasks_count_range[0], self.goto_tasks_count_range[1], self.scenarios_count, dtype=int)
        self.rng.shuffle(goto_tasks_count_configs)
        
        tasks_types_ratios_configs = [
            np.linspace(self.no_action_tasks_fraction_range[0], self.no_action_tasks_fraction_range[1], self.scenarios_count, dtype=int),
            np.linspace(self.action1_tasks_fraction_range[0], self.action1_tasks_fraction_range[1], self.scenarios_count, dtype=int),
            np.linspace(self.action2_tasks_fraction_range[0], self.action2_tasks_fraction_range[1], self.scenarios_count, dtype=int)
        ]
        
        for i in range(len(tasks_types_ratios_configs)):
            self.rng.shuffle(tasks_types_ratios_configs[i])
        
        initial_tasks_announcement_configs = np.linspace(self.initial_tasks_announcement_fraction_range[0], self.initial_tasks_announcement_fraction_range[1], self.scenarios_count, dtype=int)
        self.rng.shuffle(initial_tasks_announcement_configs)
        
        release_max_epoch_configs = np.linspace(self.release_max_epoch_range[0], self.release_max_epoch_range[1], self.scenarios_count, dtype=int)
        self.rng.shuffle(release_max_epoch_configs)
        
        # ----- Agents
        fleets_skillsets_configs = np.linspace(0, len(self.fleets_skillsets)-1, self.scenarios_count, dtype=int)
        self.rng.shuffle(fleets_skillsets_configs)
        
        fleets_bids_mechanisms_configs = np.linspace(0, len(self.fleet_bids_mechanisms)-1, self.scenarios_count, dtype=int)
        self.rng.shuffle(fleets_bids_mechanisms_configs)

        # -> Generate scenario files
        for i in range(self.scenarios_count):
            self.gen_scenario_config(
                scenario_id=f"Scenario_{i}",
                env_connectivity=env_connectivity_configs[i],
                env_size=env_size_configs[i],
                goto_tasks_count=goto_tasks_count_configs[i],
                tasks_types_ratios=[
                    tasks_types_ratios_configs[0][i],
                    tasks_types_ratios_configs[1][i],
                    tasks_types_ratios_configs[2][i]
                ],
                initial_tasks_announcement=initial_tasks_announcement_configs[i],
                release_max_epoch=release_max_epoch_configs[i],
                fleet_skillsets=self.fleets_skillsets[fleets_skillsets_configs[i]],
                fleet_bids_mechanisms=self.fleet_bids_mechanisms[fleets_bids_mechanisms_configs[i]],
                recompute_bids_on_state_change=self.recompute_bids_on_state_change,
                with_interceding=self.with_interceding,
                gen_type=gen_type,
                save_to_file=save_to_file
            )

    def latin_hypercube_sampling(self, n: int, k: int) -> np.ndarray:
        """
        Generate a Latin Hypercube Sample of size n and dimension k.
        """
        # -> Generate the intervals
        intervals = np.linspace(0, 1, n + 1)

        # -> Generate the random samples
        samples = np.zeros((n, k))

        for i in range(k):
            samples[:, i] = self.rng.uniform(intervals[:-1], intervals[1:], n)

        # -> Shuffle the samples
        for i in range(k):
            self.rng.shuffle(samples[:, i])

        return samples

    def gen_scenario_config(self,
                            scenario_id: str,
                            env_size: int,
                            env_connectivity: float,
                            goto_tasks_count: int,
                            tasks_types_ratios: List[float],
                            initial_tasks_announcement: int,
                            release_max_epoch: int,
                            fleet_skillsets: dict,
                            fleet_bids_mechanisms: dict,
                            recompute_bids_on_state_change: bool = True,
                            with_interceding: bool = False,
                            gen_type: int = SOLO_GEN,
                            save_to_file: bool = False,
                            ) -> dict:

        # ---------------- Generate GOTOs
        def generate_task(action_at_loc: str, target_agent_id: str, epoch: int) -> dict:
            # -> Generate task
            task = {
                "epoch": epoch,
                "creator_id": target_agent_id,
                "id": str(self.rng.randint(0, 1000000)),
                "type": "GOTO",
                "meta_action": "pending",
                "priority": 0.0,
                "affiliations": "base",
                "instructions": {
                    "x": self.rng.randint(0, env_size-1),
                    "y": self.rng.randint(0, env_size-1),
                    "ACTION_AT_LOC": action_at_loc
                }
            }

            # -> Compute task euclidian distance from start
            task["euclidian_distance_from_start"] = np.sqrt(
                task["instructions"]["x"]**2 +
                task["instructions"]["y"]**2
            )

            return task

        # -> Generate GOTO tasks
        goto_tasks = []

        actions_at_loc = random.choices(self.skills, tasks_types_ratios, k=10)

        for i in range(goto_tasks_count):
            goto_tasks.append(
                generate_task(
                    action_at_loc=actions_at_loc[i],
                    target_agent_id=self.rng.choice(self.agent_lst),
                    epoch=0
                )
            )

        # > Sort list by euclidian distance from start
        goto_tasks = sorted(goto_tasks, key=lambda x: x["euclidian_distance_from_start"])

        # > Update tasks according to schedule
        # > Set first len(initial_tasks_announcement) tasks to epoch 0
        for i in range(initial_tasks_announcement):
            goto_tasks[i]["epoch"] = 0

        # > Set the rest of the tasks to random epochs in sequence
        avg_step_size = release_max_epoch // (goto_tasks_count - initial_tasks_announcement + 1)

        step = 0

        for task in goto_tasks[initial_tasks_announcement:]:
            step += avg_step_size + self.rng.randint(0, 1 + avg_step_size // 2)

            if step > release_max_epoch:
                step = release_max_epoch

            task["epoch"] = step

        # ---------------- Save config
        scenario_config = {
            "scenario_id": scenario_id,
            "scenario_type": "gridworld",
            "env_connectivity": env_connectivity,
            "env_size": env_size,
            "goto_tasks_count": goto_tasks_count,
            "tasks_types_ratios": tasks_types_ratios,
            "initial_tasks_announcement": initial_tasks_announcement,
            "release_max_epoch": release_max_epoch,
            "fleet_skillsets": fleet_skillsets,
            "fleet_bids_mechanisms": fleet_bids_mechanisms,
            "goto_tasks": goto_tasks,
        }

        scenarios = {}

        if gen_type == SOLO_GEN:
            # -> Generate a single scenario with the given parameters
            scenario_config["recompute_bids_on_state_change"] = recompute_bids_on_state_change
            scenario_config["with_interceding"] = with_interceding

            scenarios[scenario_config["scenario_id"]] = scenario_config
        else:
            # -> Generate a scenario for each combination of parameters
            # > No intercession, no recompute
            scenario_config_no_intercession_no_recompute = deepcopy(scenario_config)
            scenario_config_no_intercession_no_recompute["recompute_bids_on_state_change"] = False
            scenario_config_no_intercession_no_recompute["with_interceding"] = False

            scenarios[f'{scenario_config["scenario_id"]}_no_intercession_no_recompute'] = scenario_config_no_intercession_no_recompute

            # > No intercession, recompute
            scenario_config_no_intercession_recompute = deepcopy(scenario_config)
            scenario_config_no_intercession_recompute["recompute_bids_on_state_change"] = True
            scenario_config_no_intercession_recompute["with_interceding"] = False

            scenarios[f'{scenario_config["scenario_id"]}_no_intercession_recompute'] = scenario_config_no_intercession_recompute

            # > Intercession, no recompute
            scenario_config_intercession_no_recompute = deepcopy(scenario_config)
            scenario_config_intercession_no_recompute["recompute_bids_on_state_change"] = False
            scenario_config_intercession_no_recompute["with_interceding"] = True

            scenarios[f'{scenario_config["scenario_id"]}_intercession_no_recompute'] = scenario_config_intercession_no_recompute

            # > Intercession, recompute
            scenario_config_intercession_recompute = deepcopy(scenario_config)
            scenario_config_intercession_recompute["recompute_bids_on_state_change"] = True
            scenario_config_intercession_recompute["with_interceding"] = True

            scenarios[f'{scenario_config["scenario_id"]}_intercession_recompute'] = scenario_config_intercession_recompute

        if save_to_file:
            for scenario_id, scenario in scenarios.items():
                # -> Dump config to file as json
                with open(f"/home/vguillet/ros2_ws/src/rlb_simple_sim/rlb_simple_sim/Configs/{scenario_id}.json", "w") as f:
                    f.write(dumps(scenario, default=convert_numpy_int64, indent=4))

        return scenarios


def convert_numpy_int64(o):
    if isinstance(o, np.int64):
        return int(o)
    raise TypeError


if __name__ == "__main__":
    sg = ScenariosGenerator()
    sg.gen_scenarios_config(
        gen_type=PAIRS_GEN,
        save_to_file=True
    )