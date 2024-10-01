
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
import sys
from abc import abstractmethod
from typing import List, Optional
from datetime import datetime, timedelta
from json import dumps, loads
from pprint import pprint, pformat
import warnings
from copy import deepcopy
from math import ceil
import random
import numpy as np

# Libs

# ROS2 Imports

# Local Imports
try:
    from graph_env.graph_generator import generate_benchmark_layout

except ModuleNotFoundError:
    from graph_env.graph_env.graph_generator import generate_benchmark_layout

######################################################################################################


SOLO_GEN = 0
PAIRS_GEN = 1

##################################################################################################################


class ScenariosGenerator:
    def __init__(self, scenarios_count=1):
        self.seed = 12
        self.rng = np.random.RandomState(seed=self.seed)

        self.scenarios_count = scenarios_count

        # ----- Allocation protocol
        self.recompute_bids_on_state_change = True
        self.with_interceding = False
        self.intercession_target = "full"

        # ----- Environment
        self.environment_type = "MAPF"              # MAPF, grid, random, star

        # -> MAPF
        if self.environment_type == "MAPF":
            self.environment_path = "Paris_0_256.map"    # Only relevant for MAPF
            self.graph, self.pos = generate_benchmark_layout(map_file=self.environment_path)

        # -> grid, random, star
        self.env_connectivity_range = [.85, .85]
        self.env_size_range = [19, 19]
        # self.env_size_range = [9, 9]

        # ----- Tasks
        self.goto_tasks_count_range = [50, 50]

        self.tasks_count_config = [
            [0, 25, 25],
            [10, 20, 20],
            [10, 35, 5],
            [10, 39, 1],
        ]

        self.no_action_tasks_fraction_range = [0, 1]
        self.action1_tasks_fraction_range = [1, 2]
        self.action2_tasks_fraction_range = [1, 2]

        self.initial_tasks_announcement_fraction_range = [0.1, 0.1]
        self.release_max_epoch_range = [100, 100]

        # ----- Agents
        self.agent_lst = ["Turtle_1", "Turtle_2", "Turtle_3", "Turtle_4"]
        #self.skills = ["GOTO", "ACTION_1", "ACTION_2"]

        self.intercession_targets = {
            "no": [],
            "partial": ["ACTION_1", "NO_TASK"],
            "full": ["ACTION_1", "ACTION_2", "NO_TASK"]
        }

        self.intercession_rates = np.linspace(0, 1, 5, dtype=float)

        self.visibility_ranges = [
            {
            "Turtle_1": 1,
            "Turtle_2": 1,
            "Turtle_3": 1,
            "Turtle_4": 1
            }
        ]

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
            # {
            #     "Turtle_1": ["GOTO", "ACTION_1"],
            #     "Turtle_2": ["GOTO"],
            #     "Turtle_3": ["GOTO", "ACTION_2"],
            #     "Turtle_4": ["GOTO", "ACTION_2"],
            # },
            # {
            #     "Turtle_1": ["GOTO", "ACTION_1"],
            #     "Turtle_2": ["GOTO"],
            #     "Turtle_3": ["GOTO"],
            #     "Turtle_4": ["GOTO", "ACTION_2"],
            # }
        ]

        INTERCEDING = "anticipated_action_task_interceding_agent"
        NO_INTERCEDING = "graph_weighted_manhattan_distance_bid"

        self.fleet_bids_mechanisms = {
            "interceding": {
                "Turtle_1": INTERCEDING,
                "Turtle_2": NO_INTERCEDING,
                "Turtle_3": NO_INTERCEDING,
                "Turtle_4": NO_INTERCEDING,
            },
            "no_interceding": {
                "Turtle_1": NO_INTERCEDING,
                "Turtle_2": NO_INTERCEDING,
                "Turtle_3": NO_INTERCEDING,
                "Turtle_4": NO_INTERCEDING,
            }
        }

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
        
        initial_tasks_announcement_configs = np.linspace(self.initial_tasks_announcement_fraction_range[0], self.initial_tasks_announcement_fraction_range[1], self.scenarios_count)
        self.rng.shuffle(initial_tasks_announcement_configs)
        
        release_max_epoch_configs = np.linspace(self.release_max_epoch_range[0], self.release_max_epoch_range[1], self.scenarios_count, dtype=int)
        self.rng.shuffle(release_max_epoch_configs)
        
        # ----- Agents
        fleets_visibility_ranges_configs = np.linspace(0, len(self.visibility_ranges)-1, self.scenarios_count, dtype=int)
        self.rng.shuffle(fleets_visibility_ranges_configs)

        fleets_skillsets_configs = np.linspace(0, len(self.fleets_skillsets)-1, self.scenarios_count, dtype=int)
        self.rng.shuffle(fleets_skillsets_configs)

        # -> Generate scenario files
        scenarios = {}

        for i in range(self.scenarios_count):
            new_scenarios = self.gen_scenario_config(
                scenario_id=f"Scenario_{i}",
                env_connectivity=env_connectivity_configs[i],
                env_size=env_size_configs[i],
                goto_tasks_count=goto_tasks_count_configs[i],
                no_tasks_count=tasks_types_ratios_configs[0][i],
                action_1_tasks_count=tasks_types_ratios_configs[1][i],
                action_2_tasks_count=tasks_types_ratios_configs[2][i],
                initial_tasks_announcement=initial_tasks_announcement_configs[i],
                release_max_epoch=release_max_epoch_configs[i],
                fleets_visibility_ranges = self.visibility_ranges[fleets_visibility_ranges_configs[i]],
                fleet_skillsets=self.fleets_skillsets[fleets_skillsets_configs[i]],
                gen_type=gen_type,
                save_to_file=save_to_file
            )

            scenarios = scenarios | new_scenarios

        return scenarios

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
                            no_tasks_count: int,
                            action_1_tasks_count: int,
                            action_2_tasks_count: int,
                            initial_tasks_announcement: int,
                            release_max_epoch: int,
                            fleets_visibility_ranges: dict,
                            fleet_skillsets: dict,
                            gen_type: int = SOLO_GEN,
                            save_to_file: bool = False,
                            ) -> dict:

        # ---------------- Save config
        # Parameters bellow are shared across configs
        scenario_config_base = {
            "seed": self.seed,
            "scenario_id": scenario_id,
            "environment_type": self.environment_type,
            "environment_path": self.environment_path,
            "env_connectivity": env_connectivity,
            "env_size": env_size,

            "goto_tasks_count": goto_tasks_count,
            "initial_tasks_announcement": initial_tasks_announcement,
            "release_max_epoch": release_max_epoch,

            "agent_lst": self.agent_lst,
            "visibility_ranges": fleets_visibility_ranges, # Fixed for now
        }

        scenarios = {}

        if gen_type == SOLO_GEN:
            # -> Generate a single scenario with the given parameters
            scenario_config_base["fleet_skillsets"] = fleet_skillsets
            scenario_config_base["recompute_bids_on_state_change"] = self.recompute_bids_on_state_change
            scenario_config_base["intercession_targets"] = self.intercession_targets[self.intercession_target]
            scenario_config_base["interventionism"] = 0.0  # TODO: Correct ot make dynamic
            scenario_config_base["with_interceding"] = self.with_interceding

            if self.with_interceding:
                scenario_config_base["fleet_bids_mechanisms"] = self.fleet_bids_mechanisms["interceding"]
            else:
                scenario_config_base["fleet_bids_mechanisms"] = self.fleet_bids_mechanisms["no_interceding"]

            # -> Generate tasks
            scenario_config_base["tasks_types_ratios"] = [no_tasks_count, action_1_tasks_count, action_2_tasks_count]

            scenario_config_base["goto_tasks"] = self.generate_tasks(
                env_size=env_size,
                goto_tasks_count=goto_tasks_count,
                no_tasks_count=no_tasks_count,
                action_1_tasks_count=action_1_tasks_count,
                action_2_tasks_count=action_2_tasks_count,
                initial_tasks_announcement=initial_tasks_announcement,
                release_max_epoch=release_max_epoch
            )

            scenarios[scenario_config_base["scenario_id"]] = scenario_config_base

        else:
            # --------------------------------- Loop for tasks types ratios
            for k in range(len(self.tasks_count_config)):
                scenario_config_tasks_types = deepcopy(scenario_config_base)

                scenario_config_tasks_types["tasks_types_ratios"] = self.tasks_count_config[k]

                # -> Add fleet skillset
                if self.tasks_count_config[k] == [10, 39, 1]:
                    scenario_config_tasks_types["fleet_skillsets"] = self.fleets_skillsets[1]
                else:
                    scenario_config_tasks_types["fleet_skillsets"] = self.fleets_skillsets[0]

                # -> Generate tasks
                no_tasks_count = self.tasks_count_config[k][0]
                action_1_tasks_count = self.tasks_count_config[k][1]
                action_2_tasks_count = self.tasks_count_config[k][2]

                scenario_config_tasks_types["goto_tasks"] = self.generate_tasks(
                    env_size=env_size,
                    goto_tasks_count=goto_tasks_count,
                    no_tasks_count=no_tasks_count,
                    action_1_tasks_count=action_1_tasks_count,
                    action_2_tasks_count=action_2_tasks_count,
                    initial_tasks_announcement=initial_tasks_announcement,
                    release_max_epoch=release_max_epoch,
                    interventionism=0.0
                )

                scenario_config_tasks_types["no_tasks_count"] = no_tasks_count
                scenario_config_tasks_types["action_1_tasks_count"] = action_1_tasks_count
                scenario_config_tasks_types["action_2_tasks_count"] = action_2_tasks_count
                scenario_config_tasks_types["tasks_types_ratios"] = self.tasks_count_config[k]

                # --------------------------------- Loop for recompute on change
                for i in range(2):
                    scenario_config_recompute = deepcopy(scenario_config_tasks_types)

                    scenario_config_recompute["recompute_bids_on_state_change"] = bool(i)

                    # --------------------------------- Loop for intercession targets
                    for j in self.intercession_targets.keys():
                        scenario_config_intercession_targets = deepcopy(scenario_config_recompute)

                        if j == "no":
                            scenario_config_intercession_targets["with_interceding"] = False
                            scenario_config_intercession_targets["fleet_bids_mechanisms"] = self.fleet_bids_mechanisms[
                                "no_interceding"]

                        else:
                            scenario_config_intercession_targets["with_interceding"] = True
                            scenario_config_intercession_targets["fleet_bids_mechanisms"] = self.fleet_bids_mechanisms[
                                "interceding"]

                        scenario_config_intercession_targets["intercession_targets"] = self.intercession_targets[j]

                        # --------------------------------- Loop for interventionism
                        for interventionism in self.intercession_rates:
                            scenario_config_interventionism = deepcopy(scenario_config_intercession_targets)

                            scenario_config_interventionism["interventionism"] = interventionism

                            # -> Set interventionism flag for the tasks
                            for task in scenario_config_interventionism["goto_tasks"]:
                                task["intervention"] = self.rng.choice([1, 0], p=[interventionism, 1 - interventionism])

                            # --------------------------------- Save scenario
                            scenario_config_final = scenario_config_interventionism

                            # -> Construct scenario_ref
                            scenario_config_final["scenario_ref"] = scenario_config_base['scenario_id']
                            scenario_config_final["scenario_ref"] += f"_{j}_intercession"
                            scenario_config_final["scenario_ref"] += "_no" if not bool(i) else ""
                            scenario_config_final["scenario_ref"] += "_recompute"
                            scenario_config_final["scenario_ref"] += f"_{k}"
                            scenario_config_final["scenario_ref"] += f"_interventionism_{int(100 * interventionism)}"
                            scenarios[f'{scenario_config_final["scenario_ref"]}'] = scenario_config_final

            if save_to_file:
                for scenario_id, scenario in scenarios.items():
                    # -> Dump config to file as json
                    with open(f"/home/vguillet/ros2_ws/src/rlb_simple_sim/rlb_simple_sim/Configs/{scenario_id}.json",
                              "w") as f:
                        f.write(dumps(scenario, default=convert_numpy_int64, indent=4))

        return scenarios

    def generate_tasks(self,
                       env_size: int,
                       goto_tasks_count: int,
                       no_tasks_count: int,
                       action_1_tasks_count: int,
                       action_2_tasks_count: int,
                       initial_tasks_announcement: int,
                       release_max_epoch: int,
                       interventionism: float = 1.0,
                       ):
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
                "intervention": self.rng.choice([1, 0], p=[interventionism, 1-interventionism]),
                "affiliations": "base",
                "instructions": {
                    "x": None,
                    "y": None,
                    "ACTION_AT_LOC": action_at_loc
                }
            }

            if self.environment_type == "MAPF":
                # -> Select a pos from the environment graph pos by generating a random index
                pos_index = self.rng.randint(0, len(self.pos.items()) - 1)
                pos = list(self.pos.values())[pos_index]

            else:
                pos = (self.rng.randint(0, env_size-1), self.rng.randint(0, env_size-1))

            task["instructions"]["x"] = pos[0]
            task["instructions"]["y"] = pos[1]

            # -> Compute task euclidian distance from start
            task["euclidian_distance_from_start"] = np.sqrt(
                task["instructions"]["x"]**2 +
                task["instructions"]["y"]**2
            )

            return task

        # -> Generate GOTO tasks
        goto_tasks = []

        # -> List all skills except goto
        actions_at_loc = ["NO_TASK"] * no_tasks_count + ["ACTION_1"] * action_1_tasks_count + ["ACTION_2"] * action_2_tasks_count

        # -> Shuffle actions
        self.rng.shuffle(actions_at_loc)

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
        initial_tasks_announcement = int(goto_tasks_count * initial_tasks_announcement)
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

        for i in range(len(goto_tasks)):
            goto_tasks[i]["id"] = f"{i}"

        return goto_tasks


def convert_numpy_int64(o):
    if isinstance(o, np.int64):
        return int(o)
    raise TypeError


if __name__ == "__main__":
    datasets_count = 1

    sg = ScenariosGenerator(datasets_count)
    scenarios = sg.gen_scenarios_config(
        gen_type=PAIRS_GEN,
        save_to_file=True
    )

    print(f"Generated {len(scenarios)} scenarios")
    # Define time delta as 1:30
    single_run_time = timedelta(seconds=1*60+30)

    print(f"Estimated time for {datasets_count} datasets: {single_run_time * datasets_count * 6*5}")