
from datetime import datetime
import os
import json
from orchestra_config.sim_config import *
from copy import deepcopy


class Results:
    def __init__(self, fleet, scenario):
        self.results = {
            # -> Scenario
            "seed": scenario.seed,
            "env_size": scenario.env_size,
            "env_connectivity": scenario.env_connectivity,
            "goto_tasks_count": scenario.goto_tasks_count,
            "tasks_types_ratios": scenario.tasks_types_ratios,
            "initial_tasks_announcement": scenario.initial_tasks_announcement,
            "release_max_epoch": scenario.release_max_epoch,

            "agent_lst": scenario.agent_lst,
            "fleet_skillsets": scenario.fleet_skillsets,
            "fleet_bids_mechanisms": scenario.fleet_bids_mechanisms,
            "goto_tasks": scenario.goto_tasks,

            "recompute_bids_on_state_change": scenario.recompute_bids_on_state_change,
            "with_interceding": scenario.with_interceding,
            "intercession_targets": scenario.intercession_targets,

            # -> Non-serializable
            "fleet": fleet,
            "sim_start_time": None,                 # datetime
            "sim_end_time": None,                   # datetime
            "sim_duration": None,                   # timedelta
            "average_time_per_epoch": None,         # timedelta

            "last_epoch": 0,                        # int
            "pose_history": {},                     # {agent_id: [(x, y), (x, y), ...]}
            "move_history": {},                     # {agent_id: [(x, y), (x, y), ...]}
            "task_history": {},                     # {task_id: [(x, y), (x, y), ...]}
            "allocation": {},                       # {task_id: {agent_id: ..., epoch: ...}}
            "total_fleet_msgs_count": 0,            # int
            "total_goal_msgs_count": 0,             # int

            # -> Config
            "sim_id": scenario.scenario_id,
            "goto_task_count": scenario.goto_tasks_count,
            "no_task_task_count": 0,
            "action_1_task_count": 0,
            "action_2_task_count": 0,
            "total_task_count": 0,
            "release_spread": deepcopy(release_spread),
            "release_ration": deepcopy(release_ration),
        }

        # -> Gen scenario stats
        for task in scenario.goto_tasks:
            if task["instructions"]["ACTION_AT_LOC"] == "NO_TASK":
                self.results["no_task_task_count"] += 1
            elif task["instructions"]["ACTION_AT_LOC"] == "ACTION_1":
                self.results["action_1_task_count"] += 1
            elif task["instructions"]["ACTION_AT_LOC"] == "ACTION_2":
                self.results["action_2_task_count"] += 1

        self.results["total_task_count"] = self.results["goto_task_count"] + self.results["action_1_task_count"] + self.results["action_2_task_count"]

    def __getitem__(self, item):
        return self.results[item]

    def __setitem__(self, key, value):
        self.results[key] = value

    def dump_results(self):
        """
        Dump results to different files
        """

        # -> Generate results directory
        # > Generate file path
        results_dir = f"{os.getcwd()}/log/results_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}_{self.results['sim_id']}"

        # > Create directory
        os.mkdir(results_dir)

        # -> Generate run recap
        # > Generate file path
        run_recap_file_path = results_dir + "/run_recap.txt"

        # > Generate recap
        run_recap = self.generate_run_recap()

        # > Remove all non-serializable objects
        unserialsed_keys = [
            "fleet",
            "sim_start_time",
            "sim_end_time",
            "sim_duration",
            "average_time_per_epoch"
        ]

        for key in unserialsed_keys:
            del self.results[key]

        # > Dump run recap to file
        with open(run_recap_file_path, 'w') as file:
            file.write(run_recap)
            print(f"- Run recap dumped to {run_recap_file_path}")

        # -> Run results
        # > Generate file path
        run_results_file_path = results_dir + "/run_results.json"

        # > Dump results to file
        with open(run_results_file_path, 'w') as file:    # Open the file in binary write mode
            # Dump the dictionary to the file
            json.dump(self.results, file, indent=4, sort_keys=True)
            print(f"- Run results dumped to {run_results_file_path}")

    def generate_run_recap(self) -> str:
        """
        Generate a recap of the run, logs all computed metrics to results dict and returns a string recap of the run

        :return: str - run_recap
        """
        run_recap = "\n"

        run_recap += "============================ Run Recap ============================\n"

        # -> Sim id
        run_recap += f">> Sim id: {self.results['sim_id']}\n"

        # -> Sim mode
        run_recap += f">> Recompute: {'Yes' if self.results['recompute_bids_on_state_change'] else 'No'}\n"

        # -> With interceding
        run_recap += f">> With intercession: {'Yes' if self.results['with_interceding'] else 'No'}\n"

        # ---------------------------------------------------------------------
        run_recap += "\n------ Sim Stats ------\n"
        # -> Sim duration
        self.results['sim_duration'] = self.results['sim_end_time'] - self.results['sim_start_time']
        run_recap += f"Sim duration: {self.results['sim_duration']}\n"

        # -> Average time per epoch
        self.results['average_time_per_epoch'] = self.results['sim_duration'] / (self.results['last_epoch']+1)
        run_recap += f"Average time per epoch: {self.results['average_time_per_epoch']}\n"

        # -> Task count
        run_recap += f"Allocations count completed: {len(self.results['allocation'])}\n"

        # -> Completion epoch
        self.results['last_epoch'] = 0

        for task_id in self.results['allocation']:
            if self.results['allocation'][task_id]['epoch'] > self.results['last_epoch']:
                self.results['last_epoch'] = self.results['allocation'][task_id]['epoch']

        run_recap += f"Last task completion epoch: {self.results['last_epoch']}\n"

        run_recap += "\n------ Tardiness ------\n"

        self.results["cumulated_goto_tardiness"] = 0
        self.results["cumulated_action_tardiness"] = 0

        # -> Remove all tasks with no termination epoch
        self.results["task_history"] = {task_id: task for task_id, task in self.results["task_history"].items() if task["termination_epoch"] != None}

        for task in self.results["task_history"].values():
            task["tardiness"] = task["termination_epoch"] - task["release_epoch"] - 1

            if task["type"] == "GOTO":
                self.results["cumulated_goto_tardiness"] += task["tardiness"]
            else:
                self.results["cumulated_action_tardiness"] += task["tardiness"]

        self.results["cumulated_total_tardiness"] = self.results["cumulated_goto_tardiness"] + self.results["cumulated_action_tardiness"]

        # -> Tardiness
        # > Cumulated
        run_recap += f"\nTotal cumulated tardiness: {self.results['cumulated_total_tardiness']}\n"
        run_recap += f"  > GOTO cumulated tardiness: {self.results['cumulated_goto_tardiness']}\n"
        run_recap += f"  > ACTION cumulated tardiness: {self.results['cumulated_action_tardiness']}\n"

        # > Avg
        self.results['avg_total_tardiness'] = self.results['cumulated_total_tardiness'] / len(self.results["task_history"])
        self.results['avg_goto_tardiness'] = self.results['cumulated_goto_tardiness'] / self.results['goto_task_count']
        self.results['avg_action_tardiness'] = self.results['cumulated_action_tardiness'] / (self.results['total_task_count'] - self.results['goto_task_count'])

        run_recap += f"\nTotal average tardiness: {round(self.results['avg_total_tardiness'], 2)}\n"
        run_recap += f"  > GOTO average tardiness: {round(self.results['avg_goto_tardiness'], 2)}\n"
        run_recap += f"  > ACTION average tardiness: {round(self.results['avg_action_tardiness'], 2)}\n"

        # > Max
        self.results['max_total_tardiness'] = max([task["tardiness"] for task in self.results["task_history"].values()])
        self.results['max_goto_tardiness'] = max([task["tardiness"] for task in self.results["task_history"].values() if task["type"] == "GOTO"])
        self.results['max_action_tardiness'] = max([task["tardiness"] for task in self.results["task_history"].values() if task["type"] != "GOTO"])

        run_recap += f"\nTotal max tardiness: {self.results['max_total_tardiness']}\n"
        run_recap += f"  > GOTO max tardiness: {self.results['max_goto_tardiness']}\n"
        run_recap += f"  > ACTION max tardiness: {self.results['max_action_tardiness']}\n"

        run_recap += "\n------ Displacement ------\n"

        # -> Cumulated move count
        self.results['cumulated_move_count'] = sum([len(self.results['move_history'][agent_id]) for agent_id in self.results['move_history']])
        run_recap += f"Cumulated move count: {self.results['cumulated_move_count']}\n"

        # -> Max move count
        self.results['max_move_count'] = max([len(self.results['move_history'][agent_id]) for agent_id in self.results['move_history']])
        self.results['max_move_count_agent_id'] = [agent_id for agent_id in self.results['move_history'] if len(self.results['move_history'][agent_id]) == self.results['max_move_count']][0]
        run_recap += f"Max move count: {self.results['max_move_count']} moves for agent {self.results['max_move_count_agent_id']}\n"

        # ---------------------------------------------------------------------
        run_recap += "\n------ Fleet messages ------\n"

        # -> Total fleet messages count
        run_recap += f"Total fleet messages count: {self.results['total_fleet_msgs_count']}\n"

        # ---------------------------------------------------------------------
        run_recap += "\n------ Allocation ------\n"

        # -> GOTO/ACTION combinations taken on correctly
        self.results['matched_allocation'] = 0
        self.results['miss_matched_allocation'] = 0
        self.results['task_pairs'] = 0

        for task_id in self.results['allocation']:
            for task_id_2 in self.results['allocation']:
                if task_id_2 == task_id + "'":
                    self.results['task_pairs'] += 1

                    if self.results['allocation'][task_id]['agent_id'] == self.results['allocation'][task_id_2]['agent_id']:
                        self.results['matched_allocation'] += 1
                    else:
                        self.results['miss_matched_allocation'] += 1

                    break

        self.results['matched_allocation_%'] = self.results['matched_allocation'] / self.results['task_pairs'] * 100
        self.results['miss_matched_allocation_%'] = self.results['miss_matched_allocation'] / self.results['task_pairs'] * 100

        run_recap += f"Matched allocation: {self.results['matched_allocation']}/{self.results['task_pairs']} ({round(self.results['matched_allocation_%'], 2)}%)\n"
        run_recap += f"Miss-matched allocation: {self.results['miss_matched_allocation']}/{self.results['task_pairs']} ({round(self.results['miss_matched_allocation_%'], 2)}%)\n"

        # ---------------------------------------------------------------------
        run_recap += "\n------ Scenario config ------\n"

        # -> Environment size
        run_recap += f"Environment size: {self.results['env_size']}\n"

        # -> Environment connectivity
        run_recap += f"Environment connectivity: {self.results['env_connectivity']}\n"

        # -> Task count
        run_recap += (
            f"\nTasks count: {self.results['goto_task_count'] + self.results['action_1_task_count'] + self.results['action_2_task_count']}"
            f"\n  GOTOs: {self.results['goto_task_count']}"
            f"\n    > {self.results['no_task_task_count']} NO_TASK"
            f"\n    > {self.results['action_1_task_count']} ACTION_1"
            f"\n    > {self.results['action_2_task_count']} ACTION_2"
            )

        # -> Initial tasks announcement
        run_recap += f"\nInitial tasks announcement: {self.results['initial_tasks_announcement']}\n"

        # -> Release max epoch
        run_recap += f"Release max epoch: {self.results['release_max_epoch']}\n"

        # -> Agent list
        run_recap += f"\nAgents: {len(self.results['agent_lst'])} agents\n"

        for agent in self.results['agent_lst']:
            run_recap += f"  > {agent}\n"
            run_recap += f"    - Skillset: {self.results['fleet_skillsets'][agent]}\n"
            run_recap += f"    - Bid function: {self.results['fleet_bids_mechanisms'][agent]}\n"

        # ---------------------------------------------------------------------

        run_recap += "\n==================================================================\n"

        return run_recap
