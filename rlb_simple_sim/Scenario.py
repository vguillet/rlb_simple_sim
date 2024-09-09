
from json import dumps, loads
from pprint import pprint, pformat
import pathlib

from .ScenariosGenerator import ScenariosGenerator


class Scenario:
    def __init__(self,
                 scenario_id: str,
                 env_size: int = 10,
                 env_connectivity: float = .65,
                 goto_tasks_count: int = 10,
                 tasks_types_ratios: list = [1, 3, 3],    # No task, Action 1, Action 2
                 initial_tasks_announcement: int = 2,
                 release_max_epoch: int = 200,
                 fleet_skillsets: dict = {"Turtle_1": ["GOTO", "ACTION_1"],
                                         "Turtle_2": ["GOTO", "ACTION_1"],
                                         "Turtle_3": ["GOTO", "ACTION_2"],
                                         "Turtle_4": ["GOTO", "ACTION_2"]},
                 fleet_bids_mechanisms: dict = {"Turtle_1": "anticipated_action_task_interceding_agent",
                                                "Turtle_2": "graph_weighted_manhattan_distance_bid",
                                                "Turtle_3": "graph_weighted_manhattan_distance_bid",
                                                "Turtle_4": "graph_weighted_manhattan_distance_bid"},
                 recompute_bids_on_state_change: bool = True,
                 with_interceding: bool = False,
                 load_only: bool = False,
                 logger=None
                 ):

        self.scenario_id = scenario_id

        # -> If path corresponds to a scenario file, load it (check path)
        if pathlib.Path(f"/home/vguillet/ros2_ws/src/rlb_simple_sim/rlb_simple_sim/Configs/{scenario_id}").exists():
            if logger is not None:
                logger.info(f"Found scenario config: {scenario_id}")

            with open(f"/home/vguillet/ros2_ws/src/rlb_simple_sim/rlb_simple_sim/Configs/{scenario_id}",
                      "r") as f:
                scenario_config = loads(f.read())

                self.load_gridworld_scenario_config(scenario_config)

        elif not load_only:
            scenario_config = ScenariosGenerator().gen_scenario_config(
                scenario_id=scenario_id,
                env_size=env_size,
                env_connectivity=env_connectivity,
                goto_tasks_count=goto_tasks_count,
                tasks_types_ratios=tasks_types_ratios,
                initial_tasks_announcement=initial_tasks_announcement,
                release_max_epoch=release_max_epoch,
                fleet_skillsets=fleet_skillsets,
                fleet_bids_mechanisms=fleet_bids_mechanisms,
                recompute_bids_on_state_change=recompute_bids_on_state_change,
                with_interceding=with_interceding,
                save_to_file=False
            )[scenario_id]

            self.load_gridworld_scenario_config(scenario_config)

        else:
            raise FileNotFoundError(f"Scenario config file not found: {scenario_id}")

    def load_gridworld_scenario_config(self, scenario_config):
        self.seed = scenario_config["seed"]
        self.env_size = scenario_config["env_size"]
        self.env_connectivity = scenario_config["env_connectivity"]
        self.goto_tasks_count = scenario_config["goto_tasks_count"]
        self.tasks_types_ratios = scenario_config["tasks_types_ratios"]
        self.initial_tasks_announcement = scenario_config["initial_tasks_announcement"]
        self.release_max_epoch = scenario_config["release_max_epoch"]

        self.agent_lst = scenario_config["agent_lst"]
        self.visibility_ranges = scenario_config["visibility_ranges"]

        self.fleet_skillsets = scenario_config["fleet_skillsets"]
        self.fleet_bids_mechanisms = scenario_config["fleet_bids_mechanisms"]
        self.goto_tasks = scenario_config["goto_tasks"]

        self.recompute_bids_on_state_change = scenario_config["recompute_bids_on_state_change"]
        self.with_interceding = scenario_config["with_interceding"]
        self.intercession_targets = scenario_config["intercession_targets"]
        self.interventionism = scenario_config["interventionism"]

        # -> Generate agent dictionary
        self.agent_dict = {}

        for agent in self.agent_lst:
            self.agent_dict[agent] = {
                "skillset": self.fleet_skillsets[agent],
                "visibility range": self.visibility_ranges[agent],
                "bid mechanism": self.fleet_bids_mechanisms[agent]
            }


if __name__ == "__main__":
    scenario = Scenario("Scenario_Scenario_0")

    print(scenario.scenario_id)
    print(scenario.scenario_type)
    print(scenario.env_connectivity)
    print(scenario.env_size)
    print(scenario.goto_tasks_count)
    print(scenario.tasks_types_ratios)
    print(scenario.initial_tasks_announcement)
    print(scenario.release_max_epoch)
    print(scenario.fleet_skillsets)
    print(scenario.fleet_bids_mechanisms)
    print(scenario.goto_tasks)