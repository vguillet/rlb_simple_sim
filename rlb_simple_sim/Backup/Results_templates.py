
from copy import deepcopy
import statistics

scenarios_def = {
        "[0, 25, 25]": [0, 25, 25],
        "[10, 20, 20]": [10, 20, 20],
        "[10, 35, 5]": [10, 35, 5],
        "[10, 39, 1]": [10, 39, 1]
    }

configs_def = {
    "R-F/I-N": {
        "recompute_bids_on_state_change": False,
        "with_interceding": False,
        "intercession_targets": []
    },
    "R-F/I-P": {
        "recompute_bids_on_state_change": False,
        "with_interceding": True,
        "intercession_targets": ["ACTION_1", "NO_TASK"]
    },
    "R-F/I-F": {
        "recompute_bids_on_state_change": False,
        "with_interceding": True,
        "intercession_targets": ["ACTION_1", "ACTION_2", "NO_TASK"]
    },

    "R-T/I-N": {
        "recompute_bids_on_state_change": True,
        "with_interceding": False,
        "intercession_targets": []
    },
    "R-T/I-P": {
        "recompute_bids_on_state_change": True,
        "with_interceding": True,
        "intercession_targets": ["ACTION_1", "NO_TASK"]
    },
    "R-T/I-F": {
        "recompute_bids_on_state_change": True,
        "with_interceding": True,
        "intercession_targets": ["ACTION_1", "ACTION_2", "NO_TASK"]
    }
}

stats_template = {
    "values": [], # -> List of all values
    "mean": 0.,
    "min": 0.,
    "max": 0.,
    "std": 0.
}

def compute_stats_in_template(value):
    # -> Compute all stats in stats template
    try:
        value["mean"] = sum(value["values"]) / len(value["values"])
        value["min"] = min(value["values"])
        value["max"] = max(value["values"])
        # Std in percentage of mean
        try:
            value["std"] = statistics.stdev(value["values"]) / value["mean"] * 100
        except:
            value["std"] = 0
        # value["std"] = statistics.stdev(value["values"])
    except ZeroDivisionError:
        pass

    return value

results_template = {
    "results": [],

    # -> Tardiness
    "cumulated_total_tardiness": deepcopy(stats_template),
    "avg_total_tardiness": deepcopy(stats_template),
    "max_total_tardiness": deepcopy(stats_template),

    "cumulated_goto_tardiness": deepcopy(stats_template),
    "avg_goto_tardiness": deepcopy(stats_template),
    "max_goto_tardiness": deepcopy(stats_template),

    "cumulated_action_tardiness": deepcopy(stats_template),
    "avg_action_tardiness": deepcopy(stats_template),
    "max_action_tardiness": deepcopy(stats_template),

    # -> Displacement
    "cumulated_move_count": deepcopy(stats_template),
    "max_move_count": deepcopy(stats_template),

    # -> Msg count
    "total_fleet_msgs_count": deepcopy(stats_template),

    # -> Allocation
    "matched_allocation": deepcopy(stats_template),
    "matched_allocation_%": deepcopy(stats_template),

    "miss_matched_allocation": deepcopy(stats_template),
    "miss_matched_allocation_%": deepcopy(stats_template),
}

results_processed = {
    "R-F/I-N": deepcopy(results_template),
    "R-F/I-P": deepcopy(results_template),
    "R-F/I-F": deepcopy(results_template),
    "R-T/I-N": deepcopy(results_template),
    "R-T/I-P": deepcopy(results_template),
    "R-T/I-F": deepcopy(results_template)
}

scenarios_results_processed = {
    "[0, 25, 25]": deepcopy(results_processed),
    "[10, 20, 20]": deepcopy(results_processed),
    "[10, 35, 5]": deepcopy(results_processed),
    "[10, 39, 1]": deepcopy(results_processed)
}

global_results = {
    "run_count": 0,
    "dataset_count": 0
}

# -> Generate table of configurations
settings = [
    "seed",
    "env_size",
    "env_connectivity",
    "goto_tasks_count",
    "tasks_types_ratios",
    "initial_tasks_announcement",
    "release_max_epoch",
    "agent_lst",
    "fleet_skillsets",
    "fleet_bids_mechanisms",
    "recompute_bids_on_state_change",
    "with_interceding",
    "intercession_targets",

    # -> Config
    "sim_id",
    "goto_task_count",
    "no_task_task_count",
    "action_1_task_count",
    "action_2_task_count",
    "total_task_count",
    "release_spread",
    "release_ration"
]