
import os
import json
import sys

from matplotlib import pyplot as plt
from copy import deepcopy
from pprint import pprint, pformat
from tabulate import tabulate
import statistics
import numpy as np

from Results_templates import *

results = []

base_path = "/home/vguillet/ros2_ws/log"

# -> Load all results recaps
for folder in os.listdir(base_path):
    print(f"> Checking results: {folder}")
    folder_split = folder.split("_")

    # -> Check if folder is a Scenario results folder
    if folder_split[0] == "results":
        with open(f"{base_path}/{folder}/run_results.json", "r") as f:
            result = json.load(f)
            result["run_id"] = folder
            results.append(result)

results.sort(key=lambda x: x["sim_id"])

print(f">> Loaded {len(results)} results")


# -> Generate statistics
def find_config(result):
    results_config = {
        "recompute_bids_on_state_change": result["recompute_bids_on_state_change"],
        "with_interceding": result["with_interceding"],
        "intercession_targets": result["intercession_targets"]
    }

    # -> Find config in configs_ref matching config
    scenario_ref = None
    config_ref = None

    for key, value in configs_def.items():
        if value == results_config:
            config_ref = key
            break

    # -> Find scenario in scenarios_ref matching scenario
    for key, value in scenarios_def.items():
        if value == result["tasks_types_ratios"]:
            scenario_ref = key
            break

    return scenario_ref, config_ref

# -> Sort results in by scenario_id
# config_table = []
#
# for result in results:
#     row = []
#
#     for setting in settings:
#         row.append(result[setting])
#
#     config_table.append(row)
#
# print(tabulate(config_table, headers=settings))

# ============================================================ Sort and label results
for result in results:
    # -> Add interventionism label (TODO: Remove, placeholder)
    result["interventionism"] = float(result["run_id"].split("_")[-1].split(".")[0])

    scenario, config = find_config(result)
    scenarios_results_processed[scenario][config]["results"].append(result)

    # -> Store values
    for metric, metric_stats in scenarios_results_processed[scenario][config].items():
        if metric == "results":
            continue

        metric_stats["values"].append(result[metric])

    # -> Update global results
    global_results["run_count"] += 1

global_results["dataset_count"] = global_results["run_count"] // (len(scenarios_results_processed) * (len(results_processed) - 3))
global_results["incomplete_dataset_length"] = global_results["run_count"] % (len(scenarios_results_processed) * (len(results_processed) - 3))

# ============================================================ Compute stats
for scenario_id, scenario in scenarios_results_processed.items():
    for config_id, config in scenario.items():
        for metric, value in config.items():
            if metric == "results":
                continue

            # -> Compute all stats in stats template
            scenarios_results_processed[scenario_id][config_id][metric] = compute_stats_in_template(value=value)

pprint(global_results)
print(f"Full dataset = {len(scenarios_results_processed) * (len(results_processed) - 3)} results")

# ============================================================ Clean up
# -> Remove incorrect results
filtered_results = deepcopy(scenarios_results_processed)

for scenario_id, scenario in scenarios_results_processed.items():
    for config in scenario.keys():
        # -> If last value of config is P, remove it
        # if config[-1] == "P":
        #     del filtered_results[scenario_id][config]

        if "R-T" in config:
            del filtered_results[scenario_id][config]

mapping = {
    "R-F/I-N": "CBBA",
    "R-F/I-P": "I-CBBA partial",
    "R-F/I-F": "I-CBBA full",
    "R-T/I-N": "CBBA+",
    "R-T/I-P": "I-CBBA+ partial",
    "R-T/I-F": "I-CBBA+ full",
}

filtered_results_2 = {}

for scenario_id, scenario in filtered_results.items():
    filtered_results_2[scenario_id] = {}
    for config_id, config in scenario.items():
        filtered_results_2[scenario_id][mapping[config_id]] = deepcopy(config)

scenarios_results_processed = filtered_results_2
# # -> Rename all scenarios keys N to F, and F to T
# filtered_results_2 = deepcopy(filtered_results)
#
# for scenario_id, scenario in filtered_results.items():
#     for config_id, config in scenario.items():
#         if config_id[-1] == "N":
#             scenarios_results_processed[scenario_id][config_id[:-1] + "F"] = deepcopy(config)
#         elif config_id[-1] == "F":
#             scenarios_results_processed[scenario_id][config_id[:-1] + "T"] = deepcopy(config)

# -> Generate one table per metric
# tables = {}
# for scenario, config in scenarios_results_processed.items():
#     for config, metrics in config.items():
#         for metric, value in metrics.items():
#             if metric == "results":
#                 continue
#
#             if metric not in tables:
#                 tables[metric] = []
#
#             tables[metric].append([scenario, config, value["mean"], value["min"], value["max"]])

# -> Generate tables
# for metric, table in tables.items():
#     print(f"\n\n>>>>>>>>>>> {metric}")
#     print(tabulate(table, headers=["Scenario", "Config", "Mean", "Min", "Max"]))

# -> Generate a table with all the metric mean values

# ============================================================ Format
table = []

include_metrics = [
    # "results",
    "cumulated_move_count",
    "cumulated_total_tardiness",
    # "avg_total_tardiness",
    # "max_total_tardiness",
    "cumulated_goto_tardiness",
    # "avg_goto_tardiness",
    # "max_goto_tardiness",
    "cumulated_action_tardiness",
    # "avg_action_tardiness",
    # "max_action_tardiness",
    # "max_move_count",
    # "matched_allocation",
    "matched_allocation_%",
    # "miss_matched_allocation",
    # "miss_matched_allocation_%",
    "total_fleet_msgs_count",
]

gen_table_results = False

if gen_table_results:
    for scenario, config in scenarios_results_processed.items():
        # -> Insert line
        # table.append(["--------"]*(len(include_metrics)+2))

        for config, metrics in config.items():
            # > Agents
            if scenario == "[10, 39, 1]":
                agents_ratio = "1, 3"
            else:
                agents_ratio = "2, 2"

            # > Scenario
            row = [str(scenario).replace("[", "").replace("]", ""), agents_ratio, config]

            for metric in include_metrics:
                if metric == "results":
                    row.append(len(metrics[metric]))
                    continue

                row.append(f"{round(metrics[metric]['mean'], 1)}")
                # row.append(f"{round(metrics[metric]['mean'], 1)} (+-{round(metrics[metric]['std'], 1)}%)")

            # -> Insert row
            table.append(row)

        # -> Insert line
        table.append(["--------"]*(len(include_metrics)+2))

            # for metric, value in metrics.items():
            #     if metric not in include_metrics:
            #         continue
            #
            #     if metric == "results":
            #         row.append(len(value))
            #         continue
            #
            #     row.append(round(value["mean"], 3))

    # > Format headers
    # include_metrics = [metric.replace("_", " ").capitalize() for metric in include_metrics]

    include_metrics_clean = [
        "Total move count",
        "Total tardiness",
        "Total goto tardiness",
        "Total action tardiness",
        "Matched allocation %",
        "Total msgs count"
    ]

    print(tabulate(table, headers=["a0, a1, a2", "s1, s2", "Config", *include_metrics_clean]))

    # metrics = list(scenarios_results_processed["[0, 25, 25]"]["R-F/I-N"].keys())
    #
    # filtered_metric = []
    #
    # for metric in metrics:
    #     if metric in include_metrics:
    #         filtered_metric.append(metric)
    #
    # print(tabulate(table, headers=["Scenario", "Config", *filtered_metric]))

    # tabulate.LATEX_ESCAPE_RULES = {}
    # print(tabulate(table, headers=["$|a_\emptyset$, $|a_1$, $|a_2$", "Algorithm", *include_metrics], tablefmt="latex"))

# ====================================================== Generate results for interventionism
# -> Generate one plot per metric per scenario
interventionism_results = {}

for metric in include_metrics:
    if metric not in interventionism_results.keys():
        interventionism_results[metric] = {}

    for scenario, scenario_data in scenarios_results_processed.items():
        if scenario not in interventionism_results[metric].keys():
            interventionism_results[metric][scenario] = {}
        # print(scenario)

        for intercession_rate, data in scenario_data.items():
            if intercession_rate not in interventionism_results[metric][scenario].keys():
                interventionism_results[metric][scenario][intercession_rate] = {}
            # print(intercession_rate)

            for result in data["results"]:
                if result["interventionism"] not in interventionism_results[metric][scenario][intercession_rate].keys():
                    interventionism_results[metric][scenario][intercession_rate][result["interventionism"]] = deepcopy(stats_template)

                # -> Store the results in the relevant location
                interventionism_results[metric][scenario][intercession_rate][result["interventionism"]]["values"].append(result[metric])

# pprint(interventionism_results)

# ---------- Compute averages
for metric, metric_data in interventionism_results.items():
    for scenario, scenario_data in metric_data.items():
        for intercession_rate, intercession_data in scenario_data.items():
            for interventionism, interventionism_data in intercession_data.items():
                interventionism_results[metric][scenario][intercession_rate][interventionism] = compute_stats_in_template(interventionism_data)

# pprint(interventionism_results)

# ---------- Generate plots
def plot_metric_against_interventionism(metric, scenario, intercession_rate):
    intercession_data = interventionism_results[metric][scenario][intercession_rate]

    # -> Generate plot
    x = list(intercession_data.keys())
    y = [intercession_data[inter]["mean"] for inter in x]

    # -> Add trendline
    z = np.polyfit(x, y, 1)
    p = np.poly1d(z)
    plt.plot(x, p(x), "r--")

    # -> Add error bars
    try:
        y_err = [intercession_data[inter]["std"] for inter in x]
        plt.errorbar(x, y, yerr=y_err, fmt='o')
    except:
        pass

    # -> Add labels
    plt.title(f"{metric} - {intercession_rate} ({len(intercession_data[0]['values'])} runs)")
    plt.xlabel("Interventionism")
    plt.ylabel("Mean")

    plt.show()


while True:
    # requested_metric = "cumulated_total_tardiness"
    # requested_scenario = "[0, 25, 25]"
    # requested_intercession_rate = "I-CBBA full"

    print("\n============================================== Plotter")

    print(f"Available metrics:")
    for i, metric in enumerate(include_metrics):
        print(f"{i}: {metric}")
    requested_metric = include_metrics[int(input("Metric: "))]

    print(f"\nAvailable scenarios:")
    scenarios = list(scenarios_def.keys())
    for i, scenario in enumerate(scenarios):
        print(f"{i}: {scenario}")
    requested_scenario = scenarios[int(input("Scenario: "))]

    print(f"\nAvailable intercession rates: (only 1 and 2 available for AAMAS)")
    intercession_rates = list(mapping.values())
    for i, intercession_rate in enumerate(intercession_rates):
        print(f"{i}: {intercession_rate}")
    requested_intercession_rate = intercession_rates[int(input("Intercession rate: "))]

    plot_metric_against_interventionism(requested_metric, requested_scenario, requested_intercession_rate)
    print("==============================================")

# for metric, metric_data in interventionism_results.items():
#     for scenario, scenario_data in metric_data.items():
#         for intercession_rate, intercession_data in scenario_data.items():
#             if len(intercession_data) == 0:
#                 continue

