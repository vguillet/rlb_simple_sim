
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
from functools import partial
import time

# Libs
import numpy as np
import pandas as pd

# Suppress FutureWarning messages
warnings.simplefilter(action='ignore', category=FutureWarning)

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped, Point
import numpy as np

# Local Imports
from orchestra_config.orchestra_config import *     # KEEP THIS LINE, DO NOT REMOVE
from maaf_msgs.msg import TeamCommStamped
from .Scenario import Scenario

from maaf_tools.datastructures.task.Task import Task
from maaf_tools.datastructures.task.TaskLog import TaskLog

from maaf_tools.datastructures.agent.Agent import Agent
from maaf_tools.datastructures.agent.Fleet import Fleet
from maaf_tools.datastructures.agent.AgentState import AgentState

from maaf_tools.tools import euler_from_quaternion
from .results_gen import Results

######################################################################################################


class RLB_simple_sim(Node):
    def __init__(self):
        super().__init__('Simple_sim')

        # -> Get launch parameters configuration
        self.declare_parameters(
            namespace="",
            parameters=[
                ("scenario_id", "simple_sim"),
            ]
        )

        scenario_id = self.get_parameter("scenario_id").get_parameter_value().string_value

        self.scenario = Scenario(scenario_id=scenario_id, load_only=True, logger=self.get_logger())

        self.fleet = Fleet()

        self.results = Results(
            fleet=self.fleet,
            scenario=self.scenario
        )

        self.goto_tasks = self.scenario.goto_tasks

        # ---------------------------------- Subscribers
        # ---------- epoch
        self.sim_epoch_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_epoch,
            callback=self.sim_epoch_callback,
            qos_profile=qos_sim_epoch
        )

        # ---------- goal
        self.goals_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_goals,
            callback=self.goal_callback,
            qos_profile=qos_goal
        )

        # # ---------- tasks
        # self.task_sub = self.create_subscription(
        #     msg_type=TeamCommStamped,
        #     topic=topic_tasks,
        #     callback=self.task_msg_subscriber_callback,
        #     qos_profile=qos_tasks
        # )

        # ---------- fleet_msgs
        self.fleet_msgs_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_fleet_msgs,
            callback=self.fleet_msgs_callback,
            qos_profile=qos_fleet_msgs
        )

        # ---------- sim_events_instructions
        self.sim_events_instructions_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_sim_events_instructions,
            callback=self.sim_events_instructions_callback,
            qos_profile=qos_sim_events_instructions
        )

        # ---------- /robot_.../pose
        self.robot_pose_sub = {}

        # ---------------------------------- Publishers
        # ---------- tasks
        self.task_pub = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=topic_tasks,
            qos_profile=qos_tasks
        )

        # ---------- simulator_signals
        self.simulator_signals_pub = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=topic_simulator_signals,
            qos_profile=qos_simulator_signals
        )

        self.get_logger().info(f"Simple_sim node initialised - Scenario ID: {scenario_id}")

    def get_task_msg(self,
                     meta_action,
                     agent_id,
                     task_id,
                     affiliations,
                     priority,
                     task_type,
                     instructions,
                     ) -> TeamCommStamped:
        """
        Generate a TeamCommStamped msg from a task dictionary.
        """
        msg = TeamCommStamped()

        msg.trace = []
        msg.source = "Simple_sim"
        msg.target = agent_id
        msg.meta_action = meta_action
        msg.memo = dumps({
            "id": task_id,
            "type": task_type,
            "creator": agent_id,
            "affiliations": affiliations,

            "priority": priority,
            "instructions": instructions,
            "creation_timestamp": 0,
            "termination_timestamp": None,
            "termination_source_id": None,
            "status": "pending"
        })

        return msg

    def fleet_msgs_callback(self, msg: TeamCommStamped):
        self.results["total_fleet_msgs_count"] += 1

    def goal_callback(self, msg: TeamCommStamped):
        """
        Callback for the goal subscription.
        """

        msg_memo = loads(msg.memo)

        # -> If the agent is not in the fleet, add it
        if msg.source not in self.fleet.ids:
            self.robot_pose_sub[msg.source] = self.create_subscription(
                msg_type=PoseStamped,
                topic=f"/{msg.source}{topic_pose}",
                callback=partial(self.pose_callback, msg.source),
                qos_profile=qos_pose
            )

            self.fleet.add_agent(agent=msg_memo["agent"])

            # -> Add entry in histories
            self.results["pose_history"][msg.source] = []
            self.results["move_history"][msg.source] = []

        # -> Update the agent state
        if msg.meta_action == "assign":
            self.fleet[msg.source].local["goal"] = msg_memo["task"]

        elif msg.meta_action == "unassign":
            self.fleet[msg.source].local["goal"] = None

        else:
            self.fleet[msg.source].local["goal"] = None

        self.results["total_goal_msgs_count"] += 1

    def pose_callback(self, agent_id, pose_msg: PoseStamped):
        """
        Callback for the pose subscriber

        :param pose_msg: PoseStamped message
        """
        # -> Convert quaternion to euler
        u, v, w = euler_from_quaternion(quat=pose_msg.pose.orientation)

        # -> Update state
        new_state = AgentState.from_dict(
            agent_dict={
                "agent_id": agent_id,
                "x": pose_msg.pose.position.x,
                "y": pose_msg.pose.position.y,
                "z": pose_msg.pose.position.z,
                "u": u,
                "v": v,
                "w": w,
                "timestamp": pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9
            },
            partial=True
        )

        if agent_id not in self.fleet.ids:
            # self.get_logger().warning(f"Agent {agent_id} not found in fleet")
            return

        if self.fleet[agent_id].state != new_state:
            self.fleet[agent_id].state = new_state

            # -> Log agent move
            self.results['move_history'][agent_id].append((self.fleet[agent_id].state.x, self.fleet[agent_id].state.y))

    def sim_events_instructions_callback(self, task_msg: TeamCommStamped):
        """
        Callback for task subscription.
        """

        if task_msg.source == "Simple_sim":
            return

        # -> Unpack msg
        task_dict = loads(task_msg.memo)

        if task_msg.meta_action == "completed": # TODO: Add support for more meta actions
            # -> Save termination epoch
            self.results["task_history"][task_dict["id"]]["termination_epoch"] = self.sim_epoch

            # -> Log allocation completion
            self.results["allocation"][task_dict["id"]] = {
                "agent_id": task_msg.target,
                "epoch": self.sim_epoch,
                "goal": task_dict
            }

            # -> If no action at location, do nothing
            if task_dict["instructions"]["ACTION_AT_LOC"] != "NO_TASK":
                # -> Construct corresponding ACTION task
                action_task_msg = self.get_task_msg(
                    meta_action="pending",
                    agent_id=task_msg.target,
                    task_id=task_dict["id"] + "'",
                    affiliations=task_dict["affiliations"],
                    priority=task_dict["priority"],
                    task_type=task_dict["instructions"]["ACTION_AT_LOC"],
                    instructions={
                        "x": task_dict["instructions"]["x"],
                        "y": task_dict["instructions"]["y"],
                        "ACTION_AT_LOC": "NO_TASK"
                    }
                )

                # -> Publish action task
                self.task_pub.publish(msg=action_task_msg)

                # -> Save task in history
                self.results["task_history"][task_dict["id"] + "'"] = {
                    "type": task_dict["instructions"]["ACTION_AT_LOC"],
                    "instructions": {
                        "x": task_dict["instructions"]["x"],
                        "y": task_dict["instructions"]["y"],
                        "ACTION_AT_LOC": "NO_TASK"
                    },
                    "creator_id": task_msg.target,
                    "affiliations": task_dict["affiliations"],
                    "priority": task_dict["priority"],
                    "creation_timestamp": self.sim_epoch,
                    "release_epoch": self.sim_epoch,
                    "termination_epoch": None
                }

            # -> Publish task completion
            self.task_pub.publish(msg=task_msg)

    def sim_epoch_callback(self, msg):
        if self.results["sim_start_time"] is None:
            self.results["sim_start_time"] = datetime.now()

        sim_state = loads(msg.memo)

        self.sim_epoch = sim_state["epoch"]

        for goto_task in self.goto_tasks:
            if goto_task["epoch"] == self.sim_epoch:
                task_msg = self.get_task_msg(
                    meta_action="pending",
                    agent_id=goto_task["creator_id"],
                    task_id=goto_task["id"],
                    affiliations=goto_task["affiliations"],
                    priority=goto_task["priority"],
                    task_type=goto_task["type"],
                    instructions=goto_task["instructions"]
                )

                print(f"\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
                print(f"++ EPOCH {self.sim_epoch}: Task {goto_task['id']} > {goto_task['type']} emitted: {goto_task['instructions']} for {goto_task['creator_id']} ++")
                print(f"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

                # -> Save task in history
                self.results["task_history"][goto_task["id"]] = {
                    "type": goto_task["type"],
                    "instructions": goto_task["instructions"],
                    "creator_id": goto_task["creator_id"],
                    "affiliations": goto_task["affiliations"],
                    "priority": goto_task["priority"],
                    "creation_timestamp": self.sim_epoch,
                    "release_epoch": self.sim_epoch,
                    "termination_epoch": None,
                }

                # -> Publish instruction msg to robot
                self.task_pub.publish(msg=task_msg)

        # -> Log agent pose
        for agent in self.fleet:
            self.results["pose_history"][agent.id].append((agent.state.x, agent.state.y))

        # -> Get run stats
        total_moves = sum([len(self.results["move_history"][agent.id]) for agent in self.fleet])
        try:
            longest_moves = max([len(self.results["move_history"][agent.id]) for agent in self.fleet])
        except ValueError:
            longest_moves = 0

        self.get_logger().info(f"-------------------------------------------")
        self.get_logger().info(f" > Sim step {self.sim_epoch}")
        self.get_logger().info(f"    -  Total moves: {total_moves}")
        self.get_logger().info(
            f"    -  Longest move: {longest_moves} ({[agent.id for agent in self.fleet if len(self.results['move_history'][agent.id]) == longest_moves]})")
        self.get_logger().info(f"    -  Total fleet messages: {self.results['total_fleet_msgs_count']}")
        self.get_logger().info(
            f"    -  Total allocations completed: {len(self.results['allocation'])}/{self.results['total_task_count']}")
        self.get_logger().info(f" Runtime: {datetime.now() - self.results['sim_start_time']}")
        self.get_logger().info(f"-----------------")
        self.get_logger().info(f"Current goals:")
        for agent in self.fleet:
            if agent.local["goal"] is not None:
                self.get_logger().info(f"    - {agent.id}: {agent.local['goal']['id']}")

        if len(self.results["allocation"]) == self.results["total_task_count"]:
            self.results["last_epoch"] = self.sim_epoch
            self.results["sim_end_time"] = datetime.now()

            self.get_logger().info(f"    -  End of simulation")
            self.get_logger().info(f"-------------------------------------------")

            run_recap = self.results.generate_run_recap()
            self.get_logger().info(f"\n\n{run_recap}")

            self.get_logger().info(f" >> Dumping results to file")
            self.results.dump_results()

            # -> Publish terminate simulator nodes signal
            for _ in range(10):
                self.simulator_signals_pub.publish(msg=TeamCommStamped(
                    source="",
                    target="",
                    meta_action="order 66",
                    memo=""
                    )
                )

                self.get_logger().info("Received order 66: Terminating simulation")
                time.sleep(0.1)

            # -> Terminate node
            self.destroy_node()

            # -> Terminate script
            sys.exit()


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_instructions = RLB_simple_sim()

    rclpy.spin(path_instructions)

    path_instructions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
