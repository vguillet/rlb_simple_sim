
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
                ("scenario_id", "Scenario_0_full_intercession_no_recompute_0_interventionism_0.json"),
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

        # --> Trackers
        self.tasks_released = []

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
        #     callback=self._task_msg_subscriber_callback,
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
                     shared={},
                     local={}
                     ) -> TeamCommStamped:
        """
        Generate a TeamCommStamped msg from a task dictionary.
        """
        # -> Construct task
        new_task = Task(
            id=task_id,
            type=task_type,
            creator=agent_id,
            instructions=instructions,
            affiliations=affiliations,
            priority=priority,
            creation_timestamp=0,
            shared=shared,
            local=local
        )

        msg = TeamCommStamped()

        msg.trace = []
        msg.source = "Simple_sim"
        msg.target = agent_id
        msg.meta_action = meta_action
        msg.memo = dumps(new_task.asdict())

        return msg

    def fleet_msgs_callback(self, msg: TeamCommStamped):
        self.results["total_fleet_msgs_count"] += 1

    def goal_callback(self, msg: TeamCommStamped):
        """
        Callback for the goal subscription.
        """

        msg_memo = loads(msg.memo)

        # -> Construct source agent
        source_agent = Agent.from_dict(msg_memo["agent"])

        # -> If the agent is not in the fleet, add it
        if msg.source not in self.fleet.ids:
            self.robot_pose_sub[msg.source] = self.create_subscription(
                msg_type=PoseStamped,
                topic=f"/{msg.source}{topic_pose}",
                callback=partial(self.pose_callback, msg.source),
                qos_profile=qos_pose
            )

            self.fleet.add_agent(agent=source_agent)
            self.fleet[msg.source].local["tasks"] = {}

            # -> Add entry in histories
            self.results["pose_history"][msg.source] = []
            self.results["move_history"][msg.source] = []

        if msg.meta_action == "update":
            self.fleet[msg.source].plan = source_agent.plan
            self.fleet[msg.source].local["tasks"] = {}

            for task_id, task_dict in msg_memo["tasks"].items():
                self.fleet[msg.source].local["tasks"][task_id] = Task.from_dict(task_dict)

        self.results["total_goal_msgs_count"] += 1

    def pose_callback(self, agent_id, pose_msg: PoseStamped):
        """
        Callback for the pose subscriber

        :param pose_msg: PoseStamped message
        """
        # -> Convert quaternion to euler
        u, v, w = euler_from_quaternion(quat=pose_msg.pose.orientation)

        # -> Update state
        new_state = AgentState(
                agent_id=agent_id,
                x=pose_msg.pose.position.x,
                y=pose_msg.pose.position.y,
                z=pose_msg.pose.position.z,
                u=u,
                v=v,
                w=w,
                _timestamp=pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9
        )

        if agent_id not in self.fleet.ids:
            # self.get_logger().warning(f"Agent {agent_id} not found in fleet")
            return

        if self.fleet[agent_id].state != new_state:
            self.fleet[agent_id].state = new_state

            # -> Log agent move
            self.results['move_history'][agent_id].append((self.fleet[agent_id].state.x, self.fleet[agent_id].state.y))

        # -> Release task to agent if within radius of visibility
        visibility_range = self.scenario.agent_dict[agent_id]["visibility_range"]

        for goto_task in self.goto_tasks:
            # -> If task already released, skip
            if goto_task["id"] in self.tasks_released:
                continue

            # If task in range, update to release at next epoch
            if new_state.x - visibility_range <= goto_task["instructions"]["x"] <= new_state.x + visibility_range and \
                    new_state.y - visibility_range <= goto_task["instructions"]["y"] <= new_state.y + visibility_range:

                goto_task["epoch"] = self.sim_epoch + 1

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

            # -> Publish task completion
            self.task_pub.publish(msg=task_msg)
            self.task_pub.publish(msg=task_msg)
            self.task_pub.publish(msg=task_msg)

            # -> If action task, construct corresponding ACTION task
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
                    },
                    shared={
                        "intervention": task_dict["intervention"]
                    }
                )

                # -> Publish action task
                self.task_pub.publish(msg=action_task_msg)
                self.task_pub.publish(msg=action_task_msg)
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

    def sim_epoch_callback(self, msg):
        if self.results["sim_start_time"] is None:
            self.results["sim_start_time"] = datetime.now()

        sim_state = loads(msg.memo)

        self.sim_epoch = sim_state["epoch"]

        for goto_task in self.goto_tasks:
            if goto_task["epoch"] == self.sim_epoch:        # TODO: Add task release checking
                task_msg = self.get_task_msg(
                    meta_action="pending",
                    agent_id=goto_task["creator_id"],
                    task_id=goto_task["id"],
                    affiliations=goto_task["affiliations"],
                    priority=goto_task["priority"],
                    task_type=goto_task["type"],
                    instructions=goto_task["instructions"],
                    shared={
                        "intervention": goto_task["intervention"]
                    }
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
                self.task_pub.publish(msg=task_msg)
                self.task_pub.publish(msg=task_msg)

                self.tasks_released.append(goto_task["id"])

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
            if agent.plan is not None:
                no_action_tasks = []
                action_1_tasks = []
                action_2_tasks = []

                for task_id in agent.plan.task_sequence:
                    if agent.local["tasks"][task_id].instructions["ACTION_AT_LOC"] == "NO_TASK":
                        no_action_tasks.append(task_id)
                    elif agent.local["tasks"][task_id].instructions["ACTION_AT_LOC"] == "ACTION_1":
                        action_1_tasks.append(task_id)
                    elif agent.local["tasks"][task_id].instructions["ACTION_AT_LOC"] == "ACTION_2":
                        action_2_tasks.append(task_id)
                    else:
                        self.get_logger().warning(f"!!! Unknown action type for task {task_id}")

                self.get_logger().info(f"    - {agent.id}: {agent.plan.task_sequence}\n > No action: {no_action_tasks}\n > Action 1: {action_1_tasks}\n > Action 2: {action_2_tasks}")

        # -> FInd all doubles
        doubles = {}
        for agent in self.fleet:
            if agent.plan is not None:
                for task in agent.plan.task_sequence:
                    if task in doubles:
                        doubles[task].append(agent.id)
                    else:
                        doubles[task] = [agent.id]

        for task, agents in doubles.items():
            if len(agents) > 1:
                self.get_logger().warning(f"!!! Overlapping allocation of task {task} between {agents}")

        terminate = False

        if len(self.results["allocation"]) == self.results["total_task_count"]:
            self.results["last_epoch"] = self.sim_epoch
            self.results["sim_end_time"] = datetime.now()

            self.get_logger().info(f"    -  End of simulation")
            self.get_logger().info(f"-------------------------------------------")

            run_recap = self.results.generate_run_recap()
            self.get_logger().info(f"\n\n{run_recap}")

            self.get_logger().info(f" >> Dumping results to file")
            self.results.dump_results()

            terminate = True

        elif self.sim_epoch >= 400:
            terminate = True

        if terminate:
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
