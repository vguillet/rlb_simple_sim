
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
from maaf_allocation_node.node_config import *
from maaf_msgs.msg import TeamCommStamped

######################################################################################################

# Create a RandomState object with a specific seed
rng = np.random.RandomState(seed=seed)


class RLB_simple_sim(Node):
    def __init__(self):
        super().__init__('Simple_sim')

        # ---------------------------------- Subscribers
        # ---------- epoch
        self.sim_epoch_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_epoch,
            callback=self.sim_epoch_callback,
            qos_profile=qos_sim_epoch
        )

        # ---------- tasks
        # self.task_sub = self.create_subscription(
        #     msg_type=TeamCommStamped,
        #     topic=topic_tasks,
        #     callback=self.task_msg_subscriber_callback,
        #     qos_profile=qos_tasks
        # )

        # ---------- sim_events_instructions
        self.sim_events_instructions_sub = self.create_subscription(
            msg_type=TeamCommStamped,
            topic=topic_sim_events_instructions,
            callback=self.sim_events_instructions_callback,
            qos_profile=qos_sim_events_instructions
        )

        # ---------------------------------- Publishers
        # ---------- tasks
        self.task_pub = self.create_publisher(
            msg_type=TeamCommStamped,
            topic=topic_tasks,
            qos_profile=qos_tasks
        )

        # ---------------- Generate GOTOs
        def generate_task(action_at_loc: str, target_agent_id: str, epoch: int) -> dict:
            # -> Generate task
            task = {
                "epoch": epoch,
                "creator_id": target_agent_id,
                "id": str(rng.randint(0, 1000000)),
                "type": "GOTO",
                "meta_action": "pending",
                "priority": 0.0,
                "affiliations": "base",
                "instructions": {
                    "x": rng.randint(0, env_size-1),
                    "y": rng.randint(0, env_size-1),
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
        self.goto_tasks = []

        for i in range(no_task_task_count):
            self.goto_tasks.append(
                generate_task(
                    action_at_loc="NO_TASK",
                    target_agent_id=rng.choice(agent_lst),
                    epoch=0
                )
            )

        for i in range(action_1_task_count):
            self.goto_tasks.append(
                generate_task(
                    action_at_loc="ACTION_1",
                    target_agent_id=rng.choice(agent_lst),
                    epoch=0
                )
            )

        for i in range(action_2_task_count):
            self.goto_tasks.append(
                generate_task(
                    action_at_loc="ACTION_2",
                    target_agent_id=rng.choice(agent_lst),
                    epoch=0
                )
            )

        # > Sort list by euclidian distance from start
        self.goto_tasks = sorted(self.goto_tasks, key=lambda x: x["euclidian_distance_from_start"])

        # > Update tasks according to schedule
        # > Set first len(initial_tasks_announcement) tasks to epoch 0
        for i in range(initial_tasks_announcement):
            self.goto_tasks[i]["epoch"] = 0

        # > Set the rest of the tasks to random epochs in sequence
        avg_step_size = release_max_epoch // (len(self.goto_tasks) - initial_tasks_announcement)

        step = 0

        for task in self.goto_tasks[initial_tasks_announcement:]:
            step += avg_step_size + rng.randint(0, ceil(avg_step_size / 2))

            if step > release_max_epoch:
                step = release_max_epoch

            task["epoch"] = step

        self.get_logger().info("Simple_sim node initialised")

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

    def sim_events_instructions_callback(self, task_msg: TeamCommStamped):
        """
        Callback for task subscription.
        """

        if task_msg.source == "Simple_sim":
            return

        # -> Unpack msg
        task_dict = loads(task_msg.memo)

        if task_msg.meta_action == "completed":
            # -> If no action at location, do nothing
            if task_dict["instructions"]["ACTION_AT_LOC"] == "NO_TASK":
                # -> Publish task completion
                self.task_pub.publish(msg=task_msg)
                return

            # -> Construct corresponding ACTION task
            action_task_msg = self.get_task_msg(
                meta_action="pending",
                agent_id=task_msg.target,
                task_id=task_dict["id"] + "0",
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

        # -> Publish task completion
        self.task_pub.publish(msg=task_msg)

    def sim_epoch_callback(self, msg):
        sim_state = loads(msg.memo)

        sim_epoch = sim_state["epoch"]
        print(f">>>>>>>>>>>>>>> Sim epoch: {sim_epoch}")

        for goto_task in self.goto_tasks:
            if goto_task["epoch"] == sim_epoch:
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
                print(f"++ Task {goto_task['id']} > {goto_task['type']} emitted: {goto_task['instructions']} for {goto_task['creator_id']} ++")
                print(f"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

                # -> Publish instruction msg to robot
                self.task_pub.publish(msg=task_msg)


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_instructions = RLB_simple_sim()

    rclpy.spin(path_instructions)

    path_instructions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
