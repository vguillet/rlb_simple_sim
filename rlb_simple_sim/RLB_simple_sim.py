import os
import select
import sys
import rclpy
import time
from json import dumps
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import ClockType

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile
from rlb_utils.msg import Goal
from maaf_msgs.msg import TeamCommStamped
import numpy as np
import math
import random


class RLB_simple_sim(Node):
    def __init__(self):
        super().__init__('Simple_sim')

        # -> Initiate datastreams
        qos = QoSProfile(depth=10)

        # Goals publisher
        self.task_publisher = self.create_publisher(
            # msg_type=Goal,
            msg_type=TeamCommStamped,
            topic='/fleet/task',
            qos_profile=qos
        )

        self.goal_emission_interval = .5
        # self.goal_emission_interval = 5.

        timer_period = self.goal_emission_interval  # seconds
        self.timer = self.create_timer(
            timer_period,
            self.goal_instructions_publisher_callback
        )

        timer_period = 1.  # seconds
        self.timer = self.create_timer(
            timer_period,
            self.timer_print
        )

        self.tick = self.goal_emission_interval

        # self.goal_instructions = [
        #     {
        #         "creator_id": "Turtle_1",
        #         "id": 1,
        #         "meta_action": "add",
        #         "priority": 0.0,
        #         "instructions": [{"x": 1.2, "y": 1.0}, {"x": -1.0, "y": 1.5}, {"x": -1.7, "y": -1.0},
        #                      {"x": 1.0, "y": -1.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_1",
        #         "id": 2,
        #         "meta_action": "add",
        #         "priority": 0.0,
        #         "instructions": [{"x": 1.0, "y": 1.0}, {"x": -1.0, "y": 1.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_1",
        #         "id": 3,
        #         "meta_action": "add",
        #         "priority": 1.0,
        #         "instructions": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_1",
        #         "id": 4,
        #         "meta_action": "add",
        #         "priority": 20.0,
        #         "instructions": [{"x": 1.0, "y": -0.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_2",
        #         "id": 5,
        #         "meta_action": "add",
        #         "priority": 0.0,
        #         "instructions": [{"x": 1.0, "y": 1.0}, {"x": -1.0, "y": 1.3}, {"x": -1.5, "y": -1.0},
        #                      {"x": 1.0, "y": -1.3}]
        #     },
        #     {
        #         "creator_id": "Turtle_2",
        #         "id": 6,
        #         "meta_action": "add",
        #         "priority": 0.0,
        #         "instructions": [{"x": 1.7, "y": 1.0}, {"x": -2.0, "y": 1.1}]
        #     },
        #     {
        #         "creator_id": "Turtle_2",
        #         "id": 7,
        #         "meta_action": "add",
        #         "priority": 1.0,
        #         "instructions": [{"x": 0.3, "y": 0.0}, {"x": 1.2, "y": 1.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_2",
        #         "id": 8,
        #         "meta_action": "add",
        #         "priority": 20.0,
        #         "instructions": [{"x": 1.1, "y": -0.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_3",
        #         "id": 9,
        #         "meta_action": "add",
        #         "priority": 0.0,
        #         "instructions": [{"x": 1.0, "y": 1.0}, {"x": -1.0, "y": 1.0}, {"x": -1.0, "y": -1.0},
        #                      {"x": 1.0, "y": -1.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_3",
        #         "id": 10,
        #         "meta_action": "add",
        #         "priority": 0.0,
        #         "instructions": [{"x": 1.0, "y": 1.0}, {"x": -1.0, "y": 1.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_3",
        #         "id": 11,
        #         "meta_action": "add",
        #         "priority": 1.0,
        #         "instructions": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_3",
        #         "id": 12,
        #         "meta_action": "add",
        #         "priority": 20.0,
        #         "instructions": [{"x": 1.0, "y": -0.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_4",
        #         "id": 13,
        #         "meta_action": "add",
        #         "priority": 0.0,
        #         "instructions": [{"x": 1.2, "y": 1.0}, {"x": -1.0, "y": 1.5}, {"x": -1.7, "y": -1.0},
        #                      {"x": 1.0, "y": -1.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_4",
        #         "id": 14,
        #         "meta_action": "add",
        #         "priority": 0.0,
        #         "instructions": [{"x": 1.0, "y": 1.0}, {"x": -1.0, "y": 1.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_4",
        #         "id": 15,
        #         "meta_action": "add",
        #         "priority": 1.0,
        #         "instructions": [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_4",
        #         "id": 16,
        #         "meta_action": "add",
        #         "priority": 20.0,
        #         "instructions": [{"x": 1.0, "y": -0.0}]
        #     },
        #     {
        #         "creator_id": "Turtle_5",
        #         "id": 17,
        #         "meta_action": "add",
        #         "priority": 0.0,
        #         "instructions": [{"x": 1.0, "y": 1.0}, {"x": -1.0, "y": 1.3}, {"x": -1.5, "y": -1.0},
        #                      {"x": 1.0, "y": -1.3}]
        #     },
        #     {
        #         "creator_id": "Turtle_5",
        #         "id": 18,
        #         "meta_action": "add",
        #         "priority": 0.0,
        #         "instructions": [{"x": 1.7, "y": 1.0}, {"x": -2.0, "y": 1.1}]
        #     }
        #
        #     # {"creator_id": "Turtle_1", "id": "11", "meta_action": "add", "priority": 0.,
        #     #  "instructions": [Point(x=1.2, y=1.), Point(x=-1., y=1.5), Point(x=-1.7, y=-1.), Point(x=1., y=-1.)]},
        #     # {"creator_id": "Turtle_1", "id": "222", "meta_action": "add", "priority": 0.,
        #     #  "instructions": [Point(x=1., y=1.), Point(x=-1., y=1.)]},
        #     # {"creator_id": "Turtle_1", "id": "333", "meta_action": "add", "priority": 1.,
        #     #  "instructions": [Point(x=0., y=0.), Point(x=1., y=1.)]},
        #     # {"creator_id": "Turtle_1", "id": "444", "meta_action": "add", "priority": 20.,
        #     #  "instructions": [Point(x=1., y=-0.)]},
        #     #
        #     # {"creator_id": "Turtle_2", "id": "11", "meta_action": "add", "priority": 0.,
        #     #  "instructions": [Point(x=1., y=1.), Point(x=-1., y=1.3), Point(x=-1.5, y=-1.), Point(x=1., y=-1.3)]},
        #     # {"creator_id": "Turtle_2", "id": "222", "meta_action": "add", "priority": 0.,
        #     #  "instructions": [Point(x=1.7, y=1.), Point(x=-2., y=1.1)]},
        #     # {"creator_id": "Turtle_2", "id": "333", "meta_action": "add", "priority": 1.,
        #     #  "instructions": [Point(x=0.3, y=0.), Point(x=1.2, y=1.)]},
        #     # {"creator_id": "Turtle_2", "id": "444", "meta_action": "add", "priority": 20.,
        #     #  "instructions": [Point(x=1.1, y=-0.)]},
        #     #
        #     # {"creator_id": "Turtle_3", "id": "11", "meta_action": "add", "priority": 0.,
        #     #  "instructions": [Point(x=1., y=1.), Point(x=-1., y=1.), Point(x=-1., y=-1.), Point(x=1., y=-1.)]},
        #     # {"creator_id": "Turtle_3", "id": "222", "meta_action": "add", "priority": 0.,
        #     #  "instructions": [Point(x=1., y=1.), Point(x=-1., y=1.)]},
        #     # {"creator_id": "Turtle_3", "id": "333", "meta_action": "add", "priority": 1.,
        #     #  "instructions": [Point(x=0., y=0.), Point(x=1., y=1.)]},
        #     # {"creator_id": "Turtle_3", "id": "444", "meta_action": "add", "priority": 20.,
        #     #  "instructions": [Point(x=1., y=-0.)]},
        #     #
        #     # {"creator_id": "Turtle_4", "id": "11", "meta_action": "add", "priority": 0.,
        #     #  "instructions": [Point(x=1.2, y=1.), Point(x=-1., y=1.5), Point(x=-1.7, y=-1.), Point(x=1., y=-1.)]},
        #     # {"creator_id": "Turtle_4", "id": "222", "meta_action": "add", "priority": 0.,
        #     #  "instructions": [Point(x=1., y=1.), Point(x=-1., y=1.)]},
        #     # {"creator_id": "Turtle_4", "id": "333", "meta_action": "add", "priority": 1.,
        #     #  "instructions": [Point(x=0., y=0.), Point(x=1., y=1.)]},
        #     # {"creator_id": "Turtle_4", "id": "444", "meta_action": "add", "priority": 20.,
        #     #  "instructions": [Point(x=1., y=-0.)]},
        #     #
        #     # {"creator_id": "Turtle_5", "id": "11", "meta_action": "add", "priority": 0.,
        #     #  "instructions": [Point(x=1., y=1.), Point(x=-1., y=1.3), Point(x=-1.5, y=-1.), Point(x=1., y=-1.3)]},
        #     # {"creator_id": "Turtle_5", "id": "222", "meta_action": "add", "priority": 0.,
        #     #  "instructions": [Point(x=1.7, y=1.), Point(x=-2., y=1.1)]},
        #     # {"creator_id": "Turtle_5", "id": "333", "meta_action": "add", "priority": 1.,
        #     #  "instructions": [Point(x=0.3, y=0.), Point(x=1.2, y=1.)]},
        #     # {"creator_id": "Turtle_5", "id": "444", "meta_action": "add", "priority": 20.,
        #     #  "instructions": [Point(x=1.1, y=-0.)]},
        #
        #     # {"creator_id": "Turtle_6", "id": "11", "meta_action": "add", "priority": 0., "instructions": [Point(x=1., y=1.), Point(x=-1., y=1.), Point(x=-1., y=-1.), Point(x=1., y=-1.)]},
        #     # {"creator_id": "Turtle_6", "id": "222", "meta_action": "add", "priority": 0., "instructions": [Point(x=1., y=1.), Point(x=-1., y=1.)]},
        #     # {"creator_id": "Turtle_6", "id": "333", "meta_action": "add", "priority": 1., "instructions": [Point(x=0., y=0.), Point(x=1., y=1.)]},
        #     # {"creator_id": "Turtle_6", "id": "444", "meta_action": "add", "priority": 20., "instructions": [Point(x=1., y=-0.)]},
        #
        #     # {"creator_id": "Turtle_1", "id": "11", "meta_action": "add", "priority": 0., "instructions": [Point(x=0.67, y=.67), Point(x=-0.67, y=0.67), Point(x=-0.67, y=-0.67), Point(x=0.67, y=-0.67)]},
        #     # {"creator_id": "Turtle_1", "id": "222", "meta_action": "add", "priority": 0., "instructions": [Point(x=1., y=1.), Point(x=-1., y=1.)]},
        #     # {"creator_id": "Turtle_1", "id": "333", "meta_action": "add", "priority": 1., "instructions": [Point(x=0., y=0.), Point(x=1., y=1.)]},
        #     # {"creator_id": "Turtle_1", "id": "444", "meta_action": "add", "priority": 20., "instructions": [Point(x=1., y=-0.)]},
        #     # {"creator_id": "Turtle_1", "id": "1", "meta_action": "add", "priority": 0., "instructions": [Point(x=0.0, y=.0), Point(x=2., y=0.0)]},
        # ]
        #
        # for goal in self.goal_instructions:
        #     goal["instructions"] = goal["instructions"][0]
        #
        # print(self.goal_instructions)

        agent_lst = ["Turtle_1", "Turtle_2", "Turtle_3", "Turtle_4", "Turtle_5", "Turtle_6"]
        x_range = [0, 6]
        y_range = [0, 6]

        self.goal_instructions = []

        for _ in range(10):
            self.goal_instructions.append(
                {
                    "creator_id": random.choice(agent_lst),
                    "id": random.randint(100, 1000),
                    "meta_action": "add",
                    "priority": 0.0,
                    "instructions": {"x": random.randint(x_range[0], x_range[1]), "y": random.randint(y_range[0], y_range[1])}
                }
            )

        # -> Generate tasks

    def timer_print(self):
        self.tick -= 1

        if self.tick == 0:
            self.tick = self.goal_emission_interval
        else:
            print(self.tick)

    def goal_instructions_publisher_callback(self):
        # -> Fetch a random goal instructions
        goal_id = random.randint(0, len(self.goal_instructions) - 1)
        goal_details = self.goal_instructions[goal_id]

        # -> Create message
        # msg = Goal(
        #     creator_id=goal_details["creator_id"],
        #     id=goal_details["id"],
        #     meta_action=goal_details["meta_action"],
        #     priority=goal_details["priority"],
        #     instructions=goal_details["instructions"]
        # )

        msg = TeamCommStamped()

        msg.trace = []
        msg.source = "Simple_sim"
        msg.target = goal_details["creator_id"]
        msg.meta_action = goal_details["meta_action"]
        msg.memo = dumps({
            "id": goal_details["id"],
            "type": "GOTO",
            "creator": "Simple_sim",
            "affiliations": "base",
            
            "priority": goal_details["priority"],
            "instructions": goal_details["instructions"],
            "creation_timestamp": 0,
            "termination_timestamp": None,
            "status": "pending"
        })

        print(f"\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print(f"++ Task {goal_details['id']} emitted: {goal_details['instructions']} for {goal_details['creator_id']} ++")
        print(f"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

        # -> Publish instruction msg to robot
        self.task_publisher.publish(msg=msg)


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_instructions = RLB_simple_sim()

    rclpy.spin(path_instructions)

    path_instructions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
