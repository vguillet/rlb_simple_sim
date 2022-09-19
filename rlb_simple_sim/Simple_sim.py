
import os
import select
import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import ClockType

from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseStamped, Point
from rclpy.qos import QoSProfile
from rlb_utils.msg import Goal
import numpy as np
import math
import random


class Simple_sim(Node):
    def __init__(self):
        super().__init__('Simple_sim')

        # -> Initiate datastreams
        qos = QoSProfile(depth=10)
        
        # Goals publisher
        self.goal_sequence_publisher = self.create_publisher(
            msg_type=Goal,
            topic='/goals_backlog',
            qos_profile=qos
            )
        
        self.goal_emission_interval = .5
        # self.goal_emission_interval = 5.

        timer_period = self.goal_emission_interval  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.goal_sequence_publisher_callback
            )
        
        timer_period = 1.  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.timer_print
            )

        self.tick = self.goal_emission_interval


        self.goal_sequence = [
            {"robot_id": "Turtle_1", "goal_sequence_id": "11", "meta_action": "add", "priority": 0., "sequence": [Point(x=1.2, y=1.), Point(x=-1., y=1.5), Point(x=-1.7, y=-1.), Point(x=1., y=-1.)]},
            {"robot_id": "Turtle_1", "goal_sequence_id": "222", "meta_action": "add", "priority": 0., "sequence": [Point(x=1., y=1.), Point(x=-1., y=1.)]},
            {"robot_id": "Turtle_1", "goal_sequence_id": "333", "meta_action": "add", "priority": 1., "sequence": [Point(x=0., y=0.), Point(x=1., y=1.)]},
            {"robot_id": "Turtle_1", "goal_sequence_id": "444", "meta_action": "add", "priority": 20., "sequence": [Point(x=1., y=-0.)]},

            {"robot_id": "Turtle_2", "goal_sequence_id": "11", "meta_action": "add", "priority": 0., "sequence": [Point(x=1., y=1.), Point(x=-1., y=1.3), Point(x=-1.5, y=-1.), Point(x=1., y=-1.3)]},
            {"robot_id": "Turtle_2", "goal_sequence_id": "222", "meta_action": "add", "priority": 0., "sequence": [Point(x=1.7, y=1.), Point(x=-2., y=1.1)]},
            {"robot_id": "Turtle_2", "goal_sequence_id": "333", "meta_action": "add", "priority": 1., "sequence": [Point(x=0.3, y=0.), Point(x=1.2, y=1.)]},
            {"robot_id": "Turtle_2", "goal_sequence_id": "444", "meta_action": "add", "priority": 20., "sequence": [Point(x=1.1, y=-0.)]},
        
            {"robot_id": "Turtle_3", "goal_sequence_id": "11", "meta_action": "add", "priority": 0., "sequence": [Point(x=1., y=1.), Point(x=-1., y=1.), Point(x=-1., y=-1.), Point(x=1., y=-1.)]},
            {"robot_id": "Turtle_3", "goal_sequence_id": "222", "meta_action": "add", "priority": 0., "sequence": [Point(x=1., y=1.), Point(x=-1., y=1.)]},
            {"robot_id": "Turtle_3", "goal_sequence_id": "333", "meta_action": "add", "priority": 1., "sequence": [Point(x=0., y=0.), Point(x=1., y=1.)]},
            {"robot_id": "Turtle_3", "goal_sequence_id": "444", "meta_action": "add", "priority": 20., "sequence": [Point(x=1., y=-0.)]},

            {"robot_id": "Turtle_4", "goal_sequence_id": "11", "meta_action": "add", "priority": 0., "sequence": [Point(x=1.2, y=1.), Point(x=-1., y=1.5), Point(x=-1.7, y=-1.), Point(x=1., y=-1.)]},
            {"robot_id": "Turtle_4", "goal_sequence_id": "222", "meta_action": "add", "priority": 0., "sequence": [Point(x=1., y=1.), Point(x=-1., y=1.)]},
            {"robot_id": "Turtle_4", "goal_sequence_id": "333", "meta_action": "add", "priority": 1., "sequence": [Point(x=0., y=0.), Point(x=1., y=1.)]},
            {"robot_id": "Turtle_4", "goal_sequence_id": "444", "meta_action": "add", "priority": 20., "sequence": [Point(x=1., y=-0.)]},

            {"robot_id": "Turtle_5", "goal_sequence_id": "11", "meta_action": "add", "priority": 0., "sequence": [Point(x=1., y=1.), Point(x=-1., y=1.3), Point(x=-1.5, y=-1.), Point(x=1., y=-1.3)]},
            {"robot_id": "Turtle_5", "goal_sequence_id": "222", "meta_action": "add", "priority": 0., "sequence": [Point(x=1.7, y=1.), Point(x=-2., y=1.1)]},
            {"robot_id": "Turtle_5", "goal_sequence_id": "333", "meta_action": "add", "priority": 1., "sequence": [Point(x=0.3, y=0.), Point(x=1.2, y=1.)]},
            {"robot_id": "Turtle_5", "goal_sequence_id": "444", "meta_action": "add", "priority": 20., "sequence": [Point(x=1.1, y=-0.)]},
        
            # {"robot_id": "Turtle_6", "goal_sequence_id": "11", "meta_action": "add", "priority": 0., "sequence": [Point(x=1., y=1.), Point(x=-1., y=1.), Point(x=-1., y=-1.), Point(x=1., y=-1.)]},
            # {"robot_id": "Turtle_6", "goal_sequence_id": "222", "meta_action": "add", "priority": 0., "sequence": [Point(x=1., y=1.), Point(x=-1., y=1.)]},
            # {"robot_id": "Turtle_6", "goal_sequence_id": "333", "meta_action": "add", "priority": 1., "sequence": [Point(x=0., y=0.), Point(x=1., y=1.)]},
            # {"robot_id": "Turtle_6", "goal_sequence_id": "444", "meta_action": "add", "priority": 20., "sequence": [Point(x=1., y=-0.)]},
            
            # {"robot_id": "Turtle_1", "goal_sequence_id": "11", "meta_action": "add", "priority": 0., "sequence": [Point(x=0.67, y=.67), Point(x=-0.67, y=0.67), Point(x=-0.67, y=-0.67), Point(x=0.67, y=-0.67)]},            
            # {"robot_id": "Turtle_1", "goal_sequence_id": "222", "meta_action": "add", "priority": 0., "sequence": [Point(x=1., y=1.), Point(x=-1., y=1.)]},
            # {"robot_id": "Turtle_1", "goal_sequence_id": "333", "meta_action": "add", "priority": 1., "sequence": [Point(x=0., y=0.), Point(x=1., y=1.)]},
            # {"robot_id": "Turtle_1", "goal_sequence_id": "444", "meta_action": "add", "priority": 20., "sequence": [Point(x=1., y=-0.)]},
            # {"robot_id": "Turtle_1", "goal_sequence_id": "1", "meta_action": "add", "priority": 0., "sequence": [Point(x=0.0, y=.0), Point(x=2., y=0.0)]},            

        ]
        
        print(self.goal_sequence)
	
    def timer_print(self):
        self.tick -= 1

        if self.tick == 0:
            self.tick = self.goal_emission_interval
        else:
            print(self.tick)

    def goal_sequence_publisher_callback(self):
        # -> Fetch random goal sequence
        goal_id  = random.randint(0, len(self.goal_sequence)-1)
        goal_details = self.goal_sequence[goal_id]

        # -> Create message
        goal = Goal(
            robot_id=goal_details["robot_id"],
            goal_sequence_id=goal_details["goal_sequence_id"],
            meta_action=goal_details["meta_action"],
            priority=goal_details["priority"],
            sequence=goal_details["sequence"]
        )
        
        print(f"\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print(f"++ Goal sequence {goal.goal_sequence_id} emitted: {goal.sequence} for {goal.robot_id} ++")
        print(f"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

        # -> Publish instruction msg to robot
        self.goal_sequence_publisher.publish(msg=goal)

def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    path_sequence = Simple_sim()

    rclpy.spin(path_sequence)

    path_sequence.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
