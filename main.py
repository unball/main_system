#!/usr/bin/env python
"""Main module system. World -> Strategy -> Control."""

import rospy

from vision.msg import VisionMessage

import time

import world_standards
from world import World
from strategy import Strategy
from controllers.SSTrajRegulator import SSTrajRegulator


def update(data, world_state):
    global start
    end = time.time()
    interval_time = end - start

    world_state.update(data)
    world_state.calc_velocities(interval_time)

    # print(interval_time)

    start = time.time()


def start_system():
    """Start the system in the 'main function'."""
    global start
    start = 0

    world_state = World(world_standards.STANDARD3)
    strategy_system = Strategy()
    control_system = SSTrajRegulator()

    rospy.init_node('main_system')
    rospy.Subscriber('vision_output_topic', VisionMessage, update, world_state)

    while not rospy.is_shutdown():
        strategy_system.plan(world_state)
        targets = strategy_system.get_targets()
        control_system.actuate(targets, world_state)
        pass


if __name__ == '__main__':
    print("---------- MAIN SYSTEM NODE STARTED ----------")
    start_system()
