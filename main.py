#!/usr/bin/env python
"""Main module system. World -> Strategy -> Control."""

import numpy as np
import rospy

from vision.msg import VisionMessage
from std_msgs.msg import String
from communication.msg import robots_speeds_msg

import time

import world_standards
from world.world import World
from strategy.strategy import Strategy
from controllers.ssLQRregulator import ssLQRregulator
import field


def updateWorld(data, world_state):
    global start
    end = time.time()
    interval_time = end - start

    world_state.update(data)
    world_state.calc_velocities(interval_time)

    # print(interval_time)

    start = time.time()

def commandGame(data, world_state):
    command = data.data.split(":")
    command[0] = command[0].lower()
    command[1] = command[1].upper()
    if command[0] == "side":
        if command[1] == "LEFT":
            world_state.change_field_side(field.LEFT)
        if command[1] == "RIGHT":
            world_state.change_field_side(field.RIGHT)
    elif command[0] == "game":
        if command[1] == "PAUSE":
            world_state.pause()
        elif command[1] == "PLAY":
            world_state.play()
    elif command[0] == "score":
        if command[1] == "+":
            world_state.gameScore = world_state.gameScore + 1
        elif command[1] == "-":
            world_state.gameScore = world_state.gameScore - 1



def start_system():
    """Start the system in the 'main function'."""
    global start
    start = 0

    world_state = World(world_standards.STANDARD3)
    strategy_system = Strategy()
    control_system = ssLQRregulator()

    rospy.init_node('main_system')
    rospy.Subscriber('vision_output_topic', VisionMessage, updateWorld, world_state)
    rospy.Subscriber('game_commands', String, commandGame, world_state)
    pub = rospy.Publisher('robots_speeds', robots_speeds_msg, queue_size=1)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        strategy_system.plan(world_state)
        targets = strategy_system.get_targets()

        # # Targets bypass for controller tests
        #targets = [[-0.65, -0.2, np.pi/2], [0.5, 0.5, np.pi/4], [0, 0, 0]]

        if world_state.isPaused:
            output_msg = robots_speeds_msg()
        elif not world_state.isPaused:
            output_msg = control_system.actuate(targets, world_state)
        pub.publish(output_msg)
        rate.sleep()


if __name__ == '__main__':
    print("---------- MAIN SYSTEM NODE STARTED ----------")
    start_system()
