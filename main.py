#!/usr/bin/env python
"""Main module system. World -> Strategy -> Control."""

import numpy as np
import rospy

from vision.msg import VisionMessage
from std_msgs.msg import String
from communication.msg import robots_speeds_msg
from communication.msg import comm_msg

import time

import world_standards
from world.world import World
from strategy.strategy import Strategy
from controllers.ssRegulator import ssRegulator
from controllers.utils import convSpeeds2Motors
import field


def updateWorld(data, world_state):
    global start
    end = time.time()
    interval_time = end - start

    world_state.update(data)
    world_state.calc_velocities(interval_time)

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
    control_system = ssRegulator()

    rospy.init_node('main_system')
    rospy.Subscriber('vision_output_topic', VisionMessage, updateWorld, world_state)
    rospy.Subscriber('game_commands', String, commandGame, world_state)
    pubSimulator = rospy.Publisher('robots_speeds', robots_speeds_msg, queue_size=1)
    pubRadio = rospy.Publisher('radio_topic', comm_msg, queue_size=1)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        loop_start = time.time()
        strategy_system.plan(world_state)
        targets = strategy_system.get_targets()

        output_msgRadio = comm_msg()

        # # Targets bypass
        # # Used in control tests
        # targets = [[world_state.ball.x, world_state.ball.y, np.arctan2(world_state.ball.y - world_state.robots[0].y, world_state.ball.x - world_state.robots[0].x)],
        #            [0, 0, 0],
        #            [0, 0, 0]] 

        if world_state.isPaused:
            output_msgSim = robots_speeds_msg()

        elif not world_state.isPaused:
            print(targets)
            velocities = control_system.actuate(targets, world_state)
            print(velocities)
            output_msgSim = velocities
            output_msgRadio = convSpeeds2Motors(velocities)
            
            # # Velocity bypass
            # # Used in firmware tests
            # output_msgRadio.MotorA[0] = 50
            # output_msgRadio.MotorB[0] = -50
            # output_msgRadio.MotorA[1] = 100
            # output_msgRadio.MotorB[1] = -100
            # output_msgRadio.MotorA[2] = -500
            # output_msgRadio.MotorB[2] = 500

        pubSimulator.publish(output_msgSim)
        print(output_msgRadio)
        pubRadio.publish(output_msgRadio)
        rate.sleep()
        loop_end = time.time()
        loop_time = loop_end - loop_start
        print("Loop time: ", loop_time)


if __name__ == '__main__':
    print("---------- MAIN SYSTEM NODE STARTED ----------")
    start_system()
