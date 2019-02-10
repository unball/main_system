#!/usr/bin/env python
"""Main module system. World -> Strategy -> Control."""

import rospy

from vision.msg import VisionMessage

# import world_standards
# from world import World
# from strategy import Strategy
# from controllers.controller import Controller


def callback(data):
    rospy.loginfo("UHUUUL")
    print("blablablabla")


def start_system():
    """Start the system in the 'main function'."""
    # world_state = World(world_standards.STANDARD3)
    # strategy_system = Strategy()
    # control_system = Controller()

    rospy.init_node('main_system')
    rospy.Subscriber('/vision_output_topic', VisionMessage, callback)

    # world_state.dummy_update()
    # strategy_system.plan(world_state)
    # targets = strategy_system.get_targets()
    # control_system.actuate(targets)
    rospy.spin()


if __name__ == '__main__':
    print("---------- MAIN SYSTEM NODE STARTED ----------")
    start_system()
