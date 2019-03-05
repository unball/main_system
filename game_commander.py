#!/usr/bin/env pytho

import rospy

from std_msgs.msg import String

def start_commanding():
    command = String()
    rospy.init_node('game_commands')
    pub = rospy.Publisher('game_commands', String, queue_size=1)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        print("Type one of the following commands: <side:SIDE>|<game:COMMAND>")
        command.data = input(">>> ")
        output_msg = command.data
        pub.publish(output_msg)
        rate.sleep()


if __name__ == '__main__':
    print("---------- GAME COMMANDS NODE STARTED ----------")
    start_commanding()