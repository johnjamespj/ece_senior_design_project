#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from interbotix_xs_msgs.msg import JointSingleCommand
from queue import Queue
import threading

rospy.init_node('simple_controller', anonymous=True)
pub = rospy.Publisher('/locobot/commands/joint_single', JointSingleCommand, queue_size=10)
rate = rospy.Rate(10)

def callback(data):
    # joint_cmds = []
    # for x in range(len(data.name)):
    #     joint_cmd = JointSingleCommand(name=data.name[x], cmd=data.position[x])
    #     joint_cmd.mode = JointSingleCommand.POSITION_MODE
    #     pub.publish(joint_cmd)
    print(data)

def subscriber():
    rospy.Subscriber('/locobot/joint_states', JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
