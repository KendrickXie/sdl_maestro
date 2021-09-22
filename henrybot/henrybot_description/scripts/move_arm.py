#!/usr/bin/env python3
import rospy
import copy

from actionlib import SimpleActionClient
import geometry_msgs.msg
import move_base_msgs.msg

from collections import OrderedDict

tf_prefix = ''
static_transforms = OrderedDict()

def position1():
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 16.6
    pose.pose.position.y = 23.5
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = .0
    pose.pose.orientation.y = .0
    pose.pose.orientation.z = -.7
    pose.pose.orientation.w = -1.5
    return pose 

def position2():
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 16.4
    pose.pose.position.y = 26.1
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = .0
    pose.pose.orientation.y = .0
    pose.pose.orientation.z = -.6
    pose.pose.orientation.w = 1.3
    return pose 


def main():
    rospy.init_node('ur5e_move_test')
    #MiR_move()
    position_goal = rospy.Publisher("move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=10)
    position_goal.publish(pose)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


