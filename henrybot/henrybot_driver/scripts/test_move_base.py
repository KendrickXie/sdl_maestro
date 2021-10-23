#!/usr/bin/env python3

from time import sleep 
import rospy
import geometry_msgs.msg
import numpy as np
import copy
import move_base_msgs.msg
from actionlib import SimpleActionClient


def main():

    pos_X = 16.33
    pos_Y = 15.0
    pos_th = -90

    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.pose.position.x = pos_X
    goal.target_pose.pose.position.y = pos_Y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = .0
    goal.target_pose.pose.orientation.y = .0
    goal.target_pose.pose.orientation.z = np.sin(pos_th/2)
    goal.target_pose.pose.orientation.w = np.cos(pos_th/2)
    print(goal)

    def move_base_simple_goal_callback(msg):
        if not move_base_client.wait_for_server(rospy.Duration(2)):
            rospy.logwarn("Could not connect to 'move_base' server after two seconds. Dropping goal.")
            rospy.logwarn("Did you activate 'planner' in the MIR web interface?")
            return
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header = copy.deepcopy(msg.header)
        goal.target_pose.pose = copy.deepcopy(msg.pose)
        move_base_client.send_goal_and_wait(goal)

    rospy.init_node('mir_move_test')

    move_base_client = SimpleActionClient('move_base_simple', move_base_msgs.msg.MoveBaseAction)
    rospy.Subscriber("move_base_simple/goal", geometry_msgs.msg.PoseStamped, move_base_simple_goal_callback)
    # pub = rospy.Publisher("move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=10)
    # sleep(1)
    # pub.publish(pose)
    move_base_client.wait_for_server()
    move_base_simple_goal_callback(goal)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


