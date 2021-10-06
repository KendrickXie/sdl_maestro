#!/usr/bin/env python3

from time import sleep 
import rospy
import geometry_msgs.msg
import numpy as np
from  argparse import ArgumentParser

class BaseClient:
    def __init__(self):
        self.pub = rospy.Publisher("move_base_simple/goal", geometry_msgs.msg.PoseStamped, queue_size=1)

    def get_pose(self, x, y, th):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = .0
        pose.pose.orientation.y = .0
        pose.pose.orientation.z = np.sin(np.pi*th/(2*180))
        pose.pose.orientation.w = np.cos(np.pi*th/(2*180))
        return pose

    def move(self, x, y, th):
        sleep(1)
        pose = self.get_pose(x,y,th)
        print(pose)
        self.pub.publish(pose)

    def move_rel(self, x,y,th):
        pass


def main():

    parser = ArgumentParser()
    parser.add_argument("-x", default=16.0, type=float)
    parser.add_argument("-y", default=23.0, type=float)
    parser.add_argument("-th", default=0.0, type=float)
    args = parser.parse_args()

    rospy.init_node("MiR_python")

    pos_X = args.x
    pos_Y = args.y
    pos_th = args.th

    base = BaseClient()
    base.move(pos_X, pos_Y, pos_th)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


