# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Denis Štogl, Lovro Ivanov
#

from multiprocessing.connection import wait
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
import time

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

# import sys
# # adding gripper folder to the system path
# sys.path.insert(0, '/home/kendrick/gripper_workspace')
import ur_publisher.robotiq_gripper as robotiq_gripper



class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("rpl_ur_move")
        # Declare all parameters
        self.declare_parameter("controller_name", "position_trajectory_controller")
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("gripper_close_pos", 255)
        self.declare_parameter("joints")
        self.declare_parameter("check_starting_point", False)
        self.declare_parameter("starting_point_limits")

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        goal_names = self.get_parameter("goal_names").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}
        self.gripper_close_pos = self.get_parameter("gripper_close_pos").value
        self.griper_open_pose = 0

        if self.joints is None or len(self.joints) == 0:
            raise Exception('"joints" parameter is not set!')

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise Exception('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(
                JointState, "joint_states", self.joint_state_callback, 10
            )
        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = []
        for name in goal_names:
            self.declare_parameter(name)
            goal = self.get_parameter(name).value
            if goal is None or len(goal) == 0:
                raise Exception(f'Values for goal "{name}" not set!')

            float_goal = []
            for value in goal:
                float_goal.append(float(value))
            self.goals.append(float_goal)

        # Hardcoded gripper info
        ur_robot_ip = "192.168.1.102" 
        self.get_logger().info(
                'Creating gripper...'
            )
        self.gripper = robotiq_gripper.RobotiqGripper()
        self.get_logger().info(
            'Connecting to gripper...'
        )
        self.gripper.connect(ur_robot_ip, 63352)
        self.get_logger().info(
            'Activating gripper...'
        )
        self.gripper.activate()
        self.get_logger().info(
            'Opening gripper...'
        )
        self.gripper.move_and_wait_for_pos(0, 255, 255)


        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            'Publishing pick up from [{}] and put down at [{}] goals on topic "{}"'.format(
                ','.join([str(n) for n in self.goals[0]]), ','.join([str(n) for n in self.goals[1]]), publish_topic
            )
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.positions_published = False
        if not self.check_starting_point:
            self.pick_up_and_put_down()


    def create_trajectory(self, goal):
        '''Creates a new trajectory with a given goal'''
        traj = JointTrajectory()
        traj.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = goal
        point.time_from_start = Duration(sec=4) # Can change trajectory duration here
        traj.points.append(point)
        return traj


    def change_gripper_at_pos(self, goal, new_gripper_pos):
        '''Publish trajectories to move to above goal, move down to goal, move to new gripper position, and move back to above goal'''
        # Publish above pick up position
        above_goal_pos = self.create_trajectory([-1.57,-1.03,-2.08,-1.60,1.57,0.0]) #TODO: get position + certain amount above

        self.get_logger().info(
            'Publishing above goal position'
        )
        self.publisher_.publish(above_goal_pos)

        time.sleep(4)


        # Publish pick up position
        goal_pos = self.create_trajectory(goal)

        self.get_logger().info(
            'Publishing goal position'
        )
        self.publisher_.publish(goal_pos)

        time.sleep(4)


        # MOVE GRIPPER HERE
        self.get_logger().info(
            'Moving gripper...'
        )
        self.gripper.move_and_wait_for_pos(new_gripper_pos, 255, 255)

        self.get_logger().info(
            'Publishing above goal position'
        )
        self.publisher_.publish(above_goal_pos)

        time.sleep(4)

        #move to neutral position here?


    def pick_up(self):
        '''Pick up from first goal position'''
        self.change_gripper_at_pos(self.goals[0], self.gripper_close_pos)
    

    def put_down(self):
        '''Put down at second goal position'''
        self.change_gripper_at_pos(self.goals[1], self.griper_open_pose)


    def pick_up_and_put_down(self):
        '''Pick up from first position and put down at second position'''
        self.positions_published = True
        if self.starting_point_ok:
            self.pick_up()
            self.put_down()
            self.get_logger().info(
                'Finished publishing'
            )
        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warn("Start configuration is not within configured limits!")


    def joint_state_callback(self, msg):
        self.get_logger().info(
            'Entering Callback'
        )
        if not self.joint_state_msg_received:
            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            if not self.positions_published:
                self.pick_up_and_put_down()
            return


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
