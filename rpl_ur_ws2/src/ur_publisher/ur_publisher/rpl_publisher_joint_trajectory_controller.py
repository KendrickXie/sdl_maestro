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
# import robotiq_gripper



class PublisherJointTrajectory(Node):
    def __init__(self):
        super().__init__("rpl_publisher_position_trajectory_controller")
        # Declare all parameters
        self.declare_parameter("controller_name", "position_trajectory_controller")
        self.declare_parameter("wait_sec_between_publish", [1,1])
        self.declare_parameter("goal_names", ["pos1", "pos2"])
        self.declare_parameter("gripper_pos", [0, 0])
        self.declare_parameter("joints")
        self.declare_parameter("check_starting_point", False)
        self.declare_parameter("starting_point_limits")

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        self.wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value
        #self.gripper_pos = self.get_parameter("gripper_pos").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}

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

        # # Hardcoded gripper info
        # ur_robot_ip = "192.168.1.102" 
        # self.get_logger().info(
        #         'Creating gripper...'
        #     )
        # self.gripper = robotiq_gripper.RobotiqGripper()
        # self.get_logger().info(
        #     'Connecting to gripper...'
        # )
        # self.gripper.connect(ur_robot_ip, 63352)
        # self.get_logger().info(
        #     'Activating gripper...'
        # )
        # self.gripper.activate()
        # self.get_logger().info(
        #     'Opening gripper...'
        # )
        # self.gripper.move_and_wait_for_pos(0, 255, 255)


        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            'Publishing {} goals on topic "{}" every {} s'.format(
                len(goal_names), publish_topic, ','.join([str(n) for n in self.wait_sec_between_publish])
            )
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)
        self.positions_published = False


    def publish_positions(self):
        self.positions_published = True
        if self.starting_point_ok:
            for i, wait_time in enumerate(self.wait_sec_between_publish):
                traj = JointTrajectory()
                traj.joint_names = self.joints
                point = JointTrajectoryPoint()
                point.positions = self.goals[i]
                point.time_from_start = Duration(sec=4)

                traj.points.append(point)
                
                self.get_logger().info(
                    'Publishing position #{}'.format(
                        (i+1)
                    )
                )

                self.publisher_.publish(traj)

                self.get_logger().info(
                    'Publishing next position in {} s'.format(
                        wait_time
                    )
                )

                time.sleep(wait_time)
                # if i==0 or self.gripper_pos[i] != self.gripper_pos[i-1]:
                #     self.get_logger().info(
                #         'Moving gripper...'
                #     )
                #     self.gripper.move_and_wait_for_pos(self.gripper_pos[i], 255, 255)

                i += 1
                i %= len(self.goals)

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
                self.publish_positions()
            return


def main(args=None):
    rclpy.init(args=args)

    publisher_joint_trajectory = PublisherJointTrajectory()

    rclpy.spin(publisher_joint_trajectory)
    publisher_joint_trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
