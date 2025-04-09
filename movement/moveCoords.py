#!/usr/bin/env python3

# Copyright 2024 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import math

"""
This script makes the end-effector go to a specific pose by defining the pose components

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250

Then change to this directory and type:

    python3 ee_pose_components.py
"""


# calculates the position the arm should go to before moving forward to grab can
def calculate_first_pos(can_x, can_y):
    # robot origin is (0,0,~.2)

    # y = mx + b

    # (0,0)
    # m = can_y / can_x
    # b = 0
    # y = can_y / can_x

    # percentage of x to move to (Can be adjusted if we start hitting cans)
    percent_x = 0.75
    percent_x_pos = percent_x * can_x

    pos_y = (can_y / can_x) * percent_x_pos + 0
    print("HELLOOOO")
    print(percent_x_pos)
    print(pos_y)
    return percent_x_pos, pos_y


def scout_pos():
    x_pos = 0.5
    y_pos = 0.0
    z_pos = 0.2

    # first_x_pos, first_y_pos = calculate_first_pos(x_pos, y_pos)

    bot = InterbotixManipulatorXS(
        robot_model="wx250",
        group_name="arm",
        gripper_name="gripper",
    )

    robot_startup()

    # bot.arm.go_to_home_pose()

    # first_pos = bot.arm.set_ee_pose_components(x=x_pos, y=y_pos, z=z_pos, execute=True)
    # curr_joints = first_pos[0]

    # adjust shoulder and elbow to move end effector straight down
    shoulder_index = bot.arm.info_index_map["shoulder"]
    elbow_index = bot.arm.info_index_map["elbow"]
    wrist_rotate_index = bot.arm.info_index_map["wrist_rotate"]
    wrist_angle_index = bot.arm.info_index_map["wrist_angle"]

    # curr_joints[shoulder_index] = math.pi / 4.4
    # curr_joints[elbow_index] = math.pi / 4

    # flatten wrist to be parallel
    # wrist angle to look up at - maybe calculate, maybe can just cheat and hard code
    wrist_rotate_angle = math.pi / 2
    wrist_angle = -math.pi / 70
    # - twists it right

    # curr_joints[wrist_rotate_index] = wrist_rotate_angle
    # curr_joints[wrist_angle_index] = wrist_angle
    bot.arm.go_to_sleep_pose()

    scout_joints_from_sleep = bot.arm.get_joint_positions()
    scout_joints_from_sleep[wrist_rotate_index] = wrist_rotate_angle
    scout_joints_from_sleep[wrist_angle_index] = wrist_angle

    scout_joints_from_sleep[elbow_index] = math.pi / 2.5
    scout_joints_from_sleep[shoulder_index] = math.pi / 8
    print(scout_joints_from_sleep)
    bot.arm.set_joint_positions(scout_joints_from_sleep)
    print("scouting")

    bot.arm.set_single_joint_position("waist", math.pi / 2)
    # extend wrist outwrist_rotate_angle

    bot.arm.go_to_sleep_pose()
    robot_shutdown()


def main():

    # These will be an input from the camera telling us where the can is
    x_pos = 0.5
    y_pos = 0.1
    z_pos = 0.2

    first_x_pos, first_y_pos = calculate_first_pos(x_pos, y_pos)

    bot = InterbotixManipulatorXS(
        robot_model="wx250",
        group_name="arm",
        gripper_name="gripper",
    )

    robot_startup()

    # wrist angle to look up at - maybe calculate, maybe can just cheat and hard code
    wrist_rotate_angle = math.pi / 4

    wrist_angle = math.pi / 5

    bot.arm.go_to_home_pose()

    # bot.arm.set_ee_cartesian_trajectory(x_pos, y_pos, z_pos)
    bot.arm.set_ee_pose_components(x=first_x_pos, y=first_y_pos, z=z_pos)

    final_pos = bot.arm.set_ee_pose_components(x=x_pos, y=y_pos, z=z_pos, execute=False)
    # bot.arm.set_single_joint_position("wrist_rotate", wrist_rotate_angle)
    # bot.arm.set_single_joint_position("wrist_angle", wrist_angle)

    # curr_joints = list(bot.arm.get_joint_commands())
    curr_joints = final_pos[0]

    # adjust shoulder and elbow to move end effector straight down
    shoulder_index = bot.arm.info_index_map["shoulder"]
    elbow_index = bot.arm.info_index_map["elbow"]
    wrist_rotate_index = bot.arm.info_index_map["wrist_rotate"]
    wrist_angle_index = bot.arm.info_index_map["wrist_angle"]

    curr_joints[shoulder_index] = math.pi / 4.4
    # curr_joints[elbow_index] = math.pi / 4
    curr_joints[wrist_rotate_index] = wrist_rotate_angle
    curr_joints[wrist_angle_index] = wrist_angle

    bot.arm.set_joint_positions(curr_joints)

    # joint_angles = bot.arm.set_ee_pose_components(x=x_pos, y=y_pos, z=z_pos)
    # print(joint_angles[0])wrist_rotate
    # bot.arm.go_to_home_pose()
    # bot.arm.go_to_sleep_pose()

    robot_shutdown()


if __name__ == "__main__":
    scout_pos()
