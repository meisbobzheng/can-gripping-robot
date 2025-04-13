from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time


def grip():
    bot.gripper.set_pressure(0.9)
    bot.gripper.grasp()


if __name__ == "__main__":

    bot = InterbotixManipulatorXS(
        robot_model="wx250s",
        group_name="arm",
        gripper_name="gripper",
    )

    robot_startup()

    bot.arm.go_to_home_pose()

    bot.arm.grip()

    # Try to detect wobble

    while True:
        print("Finger pos: ", bot.gripper.get_finger_position())
        print('--------------------------------------------')
        print("Effort: ", bot.gripper.get_gripper_effort())
        print('--------------------------------------------')
        print("gripper pos: ", bot.gripper.get_gripper_position())
        print('--------------------------------------------')
        print("gripper vel: ", bot.gripper.get_gripper_velocity())
        print('--------------------------------------------')
        print("joint efforts: ", bot.arm.get_joint_efforts())
        print('--------------------------------------------')
        print("joint pos: ", bot.arm.get_joint_positions())
        print('--------------------------------------------')

        time.sleep(1)



    bot.gripper.

    robot_shutdown()
