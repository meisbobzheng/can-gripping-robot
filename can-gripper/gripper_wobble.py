from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time

import MovementController as mc


if __name__ == "__main__":

    controller = mc.MovementController()

    controller.startup()

    # controller.bot.arm.go_to_home_pose()
    # time.sleep(1)

    # controller.bot.arm.set_single_joint_position("wrist_angle", -np.pi / 6)

    controller.bot.arm.go_to_sleep_pose()

    controller.scout_pos()

    controller.bot.arm.set_ee_cartesian_trajectory(x=0.3, moving_time=2, z=0.02)
    time.sleep(4)

    controller.bot.arm.go_to_sleep_pose()

    # controller.bot.gripper.set_pressure(0.9)
    # controller.bot.gripper.grasp()
    # controller.wait_for_drop()

    # controller.bot.arm.go_to_sleep_pose()
    controller.shutdown()
