from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import cv2 as cv
from multiprocessing import Process
import VisionControllerCopy


def new_scout_pos():
    bot.arm.go_to_sleep_pose()
    bot.gripper.release()
    bot.arm.set_ee_pose_components(x=0.27, z=0.07)

    # bot.arm.set_single_joint_position(joint_name="waist", position=np.pi / 2.0)
    # bot.gripper.release()name="waist", position=np.pi / 2.0)
    # bot.gripper.release()


def new_extend(distance):
    # roughly .1 to .4 is pretty good we can test
    bot.arm.set_ee_cartesian_trajectory(x=distance)


# Rotate waist by an angle (radian)
# positive is left, negative is right
def rotate_waist(angle):
    bot.arm.set_single_joint_position("waist", angle)


def grip():
    bot.gripper.set_pressure(0.9)
    bot.gripper.grasp()


# Starts recording from the camera
def camera_start():
    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("Cannot open camera")
        exit()
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        cv.imshow("frame", frame)
        if cv.waitKey(1) == ord("q"):
            break

    cap.release()
    cv.destroyAllWindows()


def robot_total():
    robot_startup()
    new_scout_pos()
    new_extend(0.2)
    grip()
    robot_shutdown()


def vision_control():
    vision_controller = VisionControllerCopy.VisionController()

    # get_can_position crashes my vscode
    # vision_controller.get_can_position("a green sprite can", 1)

    # vision_controller.center_the_frame("a green sprite can")


if __name__ == "__main__":

    bot = InterbotixManipulatorXS(
        robot_model="wx250s",
        group_name="arm",
        gripper_name="gripper",
    )

    # Need multiprocessing so camera and robot can run at same time

    # robot_total()
    while True:

        # Go to scout
        new_scout_pos()

        # Start camera
        camera_start()
