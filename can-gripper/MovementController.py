from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_xs_msgs.srv import RegisterValues, RegisterValuesRequest
import math
import time


class MovementController:
    """
    Controller for the robot arm, and all movement.
    """

    def __init__(self):
        self.bot = InterbotixManipulatorXS(
            robot_model="wx250s",
            group_name="arm",
            gripper_name="gripper",
        )

    def startup(self) -> None:
        robot_startup()
        print("robot moved to home")

    def shutdown(self) -> None:
        robot_shutdown()
        print("robot is going to sleep") 

    def scout_pos(self):
        self.bot.arm.go_to_sleep_pose()
        self.bot.gripper.release()
        self.bot.arm.set_ee_pose_components(x=0.27, z=0.07)

    ## DONT USE THIS FUNCTION
    def extend(self, distance):
        # roughly .1 to .4 is pretty good we can test
        self.bot.arm.set_ee_cartesian_trajectory(x=distance, moving_time=2)
        time.sleep(3)
        # self.bot.gripper.grasp()
        # self.bot.arm.go_to_home_pose()

    # Rotate waist by an angle (radian)
    # positive is left, negative is right
    def rotate_waist(self, angle):
        self.bot.arm.set_single_joint_position("waist", angle)

    def grip(self):
        self.bot.gripper.set_pressure(0.9)
        self.bot.gripper.grasp()

    def release(self):
        self.bot.gripper.release()
        
    # Roughly moves to a position x,y
    def move_to(self, x, y, z):
        pass

    def pick_up_can(self):
        pass

    def return_to_home(self):
        self.bot.arm.go_to_home_pose()

    def scan_for_cans(self):
        pass

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
        return percent_x_pos, pos_y
    
    def wait_for_drop(self) -> bool :
        # sleep 1 second to let stabilize
        time.sleep(1)

        start_effort = self.bot.arm.get_joint_efforts()[0]

        last_effort = 0
        max_effort = 0

        start_time = time.time()
        
        while (True):
            if time.time() - start_time > 10:
                self.bot.gripper.release()
                return False  # Timed out
            
            joint_efforts = self.bot.arm.get_joint_efforts()
            current_effort = abs(joint_efforts[0])

            if (current_effort - last_effort > 30):
                self.bot.gripper.release()
                return True
            
            if (current_effort > max_effort):
                max_effort = current_effort

            if (max_effort - start_effort > 50):
                self.bot.gripper.release()
                return True
            
            last_effort = current_effort
            

