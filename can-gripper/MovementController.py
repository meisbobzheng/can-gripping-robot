from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import math

class MovementController:
    """
    Controller for the robot arm, and all movement. 
    """
    def __init__(self):
        self.bot = InterbotixManipulatorXS(
            robot_model="wx250",
            group_name="arm",
            gripper_name="gripper",)
        
    def startup(self) -> None:
        robot_startup()
        self.bot.arm.go_to_home_pose()
        print("robot moved to home")

    def shutdown(self) -> None:
        self.bot.arm.go_to_home_pose()
        robot_shutdown()
        print("robot is going to sleep")


    # Extneds the arm forward to (kind of the distance, not sure units)
    def extend_arm_forward(distance):
        pass

    def extend(self):
        # full extension
        pass

    # Roughly moves to a position x,y
    def move_to(self, x, y ,z):
        pass

    def pick_up_can(self):
        pass

    def return_to_home(self):
        pass

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

