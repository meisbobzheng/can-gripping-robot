import VoiceController as vc
import SodaController as sc
import MovementController as mc
import VisionController as visc
import numpy as np
import time


class RobotMain:
    """
    Main class for the robot. Contains the main loop and the main logic.
    """

    def __init__(self):
        # voice stuff
        self.voice_controller = vc.VoiceController()
        print("Voice controller initialized.")

        # vision stuff
        self.vision_controller = visc.VisionController()
        print("Vision controller initialized.")

        # soda stuff
        self.soda_controller = sc.SodaController()
        self.soda_list = self.soda_controller.get_soda_list()
        print("Soda controller initialized.")
        print("Soda list:", self.soda_list)
        self.curr_soda = ""

        # Movement
        self.movement_controller = mc.MovementController()

        # Number of sections to check
        # should be 13?
        self.num_sections = 13
        self.section_rotate_dist = np.pi / 8

    # first step
    def listen_and_identify_soda(self):
        print("Robot is listening for initial command...")

        self.voice_controller.listen_for_initial_command("hey robot")

        self.movement_controller.startup()
        self.movement_controller.scout_pos()

        soda = self.voice_controller.listen_for_soda(
            self.soda_list
        )

        if soda is None:
            print("Error when listening for initial command.")
            raise Exception("Error when listening for initial command.")
        else:
            print(f"Robot is picking up {soda}")
            self.curr_soda = soda
            return soda

    def scan_sections(self):
        print("Beginning scan sections")
        waist_index = self.movement_controller.bot.arm.info_index_map["waist"]
        i = 0
        while i < self.num_sections:

            self.vision_controller.get_can_position(self.curr_soda, i)

            if i != self.num_sections - 1:
                # Rotate to next section (Move 1/8 left for now)
                print("going to next section")

                curr_joints = self.movement_controller.bot.arm.get_joint_positions()
                curr_waist_pos = curr_joints[waist_index]
                self.movement_controller.rotate_waist(
                    curr_waist_pos + (self.section_rotate_dist)
                )
            i += 1

    def get_max_confidence_section(self, tuple_list):
        return max(tuple_list, key=lambda x: x[0])[1]

    def main(self):

        # main loop
        try:

            while True:
                # Step 1: listen for initial command get soda
                soda = self.listen_and_identify_soda()
                soda_command = self.soda_controller.get_soda_command(soda)
                self.curr_soda = soda_command
                print(self.curr_soda)
                break

            # self.curr_soda = self.soda_controller.get_soda_command("sprite")

            # go to scout

            # Start at back right section
            start_rotation = 3 * -np.pi / 4
            self.movement_controller.rotate_waist(start_rotation)

            # rotate and find can in quadrants
            self.scan_sections()
            print("Cans observed:")
            print(self.vision_controller.cans_observed)

            # We've observed all cans now, pick the max
            section = self.get_max_confidence_section(
                self.vision_controller.cans_observed
            )

            # Go to the section with max confidence:

            print("SECTION TO GO TO", section)
            section_waist_angle = start_rotation + (section * self.section_rotate_dist)
            self.movement_controller.rotate_waist(section_waist_angle)

            # Once at section, center frame and grab

            # positive is left
            # negative is right

            adjustment = self.vision_controller.center_the_frame(self.curr_soda)
            adjustment = -adjustment
            # 12% -> 30                adjustment = -adjustment  # its flipped

            # pi * 2.5 * 12 / 180

            # 15 % -> 40

            adjust_factor = 0.2
            radian_adjustment = section_waist_angle + (
                np.pi * adjust_factor * adjustment / 180
            )

            print("Waist angle and then radian adjustment")
            print(section_waist_angle)
            print(radian_adjustment)

            self.movement_controller.rotate_waist(radian_adjustment)

            # Max should be around .4, test it a bit and set that as max with this

            estimated_distance = self.vision_controller.estimate_distance_to_can(
                self.curr_soda
            )

            print("Estimated distance", estimated_distance)
            distance_scale_factor = 8

            # extend and grip can

            # Maybe add a little to this so it overshoots
            distance_to_move = estimated_distance / distance_scale_factor

            distance_to_move = min(0.4, distance_to_move)
            self.movement_controller.bot.arm.set_ee_cartesian_trajectory(
                x=distance_to_move, moving_time=2
            )
            time.sleep(4)

            self.movement_controller.grip()

            joints = [0, 0, 0, 0, 0, 0]
            waist_index = self.movement_controller.bot.arm.info_index_map["waist"]
            joints[waist_index] = radian_adjustment

            self.movement_controller.bot.arm.set_joint_positions(joints)
            self.movement_controller.return_to_home()

            self.movement_controller.wait_for_drop()

            self.movement_controller.bot.arm.go_to_sleep_pose()

            self.movement_controller.shutdown()

        except Exception as e:
            print(e)
        return

        # find cans
        # find cans

        # move robot towards the correct can

        # pick up can

        # return to start position

        # extend arm

        # listen for drop command

        # release can and return to start
        # move robot towards the correct can

        # pick up can

        # return to start position

        # extend arm

        # listen for drop command

        # release can and return to start


if __name__ == "__main__":
    robot = RobotMain()
    robot.main()


"""
This script makes the end-effector go to a specific pose by defining the pose components

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250

Then change to this directory and type:

    python3 ee_pose_components.py
"""
