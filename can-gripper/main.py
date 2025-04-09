import VoiceController as vc
import SodaController as sc
#import VisionController as visc

class RobotMain:
    """
    Main class for the robot. Contains the main loop and the main logic.
    """

    def __init__(self):
        # voice stuff
        self.voice_controller = vc.VoiceController()
        print("Voice controller initialized.")

        # vision stuff
        # self.vision_controller = visc.VisionController()
        # print("Vision controller initialized.")

        # soda stuff
        self.soda_controller = sc.SodaController()
        self.soda_list = self.soda_controller.get_soda_list()
        print("Soda controller initialized.")
        print("Soda list:", self.soda_list)
    
    # first step
    def listen_and_identify_soda(self):
        print("Robot is listening for initial command...")

        soda = self.voice_controller.listen_for_initial_command("hey robot", self.soda_list)

        if soda is None:
            print("Error when listening for initial command.")
            raise Exception("Error when listening for initial command.")
        else:
            print(f"Robot is picking up {soda}")
            return soda
    
    def main(self):

        # main loop
        while True:
            try :
                # Step 1: listen for initial command get soda
                soda = self.listen_and_identify_soda()


            except Exception as e:
                print(e)
                continue
            return

            # find cans

            # move robot towards the correct can

            # pick up can

            # return to start position

            # extend arm 

            # listen for drop command

            # release can and return to start


if __name__ == '__main__':
    robot = RobotMain()
    robot.main()


"""
This script makes the end-effector go to a specific pose by defining the pose components

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250

Then change to this directory and type:

    python3 ee_pose_components.py
"""
