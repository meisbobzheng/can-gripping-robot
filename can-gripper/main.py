import VoiceController as vc
import SodaController as sc

class RobotMain:
    """
    Main class for the robot. Contains the main loop and the main logic.
    """

    def __init__(self):
        # voice stuff
        self.voice_controller = vc.VoiceController()
        # soda stuff
        self.soda_controller = sc.SodaController()
        self.soda_list = self.soda_controller.get_soda_list()
        print(self.soda_list)

    def __str__(self):
        return f"RobotMain({self.soda_list})"
    
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

    print(robot)
    robot.main()
