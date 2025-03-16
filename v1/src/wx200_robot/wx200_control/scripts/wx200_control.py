#!/usr/bin/env python3
"""
Example script for controlling the WX250 robot arm using ROS and Python.
This script should be run inside the Docker container.

USAGE:
  - Make sure to run 'roscore' in a separate terminal before running this script
  - Press Ctrl+C at any time to safely exit
"""

import rospy
import sys
import time
import signal
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# Setup signal handler for clean exits
def signal_handler(sig, frame):
    print("\nReceived shutdown signal. Exiting gracefully...")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class WX250Controller:
    def __init__(self):
        try:
            # Initialize the ROS node with a timeout
            rospy.init_node('wx250_controller_node', anonymous=True)
            
            # Check if ROS master is running, with a timeout
            if not self._wait_for_master(timeout=5):
                print("ERROR: ROS Master not running. Start it with 'roscore' in another terminal.")
                print("Press Ctrl+C to exit.")
                return
            
            # Set up publishers for joint control
            # These topics may need adjustment for your specific WX250 ROS driver
            self.waist_pub = rospy.Publisher('/wx250/waist_controller/command', Float64, queue_size=10)
            self.shoulder_pub = rospy.Publisher('/wx250/shoulder_controller/command', Float64, queue_size=10)
            self.elbow_pub = rospy.Publisher('/wx250/elbow_controller/command', Float64, queue_size=10)
            self.wrist_angle_pub = rospy.Publisher('/wx250/wrist_angle_controller/command', Float64, queue_size=10)
            self.wrist_rotate_pub = rospy.Publisher('/wx250/wrist_rotate_controller/command', Float64, queue_size=10)
            self.gripper_pub = rospy.Publisher('/wx250/gripper_controller/command', Float64, queue_size=10)
            
            # Set up a subscriber to get current joint states
            self.joint_states = None
            rospy.Subscriber('/wx250/joint_states', JointState, self.joint_states_callback)
            
            # Wait for connections to be established
            rospy.sleep(1)
            
            print("WX250 controller initialized")
            self.initialized = True
        except rospy.ROSInitException as e:
            print(f"Error initializing ROS node: {e}")
            print("Press Ctrl+C to exit.")
            self.initialized = False
        except KeyboardInterrupt:
            print("\nExiting due to keyboard interrupt.")
            sys.exit(0)
            
    def _wait_for_master(self, timeout=5):
        """Wait for ROS master with timeout"""
        try:
            start_time = time.time()
            while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
                master = rospy.get_master()
                print("master uri: " , master[0])
                print("master port:" , master[1])
                if rospy.core.is_initialized():
                    return True
                print("Waiting for ROS master... Press Ctrl+C to exit.")
                time.sleep(1)
            return False
        except KeyboardInterrupt:
            print("\nExiting due to keyboard interrupt.")
            sys.exit(0)

    def joint_states_callback(self, data):
        """Callback function for joint state subscriber"""
        self.joint_states = data

    def move_to_joint_position(self, positions):
        """
        Move the robot to specified joint positions
        
        Args:
            positions (list): List of 6 joint angles in radians 
                             [waist, shoulder, elbow, wrist_angle, wrist_rotate, gripper]
        """
        if len(positions) != 6:
            rospy.logerr("Expected 6 joint positions")
            return
        
        # Publish each joint position
        self.waist_pub.publish(Float64(positions[0]))
        self.shoulder_pub.publish(Float64(positions[1]))
        self.elbow_pub.publish(Float64(positions[2]))
        self.wrist_angle_pub.publish(Float64(positions[3]))
        self.wrist_rotate_pub.publish(Float64(positions[4]))
        self.gripper_pub.publish(Float64(positions[5]))
        
        # Give the robot time to move
        rospy.sleep(2)
        
        print(f"Moved to joint positions: {positions}")

    def open_gripper(self):
        """Open the gripper"""
        self.gripper_pub.publish(Float64(2.0))  # Adjust value as needed for your gripper
        rospy.sleep(1)
        print("Gripper opened")

    def close_gripper(self):
        """Close the gripper"""
        self.gripper_pub.publish(Float64(0.0))  # Adjust value as needed for your gripper
        rospy.sleep(1)
        print("Gripper closed")

    def home_position(self):
        """Move the robot to a predefined home position"""
        # WX250 home position - adjust these values based on your setup
        home_joints = [0.0, -1.5, 1.5, 0.0, 0.0, 0.0]
        self.move_to_joint_position(home_joints)
        print("Robot moved to home position")

    def sleep_position(self):
        """Move the robot to a safe position for storage"""
        # WX250 sleep/storage position - adjust as needed
        sleep_joints = [0.0, -1.9, 1.55, 0.8, 0.0, 0.0]
        self.move_to_joint_position(sleep_joints)
        print("Robot moved to sleep position")

    def pick_and_place_demo(self):
        """Demonstrate a simple pick and place operation"""
        # Move to approach position
        approach_pos = [0.0, -0.8, 1.0, 0.5, 0.0, 2.0]  # Gripper open
        self.move_to_joint_position(approach_pos)
        
        # Move to pick position
        pick_pos = [0.0, -0.5, 0.5, 0.5, 0.0, 2.0]
        self.move_to_joint_position(pick_pos)
        
        # Close gripper
        self.close_gripper()
        
        # Move back to approach position
        self.move_to_joint_position(approach_pos)
        
        # Move to place position
        place_pos = [1.5, -0.8, 1.0, 0.5, 0.0, 0.0]
        self.move_to_joint_position(place_pos)
        
        # Open gripper
        self.open_gripper()
        
        # Move back to home
        self.home_position()
        
        print("Pick and place demo completed")

def main():
    try:
        # Add timeout for the entire script
        print("Starting WX250 controller...")
        print("Press Ctrl+C at any time to exit safely.")
        
        controller = WX250Controller()
        
        # Only proceed if initialization was successful
        if hasattr(controller, 'initialized') and controller.initialized:
            print("\nRunning demo sequence. Press Ctrl+C to cancel.")
            
            # Demonstrate some capabilities
            controller.home_position()
            
            # Run a pick and place demo
            controller.pick_and_place_demo()
            
            # Return to sleep position
            controller.sleep_position()
        else:
            print("\nController initialization failed. Cannot proceed.")
            print("Make sure ROS master is running with 'roscore' in another terminal.")
            print("Exiting in 5 seconds...")
            time.sleep(5)
        
    except rospy.ROSInterruptException:
        print("\nProgram interrupted by ROS.")
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting gracefully.")
    except Exception as e:
        print(f"\nError: {e}")
    
    print("Program terminated. You can now exit safely.")

if __name__ == "__main__":
    main()