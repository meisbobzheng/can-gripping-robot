#!/usr/bin/env python3
"""
Example script for controlling the WX200 robot arm using ROS and Python.
This script should be run inside the Docker container.
"""

import rospy
import sys
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class WX200Controller:
    def __init__(self):
        try:
            # Initialize the ROS node with a timeout
            rospy.init_node('wx200_controller_node', anonymous=True)
            
            # Check if ROS master is running, with a timeout
            if not self._wait_for_master(timeout=5):
                print("ERROR: ROS Master not running. Start it with 'roscore' in another terminal.")
                print("Press Ctrl+C to exit.")
                return
            
            # Set up publishers for joint control
            # These topics need to match what your WX200 ROS driver expects
            self.joint1_pub = rospy.Publisher('/wx200/joint1_position_controller/command', Float64, queue_size=10)
            self.joint2_pub = rospy.Publisher('/wx200/joint2_position_controller/command', Float64, queue_size=10)
            self.joint3_pub = rospy.Publisher('/wx200/joint3_position_controller/command', Float64, queue_size=10)
            self.joint4_pub = rospy.Publisher('/wx200/joint4_position_controller/command', Float64, queue_size=10)
            self.joint5_pub = rospy.Publisher('/wx200/joint5_position_controller/command', Float64, queue_size=10)
            self.gripper_pub = rospy.Publisher('/wx200/gripper_position_controller/command', Float64, queue_size=10)
            
            # Set up a subscriber to get current joint states
            self.joint_states = None
            rospy.Subscriber('/wx200/joint_states', JointState, self.joint_states_callback)
            
            # Wait for connections to be established
            rospy.sleep(1)
            
            print("WX200 controller initialized")
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
            positions (list): List of 6 joint angles in radians [joint1, joint2, joint3, joint4, joint5, gripper]
        """
        if len(positions) != 6:
            rospy.logerr("Expected 6 joint positions")
            return
        
        # Publish each joint position
        self.joint1_pub.publish(Float64(positions[0]))
        self.joint2_pub.publish(Float64(positions[1]))
        self.joint3_pub.publish(Float64(positions[2]))
        self.joint4_pub.publish(Float64(positions[3]))
        self.joint5_pub.publish(Float64(positions[4]))
        self.gripper_pub.publish(Float64(positions[5]))
        
        # Give the robot time to move
        rospy.sleep(2)
        
        print(f"Moved to joint positions: {positions}")

    def open_gripper(self):
        """Open the gripper"""
        self.gripper_pub.publish(Float64(0.0))  # Adjust value as needed for your gripper
        rospy.sleep(1)
        print("Gripper opened")

    def close_gripper(self):
        """Close the gripper"""
        self.gripper_pub.publish(Float64(1.0))  # Adjust value as needed for your gripper
        rospy.sleep(1)
        print("Gripper closed")

    def home_position(self):
        """Move the robot to a predefined home position"""
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Adjust these values for your robot
        self.move_to_joint_position(home_joints)
        print("Robot moved to home position")

    def pick_and_place_demo(self):
        """Demonstrate a simple pick and place operation"""
        # Move to approach position
        approach_pos = [0.0, 0.5, 0.5, 0.0, 0.3, 0.0]
        self.move_to_joint_position(approach_pos)
        
        # Open gripper
        self.open_gripper()
        
        # Move to pick position
        pick_pos = [0.0, 0.7, 0.7, 0.0, 0.3, 0.0]
        self.move_to_joint_position(pick_pos)
        
        # Close gripper
        self.close_gripper()
        
        # Move back to approach position
        self.move_to_joint_position(approach_pos)
        
        # Move to place position
        place_pos = [1.0, 0.5, 0.5, 0.0, 0.3, 0.0]
        self.move_to_joint_position(place_pos)
        
        # Open gripper
        self.open_gripper()
        
        # Move back to home
        self.home_position()
        
        print("Pick and place demo completed")

def main():
    try:
        # Add timeout for the entire script
        print("Starting WX200 controller...")
        print("Press Ctrl+C at any time to exit safely.")
        
        controller = WX200Controller()
        
        # Only proceed if initialization was successful
        if hasattr(controller, 'initialized') and controller.initialized:
            print("\nRunning demo sequence. Press Ctrl+C to cancel.")
            
            # Demonstrate some capabilities
            controller.home_position()
            
            # Run a pick and place demo
            controller.pick_and_place_demo()
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