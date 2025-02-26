#!/bin/bash

# This script sets up the necessary files for the WX200 robot control
# Run this after the container is built and running

# Create directories if they don't exist
mkdir -p src/wx200_robot/wx200_control/scripts

# Copy the Python control script
cat > src/wx200_robot/wx200_control/scripts/wx200_control.py << 'EOL'
#!/usr/bin/env python3
"""
Example script for controlling the WX200 robot arm using ROS and Python.
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

class WX200Controller:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('wx200_controller_node', anonymous=True)
        
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
        controller = WX200Controller()
        
        # Demonstrate some capabilities
        controller.home_position()
        
        # Run a pick and place demo
        controller.pick_and_place_demo()
        
    except rospy.ROSInterruptException:
        print("Program interrupted")
        pass
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
EOL

# Make the script executable
chmod +x src/wx200_robot/wx200_control/scripts/wx200_control.py

# Create package.xml
cat > src/wx200_robot/wx200_control/package.xml << 'EOL'
<?xml version="1.0"?>
<package format="2">
  <name>wx200_control</name>
  <version>0.1.0</version>
  <description>Control package for WX200 robot arm</description>
  <maintainer email="user@example.com">Your Name</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
</package>
EOL

# Create CMakeLists.txt
cat > src/wx200_robot/wx200_control/CMakeLists.txt << 'EOL'
cmake_minimum_required(VERSION 3.0.2)
project(wx200_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/wx200_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
EOL

echo "Setup completed! Files are ready in src/wx200_robot directory."