ROBOT X Y Z MAX COORDS AND STUFF

Range for all coords: -.75 to .75

+X is right of robot ?
+Y is forward of robot ?
+Z is above robot

Z = .2 is a good height off ground to be grabbing a can.
If Z is less than .2, arm will probably go into the ground since 0 axis is roughly at the floor

X,Y .5 range around the robot would be a good maximum range for cans to be placed on table ?

Base: (0,0,0) coordinates is roughly where the "-" is in the name on the robot


### Getting Environment Setup:

1. Create Ubuntu 22.04 VM in VMWare Fusion (if on Mac)
2. Install Ros on VM 
3. Follow this video to intstall interbotix:
   https://www.youtube.com/watch?v=kZx2tNVfQAQ&list=PL8X3t2QTE54sMTCF59t0pTFXgAmdf0Y9t&index=5&ab_channel=TrossenRobotics
   - When installing interbotix, I had an error about dynamixel, and had to run this command, and then install again:
      - sudo apt install ros-humble-dynamixel-sdk
4. You will now have an interbotix_ws directory on your VM containing a bunch of useful files and stuff

### To connect to Robot:
1. ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250
   - Type in "wx250" in name in RViZ and select "arm" from the drop down
   - Should now be able to go into sleep or home pose
   - ** Ensure USB connection is going through to VM, and may need to link ttyDXL to USB file if it isn't properly recognized:
      -To do so, go to /dev directory and run: ln -s ttyUSB0 ttyDXL
2. Run any of the scripts in demos.py (Be careful of bartender.py it makes the arm move real fast), and they should work

### Useful stuff
 This github has most of the relevant code to moving the robot
 set_ee_pose(x,y,z,yaw,pitch,roll) seems most useful
 gripper.py has useful functions for gripper

### The whole interbotix repo (Which will be installed on your VM after installing interbotix):
https://github.com/Interbotix/interbotix_ros_manipulators/blob/0bb2b0e6d0e619bff02cf74dbd5af5681dcf80c9/interbotix_ros_uxarms/examples/python_demos/joint_position_control.py
      
