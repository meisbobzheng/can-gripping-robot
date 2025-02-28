# WX200 Robot Arm - ROS Docker Setup

This repository contains Docker configuration files and sample Python scripts for controlling a WX200 robot arm using ROS (Robot Operating System). The setup is designed to work across both macOS and Windows.

## Prerequisites

- Docker Desktop installed:
  - [Docker Desktop for Mac](https://docs.docker.com/desktop/install/mac-install/)
  - [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/)
- Git installed
- For GUI applications (RViz, etc.):
  - Mac: XQuartz installed
  - Windows: VcXsrv or similar X server

## Setup Instructions

### 1. Create project directory and files

Create a new directory for this project:

```bash
mkdir wx200-ros-docker
cd wx200-ros-docker
```

Save the Dockerfile, docker-compose.yml, and setup.sh files provided in this repository to your project directory.

### 2. Build and start the Docker container

```bash
docker-compose up -d --build
```

This will build the Docker image and start the container in the background. The first build may take several minutes to complete.

### 3. Run the setup script

```bash
chmod +x setup.sh
./setup.sh
```

This will create the necessary ROS package and Python script for controlling the WX200 robot.

### 4. Enter the Docker container

```bash
docker exec -it wx200_ros bash
```

This will give you an interactive bash shell inside the container where you can run ROS commands.

### 5. Build the ROS workspace

Inside the container:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Setting up the environment

After building and starting the container, run the setup script to create the necessary files:

```bash
# On your host machine
chmod +x setup.sh
./setup.sh
```

### Running the example script

Once inside the container, you can run the example Python script:


Note: wx200_control.py is in ~/catkin_ws/src/wx200_robot/wx200_control/scripts

```bash
cd /root/catkin_ws
rosrun wx200_control wx200_control.py
```

If you get a "package not found" error, build the workspace first:

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

### Connecting to the physical robot

To connect to the physical WX200 robot arm:

1. Connect the robot to your computer via USB
2. Inside the Docker container, the USB device should be accessible
3. You may need to adjust permissions for USB access:

```bash
# Inside the container
ls -la /dev/ttyUSB*  # Check if USB devices are visible
sudo chmod 666 /dev/ttyUSB0  # Adjust permissions if needed
```

### GUI Applications (RViz, etc.)

To run RViz or other GUI applications:

#### On Mac:

1. Install and start XQuartz
2. In XQuartz preferences, go to Security tab and enable "Allow connections from network clients"
3. Run in terminal: `xhost +localhost`
4. Inside the container:

```bash
export DISPLAY=host.docker.internal:0
rosrun rviz rviz
```

#### On Windows:

1. Install and start VcXsrv (or another X server)
2. Set "Multiple windows" and "Start no client"
3. In "Extra settings", check "Disable access control"
4. Inside the container:

```bash
export DISPLAY=host.docker.internal:0
rosrun rviz rviz
```

## Development Workflow

1. Write your Python scripts in the `src` directory on your host machine
2. The `src` directory is mounted inside the container at `/root/catkin_ws/src`
3. Run and test your scripts inside the container
4. Changes made on your host machine will be immediately reflected inside the container

## Troubleshooting

### USB Connection Issues

- Make sure the robot is powered on
- Check if the USB device is visible: `ls -la /dev/ttyUSB*`
- Try unplugging and reconnecting the USB cable
- Ensure the Docker container is running in privileged mode

### ROS Issues

- Check if ROS is properly sourced: `source /opt/ros/noetic/setup.bash`
- Verify the ROS master is running: `rosnode list`
- Start ROS master if needed: `roscore`

### GUI Issues

- Verify your X server is running and accessible
- Check the DISPLAY environment variable: `echo $DISPLAY`
- For Mac users, make sure XQuartz is allowing network connections

## Additional Resources

- [ROS Documentation](http://wiki.ros.org/)
- [Docker Documentation](https://docs.docker.com/)
- [ROS with Docker Tutorial](http://wiki.ros.org/docker/Tutorials/Docker)
