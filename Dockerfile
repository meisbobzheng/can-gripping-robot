# Use ROS Noetic with Ubuntu 20.04 as the base image
FROM osrf/ros:noetic-desktop-full

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Install necessary tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-dev \
    python3-setuptools \
    build-essential \
    curl \
    wget \
    vim \
    nano \
    net-tools \
    iputils-ping \
    usbutils \
    ros-noetic-catkin \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    numpy \
    scipy \
    matplotlib \
    pyyaml \
    rospkg \
    pyserial \
    rospy-message-converter

# Create a workspace for ROS
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Create a basic package structure instead of cloning a non-existent repository
RUN cd /root/catkin_ws/src && \
    mkdir -p wx200_robot && \
    cd wx200_robot && \
    catkin_create_pkg wx200_control rospy roscpp std_msgs sensor_msgs geometry_msgs

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make"

# Add source commands to bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set up environment variables
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_HOSTNAME=localhost

# Create an entrypoint script
RUN echo '#!/bin/bash\nsource /opt/ros/noetic/setup.bash\nsource /root/catkin_ws/devel/setup.bash\nexec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

# Expose ROS ports
EXPOSE 11311 9090