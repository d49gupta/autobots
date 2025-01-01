# Use the official ROS 2 Humble base image
FROM ros:humble

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary packages
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-colcon-common-extensions \
    build-essential \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage \
    && rm -rf /var/lib/apt/lists/*

# Download demo ros2 nodes to test
RUN apt-get update && apt-get install -y ros-humble-demo-nodes-cpp ros-humble-demo-nodes-py

# Install dependencies
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    i2c-tools \
    libi2c-dev \
    libgpiod-dev \
    nano

# Set up a ROS workspace
RUN mkdir -p /home/ros2_ws/src
WORKDIR /home/ros2_ws

# Initialize and build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build"

# Source ROS setup scripts automatically
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /home/ros2_ws/install/setup.bash" >> ~/.bashrc

RUN pip install rosbags
RUN pip install bagpy

# Set the default command to bash
CMD ["/bin/bash"]
