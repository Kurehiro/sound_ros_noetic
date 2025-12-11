#!/bin/bash

# --- Configuration ---
# Set your ROS network configuration here
ROS_MASTER_URI="http://150.89.169.66:11311"
ROS_IP="150.89.169.140"
# ---------------------

# Build the image (if not already built or if changes detected)
docker build -t ros_noetic_audio .

# Allow X11 connections from localhost
xhost + 127.0.0.1

# Run the container
# We use a custom command to:
# 1. Remove the hardcoded ROS variables from .bashrc (which override our env vars)
# 2. Add 'source /opt/ros/noetic/setup.bash' to .bashrc if not present
# 3. Start terminator
docker run -it --rm \
    --env="DISPLAY=host.docker.internal:0" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="PULSE_SERVER=docker.for.mac.localhost" \
    --env="PULSE_COOKIE=/root/.config/pulse/cookie" \
    --env="ROS_MASTER_URI=$ROS_MASTER_URI" \
    --env="ROS_IP=$ROS_IP" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/cookie:/root/.config/pulse/cookie" \
    --volume="$(pwd):/root/workspace" \
    ros_noetic_audio \
    bash -c "sed -i '/export ROS_/d' /root/.bashrc && echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && terminator"
