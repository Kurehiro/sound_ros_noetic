#!/bin/bash

# Allow X11 connections
xhost +local:root

# Run the container
docker run -it --rm \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/root/workspace" \
    --device /dev/snd \
    ros_noetic_audio
