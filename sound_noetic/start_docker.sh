#!/bin/bash

# Allow X11 connections
xhost +local:root

# Run the container
docker run -it --rm \
    --net=host \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="PULSE_SERVER=unix:/run/user/1000/pulse/native" \
    --env="PULSE_COOKIE=/root/.config/pulse/cookie" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/run/user/1000/pulse/native:/run/user/1000/pulse/native" \
    --volume="/home/niko25/.config/pulse/cookie:/root/.config/pulse/cookie" \
    --volume="$(pwd):/root/workspace" \
    --device /dev/snd \
    --group-add audio \
    ros_noetic_audio
