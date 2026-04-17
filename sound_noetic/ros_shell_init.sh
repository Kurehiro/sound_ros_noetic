#!/bin/bash

# ROS workspace setup
source /root/workspace/devel/setup.bash 2>/dev/null || true

# Ensure ROS endpoint vars are sane even if setup.bash overwrites/clears them
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://ros_audio_container:11311}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-ros_audio_container}"

if [ -z "${ROS_IP}" ]; then
  export ROS_IP="$(hostname -I | awk '{print $1}')"
fi

exec bash
