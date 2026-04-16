#!/bin/bash

# Default values
DEFAULT_ROS_MASTER_URI="http://ros_audio_container:11311"
DEFAULT_ROS_HOSTNAME="ros_audio_container"

# Check if .env exists and load it
if [ -f .env ]; then
    echo "Loading existing .env file..."
    source .env
else
    echo "Creating new .env file..."
fi

# Get local user info
USER_ID=$(id -u)
GROUP_ID=$(id -g)
XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$USER_ID}
HOME_DIR=$HOME

# Prompt for ROS settings
read -p "Enter ROS_MASTER_URI [${ROS_MASTER_URI:-$DEFAULT_ROS_MASTER_URI}]: " INPUT_URI
ROS_MASTER_URI=${INPUT_URI:-${ROS_MASTER_URI:-$DEFAULT_ROS_MASTER_URI}}

read -p "Enter ROS_HOSTNAME [${ROS_HOSTNAME:-$DEFAULT_ROS_HOSTNAME}]: " INPUT_HOSTNAME
ROS_HOSTNAME=${INPUT_HOSTNAME:-${ROS_HOSTNAME:-$DEFAULT_ROS_HOSTNAME}}

# Write to .env
cat << EOF_ENV > .env
ROS_MASTER_URI=$ROS_MASTER_URI
ROS_HOSTNAME=$ROS_HOSTNAME
XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR
HOME_DIR=$HOME_DIR
USER_ID=$USER_ID
GROUP_ID=$GROUP_ID
DISPLAY=$DISPLAY
EOF_ENV

echo "Configuration saved to .env:"
cat .env
echo ""
echo "Next steps:"
echo "  1) bash 01_build_image.sh"
echo "  2) bash 02_create_container.sh"
echo "  3) bash 03_start_container.sh"
