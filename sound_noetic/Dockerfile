FROM osrf/ros:noetic-desktop

# Install terminator and audio utils
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    terminator \
    alsa-utils \
    pulseaudio \
    python3-pyaudio \
    python3-scipy \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Setup environment
ENV SHELL /bin/bash

# Entrypoint
CMD ["terminator"]
