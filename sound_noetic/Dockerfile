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
    python3-pip \
    libportaudio2 \
    libasound2-plugins \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install sounddevice

# Configure ALSA to use PulseAudio
RUN echo "pcm.default pulse\nctl.default pulse" > /etc/asound.conf

# Setup environment
ENV SHELL /bin/bash

# Entrypoint
CMD ["terminator"]
