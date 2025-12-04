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
    ffmpeg \
    curl \
    build-essential \
    zlib1g-dev \
    libncurses5-dev \
    libgdbm-dev \
    libnss3-dev \
    libssl-dev \
    libreadline-dev \
    libffi-dev \
    libsqlite3-dev \
    wget \
    git \
    libbz2-dev \
    && rm -rf /var/lib/apt/lists/*

RUN cd /tmp \
    && wget https://www.python.org/ftp/python/3.10.12/Python-3.10.12.tgz \
    && tar -xvf Python-3.10.12.tgz \
    && cd Python-3.10.12 \
    && ./configure --enable-optimizations \
    && make -j$(nproc) \
    && make altinstall \
    && cd .. \
    && rm -rf Python-3.10.12 Python-3.10.12.tgz

# Install sounddevice for default Python 3.8 (for sound_trigger.py)
RUN pip3 install sounddevice

# Install pip for Python 3.10
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10

RUN python3.10 -m pip install sounddevice torch torchaudio openai-whisper git+https://github.com/m-bain/whisperX.git rospkg catkin_pkg pyyaml

# Configure ALSA to use PulseAudio
RUN echo "pcm.default pulse\nctl.default pulse" > /etc/asound.conf

# Setup environment
ENV SHELL /bin/bash

# Entrypoint
CMD ["terminator"]
