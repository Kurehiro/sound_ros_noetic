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

RUN echo "export PS1='\[\e[1;36m\]\u@\h\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ '" >> /root/.bashrc

RUN cd /tmp \
    && wget https://www.python.org/ftp/python/3.10.12/Python-3.10.12.tgz \
    && tar -xvf Python-3.10.12.tgz \
    && cd Python-3.10.12 \
    && ./configure --enable-optimizations \
    && make -j$(nproc) \
    && make altinstall \
    && cd .. \
    && rm -rf Python-3.10.12 Python-3.10.12.tgz

# Install sounddevice and Whisper dependencies for default Python 3.8
RUN pip3 install sounddevice openai-whisper noisereduce soundfile && pip3 install -U scipy

# Install pip for Python 3.10
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10

RUN python3.10 -m pip install sounddevice torch torchaudio openai-whisper git+https://github.com/m-bain/whisperX.git rospkg catkin_pkg pyyaml

# Configure ALSA to use PulseAudio
RUN echo "pcm.default pulse\nctl.default pulse" > /etc/asound.conf

# Setup environment
ENV SHELL /bin/bash

# Configure Terminator
RUN mkdir -p /root/.config/terminator
COPY terminator_config /root/.config/terminator/config

# Set working directory
WORKDIR /root/workspace

# Entrypoint
CMD ["terminator"]
