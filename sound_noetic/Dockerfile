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

# Set ROS network environment variables (Default to localhost for now, user should override)
ENV ROS_MASTER_URI=http://localhost:11311
ENV ROS_IP=127.0.0.1

# Ensure they are set in interactive shells
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc \
    && echo "export ROS_IP=127.0.0.1" >> /root/.bashrc

# Install Python 3.10 from source (needed for some newer libs?)
# Note: Building Python from source on emulated amd64 is very slow.
# On arm64 native it should be faster.
RUN cd /tmp \
    && wget https://www.python.org/ftp/python/3.10.12/Python-3.10.12.tgz \
    && tar -xvf Python-3.10.12.tgz \
    && cd Python-3.10.12 \
    && ./configure --enable-optimizations \
    && make -j$(nproc) \
    && make altinstall \
    && cd .. \
    && rm -rf Python-3.10.12 Python-3.10.12.tgz

# Install pip for Python 3.10
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10

# Install dependencies for Python 3.8 (System default)
# Install CPU-only torch first to avoid pulling huge CUDA wheels
RUN pip3 install --no-cache-dir torch --index-url https://download.pytorch.org/whl/cpu
RUN pip3 install --no-cache-dir sounddevice openai-whisper noisereduce soundfile

# Install dependencies for Python 3.10
RUN python3.10 -m pip install --no-cache-dir torch torchaudio --index-url https://download.pytorch.org/whl/cpu
RUN python3.10 -m pip install --no-cache-dir faster-whisper pyannote.audio sounddevice openai-whisper git+https://github.com/m-bain/whisperX.git rospkg catkin_pkg pyyaml --extra-index-url https://download.pytorch.org/whl/cpu

# Configure ALSA to use PulseAudio
RUN echo "pcm.default pulse\nctl.default pulse" > /etc/asound.conf

# Setup environment
ENV SHELL /bin/bash

# Entrypoint
CMD ["terminator"]
