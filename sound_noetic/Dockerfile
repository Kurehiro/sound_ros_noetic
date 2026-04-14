FROM osrf/ros:noetic-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    terminator \
    alsa-utils \
    pulseaudio \
    python3-pyaudio \
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

RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10

# Python 3.8 側は軽いものだけ
RUN pip3 install --default-timeout=1000 \
    sounddevice \
    noisereduce \
    soundfile \
    rospkg \
    catkin_pkg \
    pyyaml

# Whisper系は Python 3.10 側にまとめる
RUN python3.10 -m pip install --upgrade pip setuptools wheel
RUN python3.10 -m pip install --default-timeout=1000 --no-cache-dir \
    torch \
    torchaudio \
    openai-whisper \
    git+https://github.com/m-bain/whisperX.git

RUN echo "pcm.default pulse\nctl.default pulse" > /etc/asound.conf

ENV SHELL /bin/bash

RUN mkdir -p /root/.config/terminator
COPY terminator_config /root/.config/terminator/config

WORKDIR /root/workspace

CMD ["terminator"]
