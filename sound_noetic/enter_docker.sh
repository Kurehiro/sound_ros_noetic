#!/bin/bash
set -e

cd "$(dirname "$0")"

# Xサーバーへのアクセスを許可
xhost +local:docker

# コンテナが停止中なら起動
if [ "$(docker inspect -f '{{.State.Running}}' ros_audio_container 2>/dev/null || true)" != "true" ]; then
  echo "ros_audio_container is not running. Starting it..."
  docker compose start ros_audio
fi

# コンテナの中に入る
docker exec -it ros_audio_container bash
