#!/bin/bash

# Xサーバーへのアクセスを許可
xhost +local:docker

# Auto-detect IP (first non-loopback interface)
export ROS_IP=$(hostname -I | awk '{print $1}')

echo "Detected IP: $ROS_IP"

# コンテナをバックグラウンドで起動（存在しない場合は作成、停止している場合は起動）
docker compose up -d

# コンテナの中に入る
docker exec -it ros_audio_container bash
