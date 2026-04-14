#!/bin/bash
set -e

cd "$(dirname "$0")"

# Xサーバーへのアクセスを許可（失敗しても継続）
xhost +local:docker >/dev/null 2>&1 || true

# サービスが未作成/停止中でも確実に起動
if ! docker compose ps --status running --services | grep -qx "ros_audio"; then
  echo "ros_audio_container is not running. Starting it..."
  docker compose up -d ros_audio
fi

# コンテナの中に入る
exec docker exec -it ros_audio_container bash
