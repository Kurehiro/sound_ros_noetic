#!/bin/bash
set -e

cd "$(dirname "$0")"

# .env を読み込んで ROS 設定を反映
if [ -f .env ]; then
  # shellcheck disable=SC1091
  source .env
fi

DISPLAY_FOR_CONTAINER=${DISPLAY:-:0}

# Xサーバーへのアクセスを許可（失敗しても継続）
xhost +local:docker >/dev/null 2>&1 || true

# コンテナを起動（存在しない場合は作成、停止している場合は起動）
docker compose up -d ros_audio

# bridge ネットワーク上のコンテナIPを取得し、ROSノード広告先として利用
CONTAINER_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' ros_audio_container)
MASTER_URI=${ROS_MASTER_URI:-http://ros_audio_container:11311}
HOSTNAME_FOR_ROS=${ROS_HOSTNAME:-ros_audio_container}

echo "DISPLAY=${DISPLAY_FOR_CONTAINER}"
echo "ROS_MASTER_URI=${MASTER_URI}"
echo "ROS_HOSTNAME=${HOSTNAME_FOR_ROS}"
echo "ROS_IP=${CONTAINER_IP}"

# 毎回 Terminator 設定を読み込む（8分割レイアウト）
# マウント先の違いに備えて複数パスを探索
if ! docker exec ros_audio_container bash -lc '
  mkdir -p /root/.config/terminator
  CONFIG_PATH=""
  if [ -f /root/workspace/terminator_config ]; then
    CONFIG_PATH=/root/workspace/terminator_config
  elif [ -f /root/workspace/sound_noetic/terminator_config ]; then
    CONFIG_PATH=/root/workspace/sound_noetic/terminator_config
  else
    exit 1
  fi

  # マージ競合マーカーが残っている設定はコピーしない
  if grep -Eq "^(<<<<<<<|=======|>>>>>>>)" "${CONFIG_PATH}"; then
    echo "Error: terminator_config contains merge-conflict markers: ${CONFIG_PATH}"
    exit 2
  fi

  cp "${CONFIG_PATH}" /root/.config/terminator/config
  exit 0
'; then
  echo "Warning: terminator_config not found or invalid in container. Using existing config if present."
fi

# Terminator起動。失敗した場合はbashへフォールバック
if docker exec -it \
  -e DISPLAY="${DISPLAY_FOR_CONTAINER}" \
  -e GDK_DISABLE_SHM=1 \
  -e NO_AT_BRIDGE=1 \
  -e ROS_MASTER_URI="${MASTER_URI}" \
  -e ROS_HOSTNAME="${HOSTNAME_FOR_ROS}" \
  -e ROS_IP="${CONTAINER_IP}" \
  ros_audio_container terminator --no-dbus -l ros_8; then
  exit 0
fi

echo "Terminator launch failed. Falling back to bash shell."
exec docker exec -it \
  -e DISPLAY="${DISPLAY_FOR_CONTAINER}" \
  -e GDK_DISABLE_SHM=1 \
  -e NO_AT_BRIDGE=1 \
  -e ROS_MASTER_URI="${MASTER_URI}" \
  -e ROS_HOSTNAME="${HOSTNAME_FOR_ROS}" \
  -e ROS_IP="${CONTAINER_IP}" \
  ros_audio_container bash -lc "/root/workspace/ros_shell_init.sh"

export LANG=ja_JP.UTF-8