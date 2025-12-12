#!/bin/bash

# Xサーバーへのアクセスを許可
xhost +local:docker

# コンテナをバックグラウンドで起動（存在しない場合は作成、停止している場合は起動）
docker compose up -d

# コンテナの中に入る
docker exec -it ros_audio_container bash
