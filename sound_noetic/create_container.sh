#!/bin/bash
set -e

cd "$(dirname "$0")"

echo "[Step 2/3] Creating container (without starting)..."
docker compose create ros_audio
echo "Container created: ros_audio_container"
