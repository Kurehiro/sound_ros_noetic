#!/bin/bash
set -e

cd "$(dirname "$0")"

echo "[Step 3/3] Starting container..."
docker compose start ros_audio
echo "Container started: ros_audio_container"

echo "Attach with: docker exec -it ros_audio_container bash"
