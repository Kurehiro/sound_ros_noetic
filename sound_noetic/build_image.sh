#!/bin/bash
set -e

cd "$(dirname "$0")"

echo "[Step 1/3] Building image..."
docker compose build ros_audio
echo "Build complete: ros_noetic_audio"
