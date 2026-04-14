#!/bin/bash
set -e

cd "$(dirname "$0")"
exec bash 01_build_image.sh
