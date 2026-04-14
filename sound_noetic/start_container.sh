#!/bin/bash
set -e

cd "$(dirname "$0")"
exec bash 03_start_container.sh
