#!/bin/bash
set -e

cd "$(dirname "$0")"
exec bash 02_create_container.sh
