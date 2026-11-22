#!/bin/bash
set -e
cd /workspace/map_studio
source ./env.sh
exec python3 -u app/tools/map_studio.py "$PWD/projects" --port 8850
