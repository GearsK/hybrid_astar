#!/usr/bin/env bash
# Run Hybrid A* Docker image with RViz over VNC (avoids XQuartz/GLX issues on macOS).
# From repo root: ./scripts/run_with_rviz_vnc.sh
# Then connect with a VNC client to localhost:5900 (e.g. Screen Sharing → vnc://localhost:5900).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

docker run --rm --platform linux/amd64 -p 5900:5900 hybrid_astar:noetic /catkin_ws/src/hybrid_astar/scripts/run_rviz_vnc.sh
