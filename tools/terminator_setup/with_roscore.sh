#!/usr/bin/env bash
set -e

echo "[with_roscore] Waiting for roscore..."
# Try to talk to the ROS master until it responds
while ! rosparam list >/dev/null 2>&1; do
  sleep 1
done

echo "[with_roscore] roscore is up, running: $*"
exec "$@"
