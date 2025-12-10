#!/usr/bin/env bash
set -e

# Optional: make sure ROS env helper exists
if [ ! -f /home/theo/ros_env.sh ]; then
  echo "ros_env.sh not found in /home/theo. Aborting."
  exit 1
fi

# Run the Terminator layout
terminator -l cepheus_lab
