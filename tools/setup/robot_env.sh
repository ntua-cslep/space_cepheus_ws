#!/usr/bin/env bash

# ROS Networking
export ROS_MASTER_URI=http://192.168.1.38:11311
export ROS_IP=192.168.1.4

# ROS Base
source /opt/ros/kinetic/setup.bash

# Workspace Overlay
source ~/cepheus_ws_v2/devel/setup.bash
cd ~/cepheus_ws_v2

echo "✔ Robot ROS environment loaded:"
echo "  ROS_MASTER_URI=$ROS_MASTER_URI"
echo "  ROS_IP=$ROS_IP"
echo "  Working from ~/cepheus_ws_v2"
