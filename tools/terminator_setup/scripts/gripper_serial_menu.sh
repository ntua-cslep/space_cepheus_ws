#!/usr/bin/env bash

set -u
set -o pipefail

printf '\033]0;GRIPPERS\007'

while true; do
  echo
  echo "=== Gripper Nodes Menu ==="
  echo "[b] Start BASE gripper (/dev/arduino_base)"
  echo "[e] Start END-EFFECTOR gripper (/dev/arduino_ee)"
  echo "[q] Quit this menu"
  read -n1 -p "Selection: " key || key=""
  echo

  case "$key" in
    q|Q)
      break
      ;;
    b|B)
      echo "Starting BASE gripper node on robot... (Ctrl+C to stop)"
      ssh -C -t cepheus@192.168.1.4 "bash -lc 'source ~/robot_env.sh; rosrun rosserial_python serial_node.py /dev/arduino_base'"
      echo
      echo "BASE gripper node stopped. Press b/e to start again or [q] to quit."
      ;;
    e|E)
      echo "Starting END-EFFECTOR gripper node on robot... (Ctrl+C to stop)"
      ssh -C -t cepheus@192.168.1.4 "bash -lc 'source ~/robot_env.sh; rosrun rosserial_python serial_node.py /dev/arduino_ee'"
      echo
      echo "END-EFFECTOR gripper node stopped. Press b/e to start again or [q] to quit."
      ;;
    *)
      echo "Invalid selection. Use b (base), e (end-effector), or q to quit."
      ;;
  esac
done

exec bash
