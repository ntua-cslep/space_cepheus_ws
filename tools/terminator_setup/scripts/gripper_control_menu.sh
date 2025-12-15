#!/usr/bin/env bash

# Interactive menu to send gripper control commands.
# Do not exit on publish failures so the pane stays open.
set -u
set -o pipefail

printf '\033]0;GRIPPER CTRL\007'

# Source ROS env if available
if [[ -f /home/theo/ros_env.sh ]]; then
  set +u   # allow ROS scripts to reference unset vars safely
  # shellcheck disable=SC1091
  source /home/theo/ros_env.sh
  set -u
fi

selected_topic=""
selected_name=""

while true; do
  echo
  echo "=== Gripper Control Menu ==="
  echo "[e] Select END-EFFECTOR (/lefo_hear)"
  echo "[b] Select BASE (/tsoulias_hear)"
  echo "[r] Release command"
  echo "[s] Softgrip command"
  echo "[h] Hardgrip command"
  echo "[q] Quit this menu"
  if [[ -n "$selected_name" ]]; then
    echo "Current selection: $selected_name ($selected_topic)"
  else
    echo "Current selection: none"
  fi
  read -n1 -p "Selection: " key || key=""
  echo

  case "$key" in
    q|Q)
      break
      ;;
    e|E)
      selected_topic="/lefo_hear"
      selected_name="END-EFFECTOR"
      echo "Selected END-EFFECTOR gripper."
      ;;
    b|B)
      selected_topic="/tsoulias_hear"
      selected_name="BASE"
      echo "Selected BASE gripper."
      ;;
    r|R|s|S|h|H)
      if [[ -z "$selected_topic" ]]; then
        echo "Select a gripper first with [b] or [e]."
      else
        if [[ "$key" == [rR] ]]; then
          cmd="release"
        elif [[ "$key" == [sS] ]]; then
          cmd="softgrip"
        else
          cmd="hardgrip"
        fi
        echo "Sending $cmd command to $selected_name gripper..."
        rostopic pub "$selected_topic" std_msgs/String "$cmd" --once
        echo "Command sent."
      fi
      ;;
    *)
      echo "Invalid selection. Use b/e to select and r/s/h to command."
      ;;
  esac
done

exec bash
