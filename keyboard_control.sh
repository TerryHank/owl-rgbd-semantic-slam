#!/usr/bin/env bash
source "$HOME/linorobot2_ws/env.sh"
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p turn:=0.5
