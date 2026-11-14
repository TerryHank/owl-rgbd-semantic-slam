#!/usr/bin/env bash
source "$HOME/linorobot2_ws/env.sh"
exec ros2 launch "$HOME/linorobot2_ws/display/rosbot_xl/owl_nav.launch.py"
