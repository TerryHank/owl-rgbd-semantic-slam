#!/usr/bin/env bash
set -e
source "$HOME/linorobot2_ws/env.sh"
setsid ros2 launch "$HOME/linorobot2_ws/display/rosbot_xl/classic.launch.py" "$@" &
launch_pid=$!
cleanup() {
    trap - EXIT INT TERM
    kill -INT -- "-$launch_pid" 2>/dev/null || true
    for attempt in {1..25}; do
        kill -0 -- "-$launch_pid" 2>/dev/null || break
        sleep 0.2
    done
    kill -TERM -- "-$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
wait "$launch_pid"
