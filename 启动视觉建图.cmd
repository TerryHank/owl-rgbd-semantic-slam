@echo off
wsl -d Ubuntu-22.04 -- bash -lc "tmux has-session -t owl-rtabmap 2>/dev/null || tmux new-session -d -s owl-rtabmap bash ~/linorobot2_ws/start_rtabmap.sh"
