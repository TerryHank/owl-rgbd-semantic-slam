@echo off
wsl -d Ubuntu-22.04 -- bash -lc "tmux has-session -t owl-nav 2>/dev/null || tmux new-session -d -s owl-nav bash ~/linorobot2_ws/start_navigation.sh"
