@echo off
wsl -d Ubuntu-22.04 -- bash -lc "tmux has-session -t linorobot2-sim 2>/dev/null || tmux new-session -d -s linorobot2-sim bash ~/linorobot2_ws/start_sim.sh"
