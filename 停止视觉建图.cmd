@echo off
wsl -d Ubuntu-22.04 -- tmux send-keys -t owl-rtabmap C-c
