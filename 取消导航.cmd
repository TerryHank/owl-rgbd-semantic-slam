@echo off
wsl -d Ubuntu-22.04 -- bash -lc "source ~/linorobot2_ws/env.sh; ros2 service call /owl/navigation/cancel std_srvs/srv/Trigger {}"
