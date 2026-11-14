#!/usr/bin/env bash
set -e
cd "$HOME/linorobot2_ws"
testdb="$PWD/maps/owl_loop_$(date +%Y%m%d_%H%M%S).db"
printf '%s\n' "$testdb" > maps/active_loop_db.txt
export OWL_MAP_DB="$testdb"
export OWL_ODOM_ARGS="--Odom/Strategy 1 --Odom/KeyFrameThr 0.65 --Vis/CorType 1 --Vis/EstimationType 0 --Vis/InlierDistance 0.05 --Vis/CorFlowWinSize 24 --Vis/CorFlowMaxLevel 4 --Vis/GridRows 3 --Vis/GridCols 4 --Reg/Force3DoF true"
export RTABMAP_VIZ=false
export OWL_SLAM_ARGS="--RGBD/ProximityBySpace false --Reg/Force3DoF true --Vis/EstimationType 0 --Vis/InlierDistance 0.05"
exec bash "$PWD/start_rtabmap.sh"
