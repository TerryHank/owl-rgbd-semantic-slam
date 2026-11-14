#!/usr/bin/env bash
set -e
source "$HOME/linorobot2_ws/env.sh"
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=1
mkdir -p "$HOME/linorobot2_ws/maps"
exec ros2 launch "$HOME/linorobot2_ws/display/rosbot_xl/owl_mapping.launch.py" \
    viz:="${RTABMAP_VIZ:-true}" \
    database_path:="${OWL_MAP_DB:-$HOME/linorobot2_ws/maps/owl_rgbd_live_grid.db}"
