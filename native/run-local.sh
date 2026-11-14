#!/usr/bin/env bash
set -euo pipefail
ws="$HOME/linorobot2_ws/owl_native"
if [[ $# != 2 || ! -f $1 || -e $2 ]]; then
    echo "Usage: bash run-local.sh existing_owl_recording.owl new_map.db" >&2
    exit 2
fi
config="$(dirname -- "${BASH_SOURCE[0]}")/owlslam.params"
exec env -i HOME="$HOME" PATH=/usr/bin:/bin OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
    OWL_ODOM_PARAMS="${OWL_ODOM_PARAMS:-}" \
    LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu \
    "$ws/build/owl_slam" "$config" "$2" < "$1"
