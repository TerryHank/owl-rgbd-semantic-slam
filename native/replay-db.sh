#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 2 || ! -f $1 || -e $2 ]]; then
    echo "Usage: bash replay-db.sh existing_input.db new_output.db" >&2
    exit 2
fi
tool="${RTABMAP_REPROCESS:-rtabmap-reprocess}"
params=()
while read -r key value; do
    [[ -z $key || $key == \#* ]] || params+=("--$key" "$value")
done < "$(dirname -- "${BASH_SOURCE[0]}")/owlslam.params"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}" OPENBLAS_NUM_THREADS=1
# Recompute motion from RGB-D. No recorded odometry guesses, loop links, or IMU.
exec "$tool" -default -odom -noimu -nopriors -nolandmark -g2 "${params[@]}" "$1" "$2"
