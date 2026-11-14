#!/usr/bin/env bash
set -eo pipefail
src="$(cd "$(dirname "$0")" && pwd)"
work="$HOME/linorobot2_ws/owl_native"
schemas="${1:-/mnt/d/Workspace/foxglove-studio-cn/node_modules/@foxglove/schemas/schemas/jsonschema}"
out="$(mktemp -d "$work/live.XXXXXX")"
echo "$out" >"$work/current-live"
echo "Native OWL run: $out"
record=(cat)
if [[ -n ${OWL_LIVE_RECORDING:-} ]]; then
    record=(tee --output-error=exit "$OWL_LIVE_RECORDING")
fi

PYTHONPATH="$work/foxglove-deps" python3 -u "$src/foxglove-live.py" "$out/map.bin" "$schemas" \
    >"$out/foxglove.log" 2>&1 &
bridge=$!
trap 'kill "$bridge" 2>/dev/null || true' EXIT
(
    source "$HOME/linorobot2_ws/env.sh"
    exec python3 "$src/capture-gazebo.py" -
) 2>"$out/camera.log" |
    "${record[@]}" |
    env -i HOME="$HOME" PATH=/usr/bin:/bin OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
        OWL_ODOM_PARAMS="${OWL_ODOM_PARAMS:-}" \
        OWL_SEMANTIC_GRAPH="${OWL_SEMANTIC_GRAPH:-}" OWL_RAW_POSES="${OWL_RAW_POSES:-}" \
        LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu \
        stdbuf -oL "$work/build/owl_slam" "$src/owlslam.params" "$out/map.db" "$out/map.bin" \
        >"$out/slam.log" 2>&1
