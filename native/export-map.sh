#!/usr/bin/env bash
set -euo pipefail
if [[ $# != 2 || ! -f $1 || ! -d $2 ]]; then
    echo "Usage: bash export-map.sh map.db existing_output_dir" >&2
    exit 2
fi
tool="${RTABMAP_EXPORT:-rtabmap-export}"
exec "$tool" --map --cloud --opt 2 --output_dir "$2" "$1"
