"""Attach NPU captions to the final optimized poses of their OWL keyframes."""
import argparse
import csv
import json
from pathlib import Path
import re
import sqlite3


def build(database, poses, captions):
    with poses.open(newline="") as stream:
        optimized = {int(row["node_id"]): row for row in csv.DictReader(stream)}
    with sqlite3.connect(database) as db:
        stamps = {round(stamp, 6): node for node, stamp in db.execute("SELECT id, stamp FROM Node")}
    observations = []
    labels = {"door": "门", "window": "窗", "brick wall": "砖墙", "table": "桌子", "floor": "地面"}
    for path in sorted(captions.glob("*.log")):
        stamp = float(path.stem.split("-", 1)[1])
        node = stamps.get(round(stamp, 6))
        if node not in optimized:
            continue
        text = path.read_text(encoding="utf-8").split("User: Answer:", 1)
        if len(text) != 2:
            continue
        caption = text[1].split("User:", 1)[0].strip()
        matched = [label for term, label in labels.items() if re.search(r"\b"+term+r"\b", caption, re.I)]
        p = optimized[node]
        observations.append({"node_id": node, "stamp": stamp, "x": float(p["x"]), "y": float(p["y"]),
                             "yaw": float(p["yaw"]), "label": "、".join(matched) or "场景观察", "caption": caption})
    return sorted(observations, key=lambda item: item["stamp"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    for name in ("database", "poses", "captions", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(build(args.database, args.poses, args.captions), ensure_ascii=False, indent=2)
                           + "\n", encoding="utf-8")
