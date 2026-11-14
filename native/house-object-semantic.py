"""Project the House green-bin masks into the final optimized RGB-D map."""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def project(recording, poses, native_map, captions, output):
    detections = {348.871: 15, 355.030: 21}
    with poses.open(newline="") as stream:
        optimized = {int(p["node_id"]): p for p in csv.DictReader(stream)}
    with native_map.open("rb") as stream:
        header = stream.readline().split()
        stream.seek(int(header[9])*int(header[10]), 1)
        cloud = np.frombuffer(stream.read(), dtype=[("xyz", "<f4", 3), ("rgba", "u1", 4)])["xyz"]
    tree = cKDTree(cloud)
    observations = []
    with recording.open("rb") as stream:
        _, width, height, *calibration = stream.readline().decode().split()
        width, height = int(width), int(height)
        fx, fy, cx, cy, tx, ty, tz, roll, pitch, yaw = map(float, calibration)
        optical_to_base = Rotation.from_euler("xyz", (roll, pitch, yaw))
        size = width*height
        for row in iter(stream.readline, b""):
            stamp = round(float(row), 6)
            node = detections.get(stamp)
            if node is None:
                stream.seek(size*7, 1)
                continue
            bgr = np.frombuffer(stream.read(size*3), dtype=np.uint8).reshape(height, width, 3)
            depth = np.frombuffer(stream.read(size*4), dtype="<f4").reshape(height, width)
            mask = cv2.inRange(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), (35, 65, 7), (94, 255, 210))
            _, components, stats, _ = cv2.connectedComponentsWithStats(mask)
            mask = (components == (1+np.argmax(stats[1:, cv2.CC_STAT_AREA])))
            yy, xx = np.nonzero(mask & np.isfinite(depth) & (depth >= .3) & (depth <= 8.))
            z = depth[yy, xx]
            near = abs(z - np.median(z)) < .22
            yy, xx, z = yy[near], xx[near], z[near]
            optical = np.column_stack(((xx-cx)*z/fx, (yy-cy)*z/fy, z))
            base = optical_to_base.apply(optical) + (tx, ty, tz)
            pose = optimized[node]
            angle = float(pose["yaw"])
            points = Rotation.from_euler("z", angle).apply(base) + tuple(float(pose[k]) for k in "xyz")
            center = np.median(points, axis=0)
            # Select an actually measured object pixel near the center of the surface.
            anchor = points[np.argmin(np.linalg.norm(points-center, axis=1))]
            distance, _ = tree.query(anchor)
            x, y, zz = map(float, anchor)
            caption_path = captions / f"object-{node}-{stamp:.6f}.png.log"
            caption = caption_path.read_text(encoding="utf-8").split("User: Answer:", 1)[1].split("User:", 1)[0].strip()
            if "trash can" not in caption.lower():
                continue
            observations.append({"node_id": node, "stamp": stamp, "x": x, "y": y, "z": zz,
                                 "yaw": angle, "label": "垃圾桶", "caption": caption,
                                 "mask_source": "House green-bin RGB mask",
                                 "valid_depth_pixels": len(points), "map_surface_distance_m": float(distance)})
    if len(observations) != len(detections):
        raise RuntimeError("Missing measured object observations")
    # Both masks observe the same bin; retain the best supported surface anchor.
    anchor = min(observations, key=lambda item: item["map_surface_distance_m"])
    anchor = {**anchor, "supporting_observations": observations}
    output.write_text(json.dumps([anchor], ensure_ascii=False, indent=2)+"\n", encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    for name in ("recording", "poses", "native_map", "captions", "output"):
        parser.add_argument(name, type=Path)
    p = parser.parse_args()
    project(p.recording, p.poses, p.native_map, p.captions, p.output)
