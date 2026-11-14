"""Fuse segmented RGB-D surfaces with the optimized pose graph of each map revision."""
import argparse
import csv
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def rows(path):
    with path.open(newline="") as source:
        return list(csv.DictReader(source))


def observations(path, poses, extrinsic, offset, names):
    with path.open() as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue  # An inference may still be appending its final row.
            stamp = round(item["stamp"], 6)
            if stamp not in poses or item["category"] >= len(names):
                continue
            points = np.asarray(item["points"])
            center = np.median(points[:, 2])
            points = points[abs(points[:, 2]-center) < .25]
            if len(points) < 4:
                continue
            pose = poses[stamp]
            points = Rotation.from_euler("z", float(pose["yaw"])).apply(
                extrinsic.apply(points) + offset) + [float(pose[k]) for k in "xyz"]
            yield item["category"], stamp, item["score"], points


def associate(items):
    tracks = []
    for category, stamp, score, points in items:
        center = np.median(points, axis=0)
        matches = []
        for track in tracks:
            if track["category"] != category or stamp in track["stamps"]:
                continue
            distance = np.linalg.norm(center-track["center"])
            if distance > .65:
                continue
            sample = points[::max(1, len(points)//64)]
            near, _ = cKDTree(track["points"]).query(sample)
            if distance < .23 or (near < .15).mean() > .20:
                matches.append((distance, track))
        if not matches:
            tracks.append({"id": len(tracks)+1, "category": category, "stamps": {stamp},
                           "center": center, "points": points, "view": points, "score": score})
            continue
        track = min(matches, key=lambda pair: pair[0])[1]
        track["stamps"].add(stamp)
        # Keep representative samples across viewpoints without retaining every raw mask pixel.
        track["points"] = np.concatenate((track["points"], points))
        if len(track["points"]) > 1800:
            track["points"] = track["points"][np.linspace(0, len(track["points"])-1, 1800, dtype=int)]
        track["center"] = np.median(track["points"], axis=0)
        if score > track["score"]:
            track["view"], track["score"] = points, score
    return tracks


def merge_fragments(tracks):
    merged = []
    for track in sorted(tracks, key=lambda t: (-len(t["stamps"]), -t["score"])):
        if len(track["stamps"]) < 3:
            continue
        for other in merged:
            if np.linalg.norm(track["center"]-other["center"]) > .8:
                continue
            sample = track["points"][::max(1, len(track["points"])//100)]
            inverse = other["points"][::max(1, len(other["points"])//100)]
            distance = (cKDTree(other["points"]).query(sample)[0] < .12).mean()
            reverse = (cKDTree(track["points"]).query(inverse)[0] < .12).mean()
            same_class = track["category"] == other["category"]
            center_distance = np.linalg.norm(track["center"]-other["center"])
            common = len(track["stamps"] & other["stamps"])
            if same_class and common > 1 and center_distance >= .32 and max(distance, reverse) < .65:
                continue
            if (same_class and (center_distance < .32 or max(distance, reverse) >= .40
                                or (not common and center_distance < .30))) or (
                not same_class and max(distance, reverse) >= .65
                and center_distance < .30
            ):
                seen = other["stamps"] | track["stamps"]
                if not same_class and track["score"] > other["score"]:
                    other.update(track)
                other["stamps"] = seen
                break
        else:
            merged.append(track)
    return merged


class ObjectMap:
    def __init__(self, recording, detections, odometry, classes):
        with recording.open("rb") as stream:
            header = stream.readline().split()
        tx, ty, tz, roll, pitch, yaw = map(float, header[7:13])
        self.camera_rotation = Rotation.from_euler("xyz", (roll, pitch, yaw))
        self.camera_offset = (tx, ty, tz)
        self.detections, self.odometry = detections, odometry
        self.names = json.loads(classes.read_text(encoding="utf-8"))
        labels_path = Path(__file__).with_name("yoloe-indoor-labels-zh.json")
        self.display_names = json.loads(labels_path.read_text(encoding="utf-8"))
        self.last_objects = []
        self.next_id = 1

    def update(self, native_map):
        graph = native_map.with_name(native_map.name+".poses.csv")
        if not all(path.exists() for path in (graph, self.odometry, self.detections)):
            return None
        with native_map.open("rb") as stream:
            header = stream.readline().split()
            revision, stamp = int(header[1]), float(header[2])
            stream.seek(int(header[9])*int(header[10]), 1)
            cloud = np.frombuffer(stream.read(), dtype=[("xyz", "<f4", 3), ("color", "u1", 4)])["xyz"]
        nodes = rows(graph)
        if not nodes or int(nodes[0]["revision"]) != revision:
            return None
        nodes.sort(key=lambda n: float(n["stamp"]))
        stamps = np.array([float(n["stamp"]) for n in nodes])
        raw = {round(float(p["stamp"]), 6): p for p in rows(self.odometry) if float(p["stamp"]) <= stamp}
        items = observations(self.detections, raw, self.camera_rotation, self.camera_offset, self.names)
        projected = []
        for category, ts, score, points in items:
            i = np.searchsorted(stamps, ts)
            left, right = max(0, i-1), min(len(nodes)-1, i)
            transforms = []
            for index in (left, right):
                n = nodes[index]
                angle = float(n["yaw"])-float(n["oyaw"])
                offset = np.array([float(n[k]) for k in "xyz"]) - Rotation.from_euler("z", angle).apply(
                    [float(n["ox"]), float(n["oy"]), 0.])
                transforms.append((angle, offset))
            alpha = (ts-stamps[left])/(stamps[right]-stamps[left]) if right != left else 0.
            a, t = transforms[0]
            b, u = transforms[1]
            angle = a + alpha*np.arctan2(np.sin(b-a), np.cos(b-a))
            projected.append((category, ts, score, Rotation.from_euler("z", angle).apply(points)+(1-alpha)*t+alpha*u))

        tree = cKDTree(cloud)
        output = []
        for track in merge_fragments(associate(projected)):
            view = track["view"]
            center = np.median(view, axis=0)
            nearby = view[np.argsort(np.linalg.norm(view-center, axis=1))[:64]]
            distances, indexes = tree.query(nearby)
            valid = np.flatnonzero(distances <= .08)
            if not len(valid):
                continue
            j = valid[0]
            x, y, z = map(float, cloud[indexes[j]])
            label = self.names[track["category"]]
            duplicate = next((item for item in output if item["label"] == label
                              and np.linalg.norm([item["x"]-x, item["y"]-y, item["z"]-z]) < .22), None)
            if duplicate is not None:
                shared = sorted(track["stamps"] | set(duplicate["observation_stamps"]))
                duplicate.update(observation_stamps=shared, support=len(shared),
                                 caption=f"{len(shared)} 次 RGB-D 观测")
            else:
                output.append({"stamp": min(track["stamps"]), "x": x, "y": y, "z": z,
                               "label": label, "caption": f'{len(track["stamps"])} 次 RGB-D 观测',
                               "support": len(track["stamps"]), "confidence": track["score"],
                               "map_revision": revision, "projection_distance_m": float(distances[j]),
                               "observation_stamps": sorted(track["stamps"])})
        previous = self.last_objects
        candidates = []
        for i, item in enumerate(output):
            for j, old in enumerate(previous):
                if item["label"] != old["label"]:
                    continue
                overlap = len(set(item["observation_stamps"]) & set(old["observation_stamps"]))
                if overlap:
                    candidates.append((-overlap, i, j))
        used = set()
        for _, i, j in sorted(candidates):
            if j not in used and "node_id" not in output[i]:
                output[i]["node_id"] = previous[j]["node_id"]
                used.add(j)
        for item in output:
            if "node_id" not in item:
                item["node_id"] = self.next_id
                self.next_id += 1
        self.last_objects = output
        return [{**item, "class_name": item["label"],
                 "label": self.display_names.get(item["label"], item["label"])} for item in output]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    for name in ("recording", "detections", "odometry", "classes", "native_map", "output"):
        parser.add_argument(name, type=Path)
    parser.add_argument("--watch", action="store_true")
    args = parser.parse_args()
    mapper = ObjectMap(args.recording, args.detections, args.odometry, args.classes)
    previous = None
    while True:
        try:
            version = (args.native_map.stat().st_mtime_ns, args.detections.stat().st_mtime_ns)
        except FileNotFoundError:
            version = None
        if version is not None and version != previous:
            objects = mapper.update(args.native_map)
            if objects is not None:
                temporary = args.output.with_name(args.output.name+".tmp")
                temporary.write_text(json.dumps(objects, ensure_ascii=False)+"\n", encoding="utf-8")
                temporary.replace(args.output)
                previous = version
        if not args.watch:
            if previous is None:
                raise SystemExit("No matching map and pose revision")
            break
        time.sleep(.2)
