"""Serve complete native OWL map revisions over a local Foxglove WebSocket."""
import argparse
import asyncio
import base64
import json
from pathlib import Path
import struct
import time

import websockets


TOPICS = ("frame", "grid", "cloud", "status")
SCHEMAS = ("FrameTransform", "Grid", "PointCloud", "Log")
ZERO = {"x": 0, "y": 0, "z": 0}
ROTATION = {**ZERO, "w": 1}
FRAME = "owl_map"
COLORS = [
    {"name": name, "offset": offset, "type": 1}
    for offset, name in enumerate(("red", "green", "blue", "alpha"))
]
GRID_COLORS = [
    bytes([105 if value == 255 else round(249 - min(value, 100) * 2.34)] * 3 + [255])
    for value in range(256)
]


def load_map(path, semantics=None):
    with path.open("rb") as source:
        magic, rev, sim_time, frames, tracked, nodes, loop, moved, rotated, width, height, x, y, cell, points, slam_ms, map_ms = (
            source.readline().decode("ascii").split()
        )
        if magic != "OWL_MAP1":
            raise ValueError("Unexpected native map format")
        width, height, points = int(width), int(height), int(points)
        cells = source.read(width * height)
        cloud = source.read()
    if len(cells) != width * height or len(cloud) != points * 16:
        raise ValueError("Incomplete native map revision")
    timestamp = time.time_ns()
    stamp = {"sec": timestamp // 1_000_000_000, "nsec": timestamp % 1_000_000_000}
    shared = {"timestamp": stamp, "frame_id": FRAME}
    detail = (f"rev={rev} sim={sim_time} frames={frames} tracked={tracked} nodes={nodes} "
              f"points={points} loop_target={loop} shift_m={moved} yaw_deg={rotated} "
              f"slam_ms={slam_ms} maps_ms={map_ms}")
    messages = (
        {"timestamp": stamp, "parent_frame_id": FRAME, "child_frame_id": "owl_map_origin",
         "translation": ZERO, "rotation": ROTATION},
        {**shared, "pose": {"position": {"x": float(x), "y": float(y), "z": 0}, "orientation": ROTATION},
         "column_count": width, "cell_size": {"x": float(cell), "y": float(cell)},
         "row_stride": width * 4, "cell_stride": 4, "fields": COLORS,
         "data": base64.b64encode(b"".join(GRID_COLORS[value] for value in cells)).decode("ascii")},
        {**shared, "pose": {"position": ZERO, "orientation": ROTATION}, "point_stride": 16,
         "fields": [{"name": axis, "offset": i * 4, "type": 7} for i, axis in enumerate("xyz")]
         + [{**field, "offset": field["offset"] + 12} for field in COLORS],
         "data": base64.b64encode(cloud).decode("ascii")},
        {"timestamp": stamp, "level": 2, "name": "owl_slam", "message": detail},
    )
    if semantics is not None:
        observations = json.loads(semantics.read_text(encoding="utf-8")) if semantics.exists() else []
        observations = [item for item in observations if item.get("map_revision", int(rev)) == int(rev)]
        entities = []
        for item in observations:
            pose = {"position": {"x": item["x"], "y": item["y"], "z": item.get("z", .18)},
                    "orientation": ROTATION}
            label = {"pose": pose, "billboard": True, "font_size": .18, "scale_invariant": False,
                     "color": {"r": .45, "g": 1, "b": .80, "a": 1},
                                 "text": item["label"]}
            entities.append({**shared, "id": str(item["node_id"]), "lifetime": {"sec": 0, "nsec": 0},
                             "frame_locked": True, "texts": [label], "spheres": [
                                 {"pose": pose, "size": {"x": .07, "y": .07, "z": .07},
                                  "color": label["color"]}]})
        messages += ({"entities": entities, "deletions": [{"timestamp": stamp, "type": 1}]},
                     {"observations": observations})
    return timestamp, detail, tuple(json.dumps(message, separators=(",", ":")).encode() for message in messages)


async def serve(path, schema_dir, port, semantics=None):
    topics = TOPICS + (("semantic_markers",) if semantics else ())
    schemas = SCHEMAS + (("SceneUpdate",) if semantics else ())
    channels = [
        {"id": i, "topic": f"/owl/native/{topic}", "encoding": "json",
         "schemaName": f"foxglove.{schema}", "schema": (schema_dir / f"{schema}.json").read_text()}
        for i, (topic, schema) in enumerate(zip(topics, schemas))
    ]
    if semantics:
        channels.append({"id": len(channels), "topic": "/owl/native/semantics", "encoding": "json",
                         "schemaName": "owl.Semantics",
                         "schema": json.dumps({"type": "object", "properties": {
                             "observations": {"type": "array", "items": {"type": "object", "properties": {
                                 "node_id": {"type": "integer"}, "stamp": {"type": "number"},
                                 "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"},
                                 "yaw": {"type": "number"},
                                 "label": {"type": "string"}, "class_name": {"type": "string"},
                                 "caption": {"type": "string"}}}}}})})
    current, file_revision = None, None

    async def refresh():
        nonlocal current, file_revision
        while True:
            try:
                stat = path.stat()
            except FileNotFoundError:
                pass
            else:
                semantic_revision = semantics.stat().st_mtime_ns if semantics and semantics.exists() else 0
                updated = (stat.st_ino, stat.st_mtime_ns, semantic_revision)
                if updated != file_revision:
                    current = await asyncio.to_thread(load_map, path, semantics)
                    file_revision = updated
                    print(current[1], flush=True)
            await asyncio.sleep(.04)

    async def connection(ws, _path):
        if ws.subprotocol is None:
            await ws.close(1002, "Foxglove subprotocol required")
            return
        await ws.send(json.dumps({"op": "serverInfo", "name": "OWL Native RTAB-Map Live",
                                  "capabilities": [], "supportedEncodings": ["json"]}))
        await ws.send(json.dumps({"op": "advertise", "channels": channels}))
        subscriptions = {}

        async def receive():
            async for raw in ws:
                msg = json.loads(raw)
                if msg["op"] == "subscribe":
                    for sub in msg["subscriptions"]:
                        subscriptions[sub["id"]] = (sub["channelId"], 0)
                elif msg["op"] == "unsubscribe":
                    for sub in msg["subscriptionIds"]:
                        subscriptions.pop(sub, None)

        async def send():
            while True:
                if current:
                    timestamp, _, messages = current
                    for sub, (channel, last) in sorted(subscriptions.copy().items(), key=lambda item: item[1][0]):
                        if timestamp != last and subscriptions.get(sub) == (channel, last):
                            await ws.send(struct.pack("<BIQ", 1, sub, timestamp) + messages[channel])
                            if subscriptions.get(sub) == (channel, last):
                                subscriptions[sub] = channel, timestamp
                await asyncio.sleep(.02)

        receiver, sender = asyncio.create_task(receive()), asyncio.create_task(send())
        try:
            await asyncio.wait((receiver, sender), return_when=asyncio.FIRST_COMPLETED)
        finally:
            receiver.cancel()
            sender.cancel()
            await asyncio.gather(receiver, sender, return_exceptions=True)

    async with websockets.serve(connection, "127.0.0.1", port,
                                subprotocols=["foxglove.sdk.v1", "foxglove.websocket.v1"],
                                compression=None):
        await refresh()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("map", type=Path)
    parser.add_argument("schemas", type=Path)
    parser.add_argument("--port", type=int, default=8768)
    parser.add_argument("--semantics", type=Path)
    args = parser.parse_args()
    asyncio.run(serve(args.map, args.schemas, args.port, args.semantics))
