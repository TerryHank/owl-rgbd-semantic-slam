"""Check a short Nav2 goal and measure changed map payload arrival times."""
import hashlib
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rtabmap_msgs.msg import MapGraph
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

rclpy.init(args=["--ros-args", "-r", "/tf:=/owl/tf"])
node = Node("verify_owl_navigation", parameter_overrides=[Parameter("use_sim_time", value=True)])
buffer = Buffer()
listener = TransformListener(buffer, node)
maps = {}
changes = []
hashes = {}
start = time.monotonic()


def receive(name, msg):
    maps[name] = msg
    if name == "graph":
        key = tuple(msg.poses_id)
    else:
        key = hashlib.sha256(bytes(msg.data)).hexdigest()
    if hashes.get(name) != key:
        hashes[name] = key
        changes.append({"topic": name, "t": round(time.monotonic()-start, 3)})
        print(changes[-1], flush=True)


qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
for name, topic, kind in [
    ("grid", "/owl/map_display/grid", OccupancyGrid),
    ("cloud", "/owl/map_display/cloud", PointCloud2),
    ("graph", "/rtabmap/mapGraph", MapGraph),
]:
    node.create_subscription(kind, topic, lambda msg, key=name: receive(key, msg), qos)

client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
deadline = time.monotonic() + 20
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
    if len(maps) == 3 and buffer.can_transform("owl_map", "base_footprint", rclpy.time.Time()):
        break
pose = buffer.lookup_transform("owl_map", "base_footprint", rclpy.time.Time()).transform
q = pose.rotation
heading = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
grid = maps["grid"]
goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "owl_map"
goal.pose.header.stamp = node.get_clock().now().to_msg()
goal.pose.pose.orientation = q
goal.pose.pose.position.x = pose.translation.x + .65 * math.cos(heading)
goal.pose.pose.position.y = pose.translation.y + .65 * math.sin(heading)
x, y = goal.pose.pose.position.x, goal.pose.pose.position.y
u = int((x-grid.info.origin.position.x)/grid.info.resolution)
v = int((y-grid.info.origin.position.y)/grid.info.resolution)
assert 0 <= u < grid.info.width and 0 <= v < grid.info.height
assert grid.data[v*grid.info.width+u] == 0, "Test goal is not in observed free space"
assert client.wait_for_server(timeout_sec=5)
print("Navigation goal", x, y, flush=True)
future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future, timeout_sec=5)
handle = future.result()
assert handle.accepted
result = handle.get_result_async()
rclpy.spin_until_future_complete(node, result, timeout_sec=70)
if not result.done():
    cancel = handle.cancel_goal_async()
    rclpy.spin_until_future_complete(node, cancel, timeout_sec=5)
    raise RuntimeError("Navigation timed out and was canceled")
until = time.monotonic()+4
while time.monotonic()<until:
    rclpy.spin_once(node, timeout_sec=.1)
end = buffer.lookup_transform("owl_map","base_footprint",rclpy.time.Time()).transform.translation
report = {"status": result.result().status,
          "goal_error_m": math.hypot(end.x-x,end.y-y),
          "goal": [x,y], "end": [end.x,end.y], "map_updates": changes}
Path.home().joinpath("linorobot2_ws/maps/owl_navigation_check.json").write_text(json.dumps(report,indent=2))
print(json.dumps(report), flush=True)
node.destroy_node()
rclpy.shutdown()
assert report["status"] == 4 and report["goal_error_m"] < .2, report
