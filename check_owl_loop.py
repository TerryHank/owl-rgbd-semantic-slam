"""Drive a loop in Gazebo using simulator pose for steering and evaluation only."""
import json
import math
from pathlib import Path
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info, MapGraph
from rtabmap_msgs.srv import GetMap


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def xyz(p):
    return [p.x, p.y, p.z]


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def xy_yaw(msg):
    p = msg.pose.pose
    return p.position.x, p.position.y, yaw(p.orientation)


def correction(graph):
    t = graph.map_to_odom
    return [*xyz(t.translation), yaw(t.rotation)]


rclpy.init()
node = rclpy.create_node("owl_loop_test")
latest = {}
loops = []


def info(msg):
    latest["info"] = msg
    if msg.loop_closure_id > 0:
        loop = [msg.ref_id, msg.loop_closure_id]
        if loop not in loops:
            loops.append(loop)
            print("Visual loop closure", loop, flush=True)


for topic, msg_type, key in [
    ("/ground_truth/odom", Odometry, "truth"),
    ("/owl/visual_odom", Odometry, "vo"),
    ("/rtabmap/mapGraph", MapGraph, "graph"),
]:
    node.create_subscription(msg_type, topic, lambda msg, k=key: latest.update({k: msg}),
                             qos_profile_sensor_data)
node.create_subscription(Info, "/rtabmap/info", info, qos_profile_sensor_data)
pub = node.create_publisher(Twist, "/cmd_vel", 10)
zero = Twist()
cx, cy, radius = -1.6, -.8, 1.1


def pump(seconds, cmd=None):
    end = time.monotonic() + seconds
    if cmd is not None:
        pub.publish(cmd)
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.025)


try:
    deadline = time.monotonic() + 30
    while not all(k in latest for k in ("truth", "vo", "graph", "info")):
        assert time.monotonic() < deadline, list(latest)
        pump(.1, zero)
    pump(2., zero)
    initial = xy_yaw(latest["truth"])
    initial_vo = xy_yaw(latest["vo"])
    initial_correction = correction(latest["graph"])
    last_angle = math.atan2(initial[1]-cy, initial[0]-cx)
    angle = 0.
    distance = 0.
    last_xy = initial[:2]
    last_print = 0.
    deadline = time.monotonic() + 220
    print("Loop start", initial, "first graph", list(latest["graph"].poses_id), flush=True)
    while angle < 2 * math.pi - .01:
        assert time.monotonic() < deadline, "Loop timed out"
        x, y, heading = xy_yaw(latest["truth"])
        a = math.atan2(y-cy, x-cx)
        angle += wrap(a-last_angle)
        last_angle = a
        distance += math.dist(last_xy, (x,y))
        last_xy = (x,y)
        ahead = a + .21
        target = math.atan2(cy+radius*math.sin(ahead)-y, cx+radius*math.cos(ahead)-x)
        error = wrap(target-heading)
        speed = min(.15, max(.045, .3 * (2*math.pi-angle)))
        command = Twist()
        command.linear.x = speed * math.cos(error)
        command.linear.y = speed * math.sin(error)
        command.angular.z = max(-.25, min(.25, 2*wrap(initial[2]-heading)))
        if angle-last_print > math.pi/2:
            print("Loop angle", round(angle,2), "world pose", (x,y,heading), flush=True)
            last_print = angle
        pump(.09, command)
    # Match the initial view while stopped at the end of the driven circle.
    for _ in range(60):
        err = wrap(initial[2] - xy_yaw(latest["truth"])[2])
        if abs(err) < .018:
            break
        cmd = Twist()
        cmd.angular.z = max(-.30, min(.30, 1.8*err))
        pump(.10, cmd)
    for _ in range(10):
        pump(.25, zero)
    end = xy_yaw(latest["truth"])
    graph = latest["graph"]
    map_correction = correction(graph)
    get_map = node.create_client(GetMap, "/rtabmap/rtabmap/get_map_data")
    graphs = {}
    for optimized in (False, True):
        request = GetMap.Request(global_map=True, optimized=optimized, graph_only=True)
        future = get_map.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=12)
        result = future.result()
        if result is not None:
            graphs[optimized] = {i: xyz(p.position) for i,p in
                                 zip(result.data.graph.poses_id,result.data.graph.poses)}
    drifts = {}
    for key, poses in graphs.items():
        if len(poses) >= 2:
            drifts[str(key)] = math.dist(poses[min(poses)], poses[max(poses)])
    corrections = [math.dist(graphs[False][i], graphs[True][i])
                   for i in graphs.get(False, {}) if i in graphs.get(True, {})]
    end_vo = xy_yaw(latest["vo"])
    result = {
        "start_world": initial, "end_world": end, "distance_m": distance,
        "return_error_m": math.dist(initial[:2], end[:2]),
        "return_yaw_deg": math.degrees(abs(wrap(initial[2]-end[2]))),
        "visual_odom_return_error_m": math.dist(initial_vo[:2],end_vo[:2]),
        "world_loop_angle_rad": angle, "live_visual_loop_closures": loops,
        "graph_nodes": len(graph.poses_id),
        "links_by_type": {str(k):len([x for x in graph.links if x.type == k])
                          for k in set(x.type for x in graph.links)},
        "map_to_visual_odom_start": initial_correction,
        "map_to_visual_odom_end": map_correction,
        "first_last_node_gap_m_unoptimized_optimized": drifts,
        "max_graph_position_correction_m": max(corrections,default=None),
    }
    print(json.dumps(result, indent=2), flush=True)
    (Path.home() / "linorobot2_ws/maps/owl_loop_result.json").write_text(json.dumps(result,indent=2))
finally:
    for _ in range(4):
        pump(.10, zero)
    node.destroy_node()
    rclpy.shutdown()
