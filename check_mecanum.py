"""Exercise the simulated base through cmd_vel and record odometry response."""
import json
import math
from pathlib import Path
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, JointState
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node("check_rosbot_xl")
node.set_parameters([Parameter("use_sim_time", value=True)])
publisher = node.create_publisher(Twist, "/cmd_vel", 10)
samples = {}
subscriptions = []
for topic, kind in [("/odom/unfiltered", Odometry), ("/odom", Odometry),
                    ("/scan", LaserScan), ("/owl/rgbd/image", Image),
                    ("/joint_states", JointState), ("/ground_truth/odom", Odometry)]:
    subscriptions.append(node.create_subscription(
        kind, topic, lambda msg, key=topic: samples.update({key: msg}), qos_profile_sensor_data))


def wait(seconds, command=None):
    start = node.get_clock().now().nanoseconds
    deadline = time.monotonic() + 90
    while (node.get_clock().now().nanoseconds - start) / 1e9 < seconds:
        assert time.monotonic() < deadline, "Simulation clock stopped"
        if command is not None:
            publisher.publish(command)
        rclpy.spin_once(node, timeout_sec=.04)


def pose():
    p = samples["/ground_truth/odom"].pose.pose
    q = p.orientation
    return p.position.x, p.position.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


results = []
zero = Twist()
try:
    required = ("/odom", "/odom/unfiltered", "/scan", "/owl/rgbd/image",
                "/joint_states", "/ground_truth/odom")
    deadline = time.monotonic() + 30
    while not all(k in samples for k in required):
        assert time.monotonic() < deadline, f"Missing topics: {set(required) - samples.keys()}"
        rclpy.spin_once(node, timeout_sec=.1)
    wait(1)
    for name, vx, vy, wz in [("forward", .12, 0, 0), ("left", 0, .12, 0),
                              ("rotate", 0, 0, .3), ("diagonal", .08, .08, 0)]:
        before = pose()
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = float(vx), float(vy), float(wz)
        wait(2, command)
        joint = samples["/joint_states"]
        velocities = dict(zip(joint.name, joint.velocity))
        wait(1., zero)
        after = pose()
        dx, dy = after[0]-before[0], after[1]-before[1]
        local_x = math.cos(before[2])*dx + math.sin(before[2])*dy
        local_y = -math.sin(before[2])*dx + math.cos(before[2])*dy
        yaw = math.atan2(math.sin(after[2]-before[2]), math.cos(after[2]-before[2]))
        passed = ((vx == 0 or local_x > .10) and (vy == 0 or local_y > .10)
                  and (wz == 0 or yaw > .4))
        results.append(dict(test=name, dx=round(local_x,4), dy=round(local_y,4),
                            yaw=round(yaw,4), wheel_velocities=velocities, passed=passed))
    wait(.5, zero)
    stopped = samples["/odom/unfiltered"].twist.twist
    assert abs(stopped.linear.x) < .005 and abs(stopped.linear.y) < .005 and abs(stopped.angular.z) < .005
    report = dict(results=results, stopped=True)
    (Path.home()/"linorobot2_ws/display/rosbot_xl/classic-verification.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report))
    assert all(r["passed"] for r in results)
finally:
    wait(.3, zero)
    node.destroy_node()
    rclpy.shutdown()
