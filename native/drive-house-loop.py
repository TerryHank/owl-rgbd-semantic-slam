"""Drive the Gazebo-only benchmark loop and save simulator truth separately."""
import argparse
import csv
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def run(output, long_route=False):
    rclpy.init()
    node = rclpy.create_node("owl_house_loop_driver")
    command = node.create_publisher(Twist, "/cmd_vel", 10)
    pose = None
    center = (-2.65, 2.42)
    radius = 1.25
    axes = (1.84, 1.86) if long_route else (radius, radius)
    exponent = .63 if long_route else 1.

    def on_route(angle):
        return tuple(c + r*math.copysign(abs(v)**exponent, v)
                     for c, r, v in zip(center, axes, (math.cos(angle), math.sin(angle))))

    def route_angle(x, y):
        dx, dy = ((v-c)/r for v, c, r in zip((x, y), center, axes))
        return math.atan2(math.copysign(abs(dy)**(1/exponent), dy),
                          math.copysign(abs(dx)**(1/exponent), dx))
    with output.open("x", newline="") as stream:
        truth = csv.writer(stream)
        truth.writerow(("stamp", "x", "y", "yaw"))

        def measured(msg):
            nonlocal pose
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            pose = (p.x, p.y, yaw)
            truth.writerow((f"{stamp:.9f}", *pose))

        node.create_subscription(Odometry, "/ground_truth/odom", measured, qos_profile_sensor_data)

        def pump(duration):
            deadline = time.monotonic()+duration
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)

        try:
            pump(2)
            if pose is None:
                raise RuntimeError("No Gazebo truth pose")
            origin = pose
            previous = route_angle(*pose[:2])
            angle = 0.
            deadline = time.monotonic()+(180 if long_route else 105)
            while time.monotonic() < deadline:
                x, y, yaw = pose
                current = route_angle(x, y)
                change = math.remainder(current-previous, math.tau)
                angle += change
                previous = current
                remaining = math.tau-angle
                final = remaining < .055
                target = (origin[0],origin[1]) if final else on_route(current+(.16 if long_route else .22))
                heading = origin[2] if final else current + math.pi/2 + .1
                if long_route and not final:
                    heading = math.atan2(target[1]-y, target[0]-x)
                error = math.remainder(heading-yaw, math.tau)
                yaw_rate = max(-.35,min(.35, 2*error))
                dx,dy = target[0]-x,target[1]-y
                distance = math.hypot(dx,dy)
                if final and distance < .025 and abs(error) < .025:
                    print(f"Returned after {math.degrees(angle):.1f} deg, error={distance:.3f} m, yaw={math.degrees(error):.2f} deg", flush=True)
                    break
                speed = min(.23 if not final else .12, distance*1.35)
                travel = math.atan2(dy,dx)-yaw
                cmd = Twist()
                cmd.linear.x = speed*math.cos(travel)
                cmd.linear.y = speed*math.sin(travel)
                cmd.angular.z = yaw_rate
                command.publish(cmd)
                pump(.07)
            else:
                raise RuntimeError(f"Loop timed out after {math.degrees(angle):.1f} deg at {pose}")
        finally:
            command.publish(Twist())
            pump(3)
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--long", action="store_true")
    args = parser.parse_args()
    run(args.output, args.long)
