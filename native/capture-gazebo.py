"""ROS test adapter for recording or streaming synchronized OWL RGB-D."""
import argparse
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rtabmap_msgs.msg import RGBDImage
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", help="new recording path, or - to stream on stdout")
    parser.add_argument("--seconds", type=float, help="capture duration (default: 75 for files)")
    parser.add_argument("--imu-csv", type=Path, help="record the simulated IMU on the same clock")
    parser.add_argument("--imu-frame", help="physical IMU frame if Gazebo reports the reduced base link")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("capture_owl_native")
    tf = Buffer()
    static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def static_transforms(message):
        for transform in message.transforms:
            tf.set_transform_static(transform, "robot_state_publisher")

    node.create_subscription(TFMessage, "/tf_static", static_transforms, static_qos)
    count, previous = 0, -1.
    live = args.output == "-"
    output = sys.stdout.buffer if live else Path(args.output).open("xb")
    imu_output = args.imu_csv.open("x", buffering=1) if args.imu_csv else None
    imu_header = False

    def inertial(msg):
        nonlocal imu_header
        if not imu_header:
            frame = args.imu_frame or msg.header.frame_id
            if not tf.can_transform("base_footprint", frame, rclpy.time.Time()):
                return
            t = tf.lookup_transform("base_footprint", frame, rclpy.time.Time()).transform
            q = t.rotation
            angles = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
            imu_output.write("IMU1 "+" ".join(map(str, (t.translation.x, t.translation.y, t.translation.z, *angles)))+"\n")
            imu_header = True
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        w, a = msg.angular_velocity, msg.linear_acceleration
        imu_output.write(f"{stamp:.9f} {w.x} {w.y} {w.z} {a.x} {a.y} {a.z}\n")

    if imu_output:
        node.create_subscription(Imu, "/imu/data", inertial, qos_profile_sensor_data)

    def frame(msg):
        nonlocal count, previous
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp <= previous:
            return
        info = msg.rgb_camera_info
        if not count:
            if not tf.can_transform("base_footprint", msg.header.frame_id, rclpy.time.Time()):
                return
            transform = tf.lookup_transform("base_footprint", msg.header.frame_id, rclpy.time.Time()).transform
            p, q = transform.translation, transform.rotation
            rpy = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
            vals = [info.width, info.height, info.p[0], info.p[5], info.p[2], info.p[6], p.x, p.y, p.z, *rpy]
        color = cv2.imdecode(np.frombuffer(msg.rgb_compressed.data, np.uint8), cv2.IMREAD_COLOR)
        depth = cv2.imdecode(np.frombuffer(msg.depth_compressed.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if color is None or depth is None or depth.shape != (info.height, info.width, 4):
            raise ValueError("Expected simulated RGB PNG and packed float-depth PNG")
        if not count:
            output.write(("OWL1 " + " ".join(map(str, vals)) + "\n").encode())
            print("OWL capture started", file=sys.stderr, flush=True)
        output.write(f"{stamp:.9f}\n".encode())
        output.write(color.tobytes())
        output.write(depth.tobytes())
        if live:
            output.flush()
        previous = stamp
        count += 1

    node.create_subscription(RGBDImage, "/owl/rgbd/packet", frame, qos_profile_sensor_data)
    try:
        duration = args.seconds if args.seconds is not None else (float("inf") if live else 75)
        until = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        if not live:
            output.close()
        if imu_output:
            imu_output.close()
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"Sent {count} synchronized frames to {args.output}", file=sys.stderr, flush=True)
    return 0 if count else 1


if __name__ == "__main__":
    raise SystemExit(main())
