"""Publish the live RTAB-Map grid and a 3D cloud from its optimized RGB-D keyframes."""
import cv2
import time
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rtabmap_msgs.msg import MapGraph
from rtabmap_msgs.srv import GetNodeData
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def transform(points, rotation, translation):
    q = [rotation.x, rotation.y, rotation.z, rotation.w]
    rotation = Rotation.from_quat(q).as_matrix().astype(np.float32)
    result = np.empty_like(points, dtype=np.float32)
    for axis, offset in enumerate((translation.x, translation.y, translation.z)):
        result[:, axis] = (points[:, 0] * rotation[axis, 0]
                           + points[:, 1] * rotation[axis, 1]
                           + points[:, 2] * rotation[axis, 2] + offset)
    return result


def voxel_indices(points):
    cells = np.ascontiguousarray(np.floor(points / .035), dtype=np.int32)
    # Compare three integer coordinates as one key instead of sorting Python fields.
    keys = cells.view(np.dtype((np.void, 12))).ravel()
    return np.unique(keys, return_index=True)[1]


class MapDisplay(Node):
    def __init__(self):
        super().__init__("owl_map_display", parameter_overrides=[Parameter("use_sim_time", value=True)])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/owl/map_display/grid", latched)
        self.cloud_pub = self.create_publisher(PointCloud2, "/owl/map_display/cloud", latched)
        self.create_subscription(OccupancyGrid, "/rtabmap/map", self.on_grid, latched)
        self.create_subscription(MapGraph, "/rtabmap/mapGraph", self.on_graph, latched)
        self.client = self.create_client(GetNodeData, "/rtabmap/rtabmap/get_node_data")
        self.bridge = CvBridge()
        self.grid = self.cloud = self.graph = self.poses = None
        self.keyframes = {}
        self.pending = False
        self.graph_received = time.monotonic()
        self.create_timer(.05, self.update_cloud_if_ready)
        self.create_timer(1., self.publish_maps)

    def on_grid(self, grid):
        self.grid = grid
        self.grid_pub.publish(grid)

    def on_graph(self, graph):
        self.graph = graph
        self.graph_received = time.monotonic()

    def update_cloud_if_ready(self):
        if self.graph is not None:
            self.update_cloud()

    def decode(self, image, compressed, encoding):
        if len(compressed):
            return cv2.imdecode(np.frombuffer(compressed, np.uint8), cv2.IMREAD_UNCHANGED)
        return self.bridge.imgmsg_to_cv2(image, encoding) if image.width else None

    def project(self, data):
        depth = self.decode(data.right, data.right_compressed, "passthrough")
        color = self.decode(data.left, data.left_compressed, "bgr8")
        if depth is None or color is None:
            return None
        if depth.dtype == np.uint8 and depth.ndim == 3 and depth.shape[2] == 4:
            depth = depth.view("<f4").reshape(depth.shape[:2])
        elif depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * .001
        if depth.shape != color.shape[:2]:
            raise ValueError("Expected aligned OWL RGB-D frames")
        p = data.left_camera_info[0].p
        depth = depth[::4, ::4]
        v, u = np.nonzero(np.isfinite(depth) & (depth >= .3) & (depth <= 8.))
        z = depth[v, u]
        u, v = u * 4, v * 4
        points = np.column_stack(((u - p[2]) * z / p[0], (v - p[6]) * z / p[5], z))
        camera = data.local_transform[0]
        local = transform(points, camera.rotation, camera.translation).astype(np.float32)
        b, g, r = color[v, u].astype(np.uint32).T
        return local, (r << 16) | (g << 8) | b

    def load_keyframes(self, future, requested):
        self.pending = False
        try:
            nodes = {item.id: item.data for item in future.result().data}
        except Exception as error:
            self.get_logger().warn(f"RGB-D keyframe request failed: {error}")
            return
        for i in requested:
            try:
                self.keyframes[i] = self.project(nodes[i]) if i in nodes else None
            except (ValueError, IndexError, cv2.error) as error:
                self.get_logger().warn(f"Cannot project keyframe {i}: {error}")
                self.keyframes[i] = None
        self.update_cloud()

    def update_cloud(self):
        started = time.monotonic()
        poses = {i: p for i, p in zip(self.graph.poses_id, self.graph.poses) if i > 0}
        missing = [i for i in poses if i not in self.keyframes][:12]
        if missing:
            if not self.pending and self.client.service_is_ready():
                self.pending = True
                future = self.client.call_async(GetNodeData.Request(ids=missing, images=True))
                future.add_done_callback(lambda result: self.load_keyframes(result, missing))
            return
        if poses == self.poses:
            return
        self.poses = poses
        points, colors = [], []
        for i, pose in poses.items():
            if self.keyframes[i] is not None:
                local, rgb = self.keyframes[i]
                points.append(transform(local, pose.orientation, pose.position))
                colors.append(rgb)
        if not points:
            return
        points, colors = np.concatenate(points), np.concatenate(colors)
        index = voxel_indices(points)
        packed = np.empty((len(index), 4), dtype="<f4")
        packed[:, :3] = points[index]
        packed[:, 3] = colors[index].astype("<u4").view("<f4")
        self.cloud = PointCloud2(
            header=Header(frame_id=self.graph.header.frame_id), height=1, width=len(index),
            point_step=16, row_step=16 * len(index), is_dense=True, data=packed.tobytes(),
            fields=[PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(("x", "y", "z", "rgb"))],
        )
        self.cloud.header.stamp = self.get_clock().now().to_msg()
        self.cloud_pub.publish(self.cloud)
        self.get_logger().info(
            f"Cloud updated from {len(poses)} poses: {len(index)} points; "
            f"assemble={time.monotonic()-started:.3f}s graph_to_cloud={time.monotonic()-self.graph_received:.3f}s")

    def publish_maps(self):
        stamp = self.get_clock().now().to_msg()
        for msg, publisher in ((self.grid, self.grid_pub), (self.cloud, self.cloud_pub)):
            if msg is not None:
                msg.header.stamp = stamp
                publisher.publish(msg)


if __name__ == "__main__":
    rclpy.init()
    node = MapDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
