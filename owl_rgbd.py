"""Register simulated OWL color to the depth grid using nominal coincident optics."""
import cv2
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from rtabmap_msgs.msg import RGBDImage


class OwlRgbd(Node):
    def __init__(self):
        super().__init__("owl_rgbd_registration")
        self.bridge = CvBridge()
        self.color = self.create_publisher(Image, "/owl/rgbd/image", 5)
        self.depth = self.create_publisher(Image, "/owl/rgbd/depth", 5)
        self.info = self.create_publisher(CameraInfo, "/owl/rgbd/camera_info", 5)
        self.packet = self.create_publisher(RGBDImage, "/owl/rgbd/packet", 5)
        self.maps = None
        self.sync = ApproximateTimeSynchronizer([
            Subscriber(self, Image, "/owl/color/stream/image_raw", qos_profile=qos_profile_sensor_data),
            Subscriber(self, Image, "/owl/depth/stream/depth/image_raw", qos_profile=qos_profile_sensor_data),
            Subscriber(self, CameraInfo, "/owl/color/stream/camera_info", qos_profile=qos_profile_sensor_data),
            Subscriber(self, CameraInfo, "/owl/depth/stream/camera_info", qos_profile=qos_profile_sensor_data),
        ], queue_size=12, slop=.012)
        self.sync.registerCallback(self.publish_registered)

    def publish_registered(self, color, depth, color_info, depth_info):
        if self.maps is None:
            u, v = np.meshgrid(np.arange(depth.width, dtype=np.float32),
                               np.arange(depth.height, dtype=np.float32))
            c, d = color_info.p, depth_info.p
            self.maps = ((u - d[2]) * c[0] / d[0] + c[2],
                         (v - d[6]) * c[5] / d[5] + c[6])
        raw = self.bridge.imgmsg_to_cv2(color, "rgb8")
        aligned = cv2.remap(raw, *self.maps, interpolation=cv2.INTER_LINEAR)
        meters = self.bridge.imgmsg_to_cv2(depth, "32FC1").copy()
        meters[(~np.isfinite(meters)) | (meters < .3) | (meters > 8.) |
               (self.maps[0] < 0) | (self.maps[0] > color.width - 1) |
               (self.maps[1] < 0) | (self.maps[1] > color.height - 1)] = np.nan
        image = self.bridge.cv2_to_imgmsg(aligned, "rgb8", header=depth.header)
        aligned_depth = self.bridge.cv2_to_imgmsg(meters, "32FC1", header=depth.header)
        depth_info.header = depth.header
        p = depth_info.p
        depth_info.k = [p[0], 0., p[2], 0., p[5], p[6], 0., 0., 1.]
        self.info.publish(depth_info)
        self.color.publish(image)
        self.depth.publish(aligned_depth)
        # Transport the matched pair together; depth remains lossless floating-point PNG.
        rgb_bytes = cv2.imencode(".png", cv2.cvtColor(aligned, cv2.COLOR_RGB2BGR),
                                [cv2.IMWRITE_PNG_COMPRESSION, 1])[1]
        depth_bytes = cv2.imencode(".png", meters.view(np.uint8).reshape((*meters.shape, 4)),
                                  [cv2.IMWRITE_PNG_COMPRESSION, 1])[1]
        self.packet.publish(RGBDImage(
            header=depth.header, rgb_camera_info=depth_info, depth_camera_info=depth_info,
            rgb_compressed=CompressedImage(header=depth.header, format="png", data=rgb_bytes.tobytes()),
            depth_compressed=CompressedImage(header=depth.header, format="png", data=depth_bytes.tobytes())))


if __name__ == "__main__":
    rclpy.init()
    node = OwlRgbd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
