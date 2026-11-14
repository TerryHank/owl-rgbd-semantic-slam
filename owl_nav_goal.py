"""Forward Foxglove PoseStamped goals to Nav2 and expose cancellation."""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger


class Goals(Node):
    def __init__(self):
        super().__init__("owl_nav_goal", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.goal_handle = None
        self.create_subscription(PoseStamped, "/goal_pose", self.goal, 10)
        self.create_subscription(PoseStamped, "/move_base_simple/goal", self.goal, 10)
        self.create_service(Trigger, "/owl/navigation/cancel", self.cancel)

    def goal(self, pose):
        if not self.client.server_is_ready():
            self.get_logger().warn("Nav2 is not ready yet")
            return
        if pose.header.frame_id != "owl_map":
            self.get_logger().error("Select owl_map as the goal frame")
            return
        pose.header.stamp = self.get_clock().now().to_msg()
        future = self.client.send_goal_async(NavigateToPose.Goal(pose=pose))
        future.add_done_callback(self.accepted)

    def accepted(self, future):
        self.goal_handle = future.result()
        self.get_logger().info(f"Navigation goal accepted={self.goal_handle.accepted}")

    def cancel(self, request, response):
        response.success = self.goal_handle is not None and self.goal_handle.accepted
        if response.success:
            self.goal_handle.cancel_goal_async()
        response.message = "Cancel requested" if response.success else "No goal submitted through this relay"
        return response


rclpy.init()
node = Goals()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()
