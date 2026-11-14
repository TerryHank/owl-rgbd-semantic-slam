from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    directory = Path(__file__).parent
    services = [
        ("nav2_controller", "controller_server"),
        ("nav2_smoother", "smoother_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
        ("nav2_waypoint_follower", "waypoint_follower"),
        ("nav2_velocity_smoother", "velocity_smoother"),
    ]
    actions = [Node(package="rtabmap_util", executable="point_cloud_xyz", name="owl_nav_depth",
                    parameters=[{"use_sim_time": True, "decimation": 4, "max_depth": 4.5,
                                 "min_depth": .3, "approx_sync": False}],
                    remappings=[("depth/image", "/owl/rgbd/depth"),
                                ("depth/camera_info", "/owl/rgbd/camera_info"),
                                ("cloud", "/owl/nav/points")])]
    for package, name in services:
        remaps = [("/tf", "/owl/tf")]
        if name in ("controller_server", "behavior_server"):
            remaps.append(("cmd_vel", "cmd_vel_nav"))
        if name == "velocity_smoother":
            remaps += [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]
        actions.append(Node(package=package, executable=name, name=name,
                            parameters=[str(directory / "owl_nav.yaml"), {"use_sim_time": True}],
                            remappings=remaps, output="screen"))
    actions.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                        name="lifecycle_manager_navigation",
                        parameters=[{"use_sim_time": True, "autostart": True,
                                     "node_names": [name for _,name in services]}], output="screen"))
    actions.append(ExecuteProcess(cmd=["python3", str(directory / "owl_nav_goal.py")], output="screen"))
    return LaunchDescription(actions)
