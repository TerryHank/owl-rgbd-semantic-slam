import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

GRID_ARGS = (
    "--Grid/Sensor 1 --Grid/3D false --Grid/CellSize 0.05 "
    "--Grid/NormalsSegmentation false --Grid/MaxGroundHeight 0.045 "
    "--Grid/MaxObstacleHeight 0.50 --Grid/RangeMin 0.3 --Grid/RangeMax 4.5 "
    "--Grid/DepthDecimation 2 --Grid/RayTracing true"
)
ODOM_ARGS = (
    "--Odom/Strategy 1 --Odom/KeyFrameThr 0.65 --Vis/CorType 1 "
    "--Vis/EstimationType 0 --Vis/InlierDistance 0.05 "
    "--Vis/CorFlowWinSize 24 --Vis/CorFlowMaxLevel 4 --Vis/GridRows 3 --Vis/GridCols 4 "
    "--Reg/Force3DoF true"
)

def generate_launch_description():
    vendor_launch = Path(get_package_share_directory("rtabmap_launch")) / "launch/rtabmap.launch.py"
    return LaunchDescription([
        DeclareLaunchArgument("database_path"),
        DeclareLaunchArgument("viz", default_value="true"),
        ExecuteProcess(cmd=["python3", str(Path(__file__).with_name("owl_map_display.py"))], output="screen"),
        GroupAction([
            # Keep the visual base TF separate from the chassis EKF's odom TF.
            SetRemap(src="/tf", dst="/owl/tf"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(vendor_launch)),
                launch_arguments={
                    "use_sim_time": "true",
                    "frame_id": "base_footprint",
                    "vo_frame_id": "owl_visual_odom",
                    "map_frame_id": "owl_map",
                    "visual_odometry": "true",
                    "wait_imu_to_init": "false",
                    "imu_topic": "/owl/unused_imu",
                    "subscribe_scan": "false",
                    "subscribe_scan_cloud": "false",
                    "rgb_topic": "/owl/rgbd/image",
                    "depth_topic": "/owl/rgbd/depth",
                    "camera_info_topic": "/owl/rgbd/camera_info",
                    "odom_topic": "/owl/visual_odom",
                    "rgbd_sync": "false",
                    "subscribe_rgbd": "true",
                    "rgbd_topic": "/owl/rgbd/packet",
                    "approx_rgbd_sync": "false",
                    "approx_sync": "false",
                    "rtabmap_viz": LaunchConfiguration("viz"),
                    "database_path": LaunchConfiguration("database_path"),
                    "odom_args": os.environ.get("OWL_ODOM_ARGS", ODOM_ARGS),
                    "rtabmap_args": (
                        "--Reg/Strategy 0 --Optimizer/GravitySigma 0 --RGBD/ProximityBySpace false "
                        "--Reg/Force3DoF true --Vis/EstimationType 0 --Vis/InlierDistance 0.05 "
                        + GRID_ARGS + " " + os.environ.get("OWL_SLAM_ARGS", "")
                    ),
                }.items()),
        ]),
    ])
