from pathlib import Path

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ws = Path.home() / "linorobot2_ws"
    out = ws / "display/rosbot_xl"
    turtlebot = ws / "vendor/turtlebot3_simulations/turtlebot3_gazebo"
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=str(turtlebot / "worlds/turtlebot3_house.world")),
        DeclareLaunchArgument("x", default_value="-2.0"),
        DeclareLaunchArgument("y", default_value="-0.5"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        AppendEnvironmentVariable("GAZEBO_MODEL_PATH", str(turtlebot / "models"), prepend=True),
        AppendEnvironmentVariable("GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models"),
        ExecuteProcess(cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so",
                            "-s", "libgazebo_ros_init.so", LaunchConfiguration("world")],
                       additional_env={"GAZEBO_MODEL_DATABASE_URI": ""}, output="screen"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": (out / "classic.urdf").read_text(), "use_sim_time": True}]),
        ExecuteProcess(cmd=["python3", str(out / "owl_rgbd.py")], output="screen"),
        Node(package="gazebo_ros", executable="spawn_entity.py",
             arguments=["-entity", "rosbot_xl", "-topic", "robot_description",
                        "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"),
                        "-z", ".01", "-Y", LaunchConfiguration("yaw")],
             output="screen"),
        TimerAction(period=5., actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster", "rosbot_xl_base_controller",
                            "--controller-manager-timeout", "60"], output="screen"),
        ]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node",
             parameters=[str(ws / "src/linorobot2/linorobot2_base/config/ekf.yaml"),
                         {"use_sim_time": True}],
             remappings=[("odometry/filtered", "odom")], output="screen"),
    ])
