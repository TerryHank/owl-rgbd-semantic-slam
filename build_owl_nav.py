"""Adapt installed Humble Nav2 defaults for the visual ROSbot XL."""
from pathlib import Path
import yaml

config = yaml.safe_load(Path("/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml").read_text())
for key in ("amcl", "map_server", "map_saver", "robot_state_publisher"):
    config.pop(key, None)
bt = config["bt_navigator"]["ros__parameters"]
bt.update(global_frame="owl_map", robot_base_frame="base_footprint", odom_topic="/owl/visual_odom")
c = config["controller_server"]["ros__parameters"]
c.update(controller_frequency=10., min_y_velocity_threshold=.001, odom_topic="/owl/visual_odom")
c["progress_checker"].update(required_movement_radius=.15, movement_time_allowance=20.)
c["general_goal_checker"].update(xy_goal_tolerance=.12, yaw_goal_tolerance=.18)
c["FollowPath"].update(
    min_vel_x=-.10, max_vel_x=.20, min_vel_y=-.15, max_vel_y=.15,
    max_vel_theta=.35, max_speed_xy=.22, acc_lim_x=.5, acc_lim_y=.5,
    decel_lim_x=-.5, decel_lim_y=-.5, acc_lim_theta=.8, decel_lim_theta=-.8,
    vx_samples=12, vy_samples=9, vtheta_samples=12, trans_stopped_velocity=.03,
    sim_time=1.5, transform_tolerance=.5, debug_trajectory_details=False)
for name, frame in (("local_costmap", "owl_visual_odom"), ("global_costmap", "owl_map")):
    cost = config[name][name]["ros__parameters"]
    cost.update(global_frame=frame, robot_base_frame="base_footprint", robot_radius=.23,
                transform_tolerance=.5, plugins=["static_layer", "obstacle_layer", "inflation_layer"])
    cost.pop("voxel_layer", None)
    cost["static_layer"] = dict(plugin="nav2_costmap_2d::StaticLayer",
                               map_topic="/rtabmap/map", map_subscribe_transient_local=True,
                               subscribe_to_updates=True)
    cost["obstacle_layer"] = dict(
        plugin="nav2_costmap_2d::ObstacleLayer", observation_sources="owl",
        owl=dict(topic="/owl/nav/points", data_type="PointCloud2", marking=True, clearing=True,
                 min_obstacle_height=.045, max_obstacle_height=.50,
                 obstacle_min_range=.3, obstacle_max_range=4.5,
                 raytrace_min_range=.3, raytrace_max_range=4.5))
    cost["inflation_layer"] = dict(plugin="nav2_costmap_2d::InflationLayer",
                                   inflation_radius=.38, cost_scaling_factor=5.)
config["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] = False
behavior = config["behavior_server"]["ros__parameters"]
behavior.update(global_frame="owl_visual_odom", robot_base_frame="base_footprint",
                max_rotational_vel=.35, min_rotational_vel=.10, rotational_acc_lim=.8)
smooth = config["velocity_smoother"]["ros__parameters"]
smooth.update(feedback="OPEN_LOOP", odom_topic="/owl/visual_odom",
              max_velocity=[.20,.15,.35], min_velocity=[-.10,-.15,-.35],
              max_accel=[.5,.5,.8], max_decel=[-.5,-.5,-.8])
out = Path.home() / "linorobot2_ws/display/rosbot_xl/owl_nav.yaml"
out.write_text(yaml.safe_dump(config, sort_keys=False))
print(out)
