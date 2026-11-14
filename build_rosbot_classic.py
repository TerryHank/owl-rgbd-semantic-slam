"""Port the original Husarion wheel drive and friction model to Gazebo Classic."""
import copy
import json
from math import radians, tan
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml

ws = Path.home() / "linorobot2_ws"
out = ws / "display/rosbot_xl"
vendor = ws / "vendor/rosbot_xl_ros"
config = yaml.safe_load((vendor / "rosbot_xl_controller/config/mecanum_drive_controller.yaml").read_text())
for section in config.values():
    section["ros__parameters"]["use_sim_time"] = True
config["/**/rosbot_xl_base_controller"]["ros__parameters"]["base_frame_id"] = "base_footprint"
(out / "controllers.yaml").write_text(yaml.safe_dump(config))
robot = ET.fromstring(subprocess.check_output([
    "xacro", str(vendor / "rosbot_xl_description/urdf/rosbot_xl.urdf.xacro"),
    "use_sim:=true", "mecanum:=true", "lidar_model:=None", "camera_model:=None",
    f"controller_config_file:={out / 'controllers.yaml'}"]))

ET.SubElement(robot, "link", name="base_footprint")
joint = ET.SubElement(robot, "joint", name="footprint_to_base", type="fixed")
ET.SubElement(joint, "parent", link="base_footprint")
ET.SubElement(joint, "child", link="base_link")
hardware = robot.find("ros2_control/hardware/plugin")
hardware.text = "gazebo_ros2_control/GazeboSystem"
for sensor in robot.findall("ros2_control/sensor"):
    robot.find("ros2_control").remove(sensor)
for gazebo in list(robot.findall("gazebo")):
    if gazebo.find("plugin") is not None or gazebo.get("reference") == "imu_link":
        robot.remove(gazebo)
    elif gazebo.get("reference", "").endswith("_wheel_link"):
        robot.remove(gazebo)
for side in ("fl", "fr", "rl", "rr"):
    gazebo = ET.SubElement(robot, "gazebo", reference=f"{side}_wheel_link")
    ET.SubElement(gazebo, "mu1").text = "1.0"
    ET.SubElement(gazebo, "mu2").text = "0.0"

gazebo = ET.SubElement(robot, "gazebo")
control = ET.SubElement(gazebo, "plugin", name="gazebo_ros2_control", filename="libgazebo_ros2_control.so")
ET.SubElement(control, "parameters").text = str(out / "controllers.yaml")
ros = ET.SubElement(control, "ros")
ET.SubElement(ros, "remapping").text = "rosbot_xl_base_controller/cmd_vel_unstamped:=cmd_vel"
ET.SubElement(ros, "remapping").text = "rosbot_xl_base_controller/odom:=odom/unfiltered"
ET.SubElement(gazebo, "plugin", name="mecanum_friction", filename="libmecanum_friction.so")
truth = ET.SubElement(gazebo, "plugin", name="ground_truth", filename="libgazebo_ros_p3d.so")
ET.SubElement(ET.SubElement(truth, "ros"), "remapping").text = "odom:=ground_truth/odom"
ET.SubElement(truth, "body_name").text = "base_footprint"
ET.SubElement(truth, "frame_name").text = "world"
ET.SubElement(truth, "update_rate").text = "50"

lino = ET.fromstring(subprocess.check_output([
    "xacro", str(ws / "src/linorobot2/linorobot2_description/urdf/robots/mecanum.urdf.xacro")]))
sensor_names = {link.get("name") for link in lino.findall("link")
                if link.get("name") not in ("base_link", "base_footprint", "imu_link")
                and "wheel" not in link.get("name")}
for element in lino:
    if element.tag == "link" and element.get("name") in sensor_names:
        robot.append(copy.deepcopy(element))
    elif element.tag == "joint" and element.find("child").get("link") in sensor_names:
        element = copy.deepcopy(element)
        if element.find("parent").get("link") == "base_link":
            element.find("parent").set("link", "body_link")
        positions = {"laser": "-0.0475 0 0.15", "camera_link": "0.1775 0 0.063",
                     "rear_rgb_camera_link": "-0.179 0 0.063"}
        child = element.find("child").get("link")
        if child in positions:
            element.find("origin").set("xyz", positions[child])
        robot.append(element)
    elif element.tag == "gazebo" and element.get("reference") in sensor_names | {"imu_link"}:
        robot.append(copy.deepcopy(element))

# Nominal rectilinear optics from the V1.5 specification table. The 40 mm
# structured-light baseline is not a calibrated RGB-to-depth translation.
depth_gazebo = next(block for block in robot.findall("gazebo[@reference='camera_link']")
                    if block.find("sensor") is not None)
depth = depth_gazebo.find("sensor")
color = copy.deepcopy(depth)
depth.set("name", "owl_depth")
color.attrib.update(name="owl_color", type="camera")

for sensor, name, hfov, vfov, near in [
    (depth, "depth", 72, 50, .3),
    (color, "color", 78, 48, .05),
]:
    sensor.find("update_rate").text = "25"
    camera = sensor.find("camera")
    camera.set("name", name)
    camera.find("horizontal_fov").text = str(radians(hfov))
    camera.find("image/width").text = "640"
    camera.find("image/height").text = "400"
    clip = ET.SubElement(camera, "clip")
    ET.SubElement(clip, "near").text = str(near)
    ET.SubElement(clip, "far").text = "8" if name == "depth" else "100"
    lens = ET.SubElement(camera, "lens")
    intrinsics = ET.SubElement(lens, "intrinsics")
    values = {"fx": 320 / tan(radians(hfov / 2)), "fy": 200 / tan(radians(vfov / 2)),
              "cx": 320, "cy": 200, "s": 0}
    plugin = sensor.find("plugin")
    plugin.clear()
    plugin.attrib.update(name=f"owl_{name}_camera", filename="libgazebo_ros_camera.so")
    ros = ET.SubElement(plugin, "ros")
    ET.SubElement(ros, "namespace").text = f"/owl/{name}"
    ET.SubElement(plugin, "camera_name").text = "stream"
    ET.SubElement(plugin, "frame_name").text = "camera_depth_link"
    for key, value in values.items():
        ET.SubElement(intrinsics, key).text = str(value)
        if key != "s":
            ET.SubElement(plugin, "P_" + key).text = str(value)
    ET.SubElement(plugin, "min_depth").text = ".3"
    ET.SubElement(plugin, "max_depth").text = "8"
depth_gazebo.append(color)
camera_link = robot.find("link[@name='camera_link']")
for box in camera_link.findall(".//box"):
    box.set("size", ".030 .090 .025")

ET.indent(robot)
ET.ElementTree(robot).write(out / "classic.urdf", encoding="utf-8", xml_declaration=False)

world = ET.parse(ws / "worlds/playground.world")
world.find(".//gui/camera/pose").text = "0.85 -1.05 0.72 0 0.43 2.25"
world.write(out / "classic.world", encoding="utf-8", xml_declaration=True)
print("Generated Gazebo Classic model with original continuous wheel joints and ros2_control")
(out / "SOURCE.json").write_text(json.dumps({
    "description_repository": "https://github.com/husarion/rosbot_xl_ros",
    "description_commit": (vendor / "SOURCE_COMMIT").read_text(),
    "controller_repository": "https://github.com/husarion/husarion_controllers",
    "controller_commit": (ws / "vendor/husarion_controllers/SOURCE_COMMIT").read_text(),
    "adaptation": "Gazebo Classic ros2_control backend, contact direction transform, Linorobot2 sensors and EKF",
}, indent=2))
