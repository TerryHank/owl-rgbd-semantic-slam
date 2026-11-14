# OWL RGB-D 语义 SLAM

已部署到 WSL 发行版 `Ubuntu-22.04`。

- 仿真工作空间：`~/linorobot2_ws`
- 仓库：https://github.com/linorobot/linorobot2/tree/humble
- 提交：`6b7ae2ebd5e3782560363dca136c3a92864f5a46`
- 当前底盘采用 Husarion 原工程 ROSbot XL 的车身、碰撞体、惯量、四个 continuous 轮关节、轮速接口与原厂麦克纳姆控制器。统一使用 Gazebo Classic 11.10.2。适配脚本为本目录 `build_rosbot_classic.py`，生成文件位于工作空间 `display/rosbot_xl`。
- ROSbot 来源 https://github.com/husarion/rosbot_xl_ros/tree/ea9894d286606dd96420eb1aa36ba6c681ba8d12 。麦克纳姆控制器采用其依赖清单指定的 https://github.com/husarion/husarion_controllers/tree/217b09830f5f42930098b9992eda41710702b625 。源码及许可证保存在工作空间 `vendor`，构建到 `install-native`。
- 轮径 100 mm，轴距 170 mm，轮距 270 mm。车体与轮惯量沿用 ROSbot XL 参数，控制入口为 Linorobot2 的 `/cmd_vel`，输出 `/odom`、`/scan`、RGB 和深度图。
- 四轮通过 `gazebo_ros2_control` 的速度接口驱动，原厂控制器和 joint_state_broadcaster 均为 active。车辆由轮地接触产生运动。车轮使用原工程的圆柱接触体和各向异性摩擦参数 `mu=1, mu2=0`，滚子通过摩擦方向等效建模。原工程采用车体坐标定义摩擦方向，Classic 适配插件 `mecanum_friction.cpp` 每个物理步将该方向变换到车轮接触坐标。
- 当前默认场景是 ROBOTIS TurtleBot3 Humble 的原版 `turtlebot3_house.world`。来源提交为 `a35a56c8b04877dc89772b598084d8ce648a9023`，场景和模型位于 WSL 的 `vendor/turtlebot3_simulations`，外部家具缓存于 `assets/osrf_models`。现有 ROSbot XL 在室内 `(-2.0, -0.5)` 出生。

双击本目录的 `启动仿真.cmd` 或 `停止仿真.cmd`。仿真在 tmux 会话 `linorobot2-sim` 中运行。

## 原生语义建图

默认使用 F2M 视觉里程计、GTSAM 回环图优化与 5 cm 栅格。板端运行原生 RTAB-Map，输入只有 OWL RGB-D。YOLOE-26n-seg 在 RK3588/RK3588S NPU 上执行实例分割，深度和优化后的关键帧位姿把中文标签投到地图表面。构建与复现方式见 [原生程序文档](native/README.md)。

## U3D-OWL-01 与 ROS 仿真工具

先启动 Gazebo Classic，再双击 `启动视觉建图.cmd`。RTAB-Map 自带窗口显示图像、轨迹和地图。用键盘慢速驾驶积累关键帧，结束时双击 `停止视觉建图.cmd` 保存数据库。默认数据库为 `~/linorobot2_ws/maps/owl_rgbd_live_grid.db`，重新打开沿用该文件。可在 WSL 中用 `OWL_MAP_DB=/绝对路径/新地图.db bash ~/linorobot2_ws/start_rtabmap.sh` 开始新建图。

前相机采用 V1.5 规格书第 6 页表值，RGB 78°×48°，深度 72°×50°，两路 640×400、25 fps。深度数据以米发布为 32FC1，保留 0.3–8 m，越界像素使用 NaN。内参由视场和分辨率推算。两个渲染相机使用同光心近似，在深度像素网格重采样 RGB。规格书给出的 40 mm 结构光基线没有作为 RGB/深度外参。实机需要 SDK 数据、各流标定及配准。

RTAB-Map 输入为 `/owl/rgbd/image`、`/owl/rgbd/depth`、`/owl/rgbd/camera_info` 及自身产生的 `/owl/visual_odom`。其视觉 TF 在 `/owl/tf`。建图链路只用 RGB-D，底盘 EKF 继续独立运行。

RTAB-Map 的 `/rtabmap/map` 使用 5 cm 二维栅格、4.5 cm 地面高度阈值、50 cm 障碍高度上限及射线清空。Foxglove 左侧订阅 `/owl/map_display/grid`，显示最新实时栅格。右侧订阅 `/owl/map_display/cloud`，从同一套 RGB-D 关键帧和优化位姿拼接三维彩色点云。两张图会随关键帧和回环更新，面板布局由 Foxglove 保存。

在 WSL 终端使用键盘控制：

```bash
source ~/linorobot2_ws/env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` 前进，`,` 后退，`j/l` 左右转，`k` 停止；按住 Shift 再按 `J/L` 左右横移，`U/O/M/>` 斜移。

查看输出：

```bash
tmux attach -t linorobot2-sim
```

按 `Ctrl+B` 再按 `D` 可退出查看并保留运行。按 `Ctrl+C` 停止仿真。

环境由工作空间 `env.sh` 设置：Cyclone DDS、ROS_DOMAIN_ID=65、ROS_LOCALHOST_ONLY=1、Gazebo 端口 11365。通信配置位于工作空间 `cyclonedds.xml`。

## 安装处理

上游清单无法解析的依赖名 `python3-pycollada`、`python3-opencv-contrib-python`、`python-trimesh-pip` 已分别用系统 collada、带 aruco 的系统 OpenCV、项目虚拟环境 trimesh 3.23.5 满足。仿真不需要 micro-ROS Agent，按仓库说明跳过其依赖检查。

项目 `.venv` 使用系统包，固定 trimesh 3.23.5，解决已有用户级 trimesh 5.1.1 与系统 NumPy 的不兼容。

源码仅调整 `linorobot2_gazebo/scripts/command_timeout.py` 的可执行权限，否则 symlink-install 后 launch 找不到可执行文件。未修改其逻辑。

重新编译时：

```bash
cd ~/linorobot2_ws
source /opt/ros/humble/setup.bash
source .venv/bin/activate
colcon build --base-paths src --symlink-install --cmake-force-configure
```

原厂描述包与控制器的构建命令如下。模型生成和插件编译在 WSL 中执行。

```bash
cd ~/linorobot2_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths vendor/rosbot_xl_ros/rosbot_xl_description vendor/rosbot_xl_ros/rosbot_xl_controller vendor/husarion_controllers/mecanum_drive_controller --build-base build-native --install-base install-native --allow-overriding mecanum_drive_controller --cmake-args -DBUILD_TESTING=OFF
source env.sh
g++ -shared -fPIC display/rosbot_xl/mecanum_friction.cpp -o display/rosbot_xl/libmecanum_friction.so $(pkg-config --cflags --libs gazebo) -std=c++17
python3 display/rosbot_xl/build_rosbot_classic.py
```

上游地图转换脚本在 symlink-install 重编译时可能覆盖源码目录的同名 world。启动脚本使用从 Git 提交单独提取的原版场景副本。

## 已验证（2026-11-14）

- 6 个 ROS 包编译成功；仅有上游 trimesh 弃用提示。
- Gazebo GUI 显示场景，`linorobot2` 生成成功，实时因子约 1.0。
- `/clock`、`/scan`、`/odom`、RGB 和深度图均收到数据；激光和里程计约 10 Hz。
- 原厂四轮驱动测试在 Gazebo 世界坐标中测量实际位移。0.12 m/s 持续约两秒，前进 0.2393 m、横移 0.2391 m。
- 0.3 rad/s 持续约两秒旋转 0.5822 rad。斜移每轴速度 0.08 m/s，实际位移为 0.1600 m、0.1597 m。发送零命令后速度归零。
- 横移时四轮反馈约为 -2.48、+2.38、+2.44、-2.41 rad/s，轮关节位置随运动变化。
- 检验脚本为本目录 `check_mecanum.py`，完整结果位于工作空间 `display/rosbot_xl/classic-verification.json`。Nav2 自主导航尚未验证。

上游模型启动时有 IMU noise index 和 GPU 激光最大 180° 视场提示，相关数据仍能发布；未在本次部署中修改模型传感器参数。

## 低延迟地图与视觉导航

2026-11-15 更新。点云改为事件驱动更新，关键帧缓存后立即重拼。坐标变换使用单精度数组，体素去重使用整数键。RGB-D 配准后采用无损 PNG 打包传输，视觉里程计实测约 25 Hz。导航测试中四次新图更新的栅格与点云到达差为 0.049、0.109、0.041、0.094 秒。首次读取历史关键帧仍有缓存初始化时间。

启动仿真和视觉建图后，双击本目录的“启动视觉导航.cmd”。Nav2 使用 /owl/visual_odom 和 /owl/tf 定位，/rtabmap/map 做全局规划，/owl/nav/points 提供当前 OWL 深度障碍观测。当前速度上限为前进 0.20 m/s、横移 0.15 m/s、旋转 0.35 rad/s。这些设置位于 owl_nav.yaml。

Foxglove 原生 3D 面板选择“发布位姿”，目标坐标系用 owl_map，发布话题用 /move_base_simple/goal 或 /goal_pose，点击并拖动设置目标朝向。owl_nav_goal.py 将目标交给 Nav2。双击“取消导航.cmd”取消通过该入口发送的导航目标。“停止视觉导航.cmd”退出导航服务。自动导航和键盘驾驶应分开使用，切换手动前取消导航。

短程导航测试发送前方 0.65 m 目标，Nav2 返回成功，视觉地图中的终点误差 0.111 m，小于配置的 0.12 m 容差。仅验收短距离目标，长路线和复杂绕障待测。重启 ROS 通信后视觉里程计创建了新的局部会话，历史地图仍在 owl_rgbd_live_grid.db 中，回到历史区域并重定位后才能合并会话。布局只在 Foxglove 内保存。
