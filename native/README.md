# OWL RGB-D 原生 RTAB-Map

`owl_slam` 是无界面 C++ 程序，直接调用 RTAB-Map 的 `Odometry` 和 `Rtabmap` 库。输入只有配准 RGB-D、相机内参、安装外参和采集时间戳。输出视觉位姿、回环记录和 SQLite 地图。Gazebo 录制适配器使用 ROS 2，核心和回放不需要 ROS 节点、DDS 或 ROS 环境。

## 当前 WSL 复现

```bash
src="$(pwd)/native"
work="$HOME/linorobot2_ws/owl_native"
bash "$src/run-local.sh" "$work/owl_loop_25fps.owl" "$work/another_map.db"
```

原始记录约 2.3 GiB，留在 WSL 的 `owl_native` 目录。以上命令按处理速度运行，可用来比较板卡峰值吞吐。要按原始采集时间回放，请使用：

```bash
set -o pipefail
python3 "$src/pace-stream.py" "$work/owl_loop_25fps.owl" |
  env -i HOME="$HOME" PATH=/usr/bin:/bin OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
    LD_LIBRARY_PATH=/opt/ros/humble/lib/x86_64-linux-gnu \
    "$work/build/owl_slam" "$src/owlslam.params" "$work/another_paced_map.db"
```

现有 WSL 二进制链接已安装的 RTAB-Map 0.23.13，库所在的 `/opt/ros/humble` 是安装前缀。应用运行时不加载 ROS。也可用 `replay-db.sh` 调原生 `rtabmap-reprocess` 对数据库重算，入口使用 `-odom`，不使用存档中的里程计猜测和已有回环边。生成彩色点云用 `export-map.sh` 从优化位姿与 RGB-D 重新拼接，栅格仍按二维生成。

## Gazebo 实时建图

在现有 Gazebo Classic 仿真运行时，从 WSL 执行以下命令。此入口使用独立数据库，每次启动创建一个新地图。

```bash
work="$HOME/linorobot2_ws/owl_native"
python3 -m pip install --target "$work/foxglove-deps" 'websockets==13.1'
bash native/start-live.sh /path/to/foxglove/jsonschema
```

在 Foxglove 连接 `ws://127.0.0.1:8768/`，在 3D 面板启用 `/owl/native/grid` 和 `/owl/native/cloud`。两者使用同一优化图和时间戳。回环触发后，栅格重新组装，点云按新的优化位姿投影。查看 `/owl/native/status` 或当前运行目录的 `foxglove.log` 可读到地图版本、回环目标和历史节点位姿变化。运行目录记录在 `$HOME/linorobot2_ws/owl_native/current-live`。

## ARM64 板端构建与相机接入

在目标板或同 ABI 的工具链上安装 OpenCV、PCL、SQLite 和 GTSAM，并独立构建 [RTAB-Map 0.23.13](https://github.com/introlab/rtabmap/tree/0.23.13-humble)。无界面构建可配置 `-DWITH_QT=OFF -DBUILD_APP=OFF`，保留 GTSAM 后端。编译完成后用 `rtabmap-reprocess --version` 核对 `With GTSAM: true`。若保留导出工具，确认前缀里有 `rtabmap-export`。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/path/to/rtabmap-prefix
cmake --build build -j2
./build/owl_slam owlslam.params new_map.db < recording.owl
```

`owlslam.params` 默认使用帧到局部地图 RGB-D 特征匹配、平面运动约束、1 Hz 后端和 5 cm 栅格。`[odometry]` 部分只覆盖前端，建图后端仍使用 GTSAM。视觉里程计逐帧运行，建图、回环和成对地图输出在后台线程运行。后端队列最多保留四个待处理关键帧，结束时处理完队列再关闭数据库。

Orange Pi 5 Pro 的回放测试使用 `OWL_ODOM_PARAMS` 指向 `owlslam-orangepi-odom.params`，将视觉里程计局部束调整迭代上限设为 3。建图和回环继续使用原始参数文件。板端使用的 YOLO26n-seg FP16 模型由 x86 Linux 上的 RKNN-Toolkit2 转换，板上 `yolo26n-seg-rknn.cpp` 使用 RKNPU C 接口推理和 CPU 掩膜后处理。双任务测试的 YOLO 在 OWL 录像中各取最新到期的帧，独立于 SLAM 的逐帧读取。

原生 SLAM 通过 `OWL_SEMANTIC_GRAPH=1` 输出每版优化图的关键帧位姿，通过 `OWL_RAW_POSES` 输出带时间戳的原始视觉位姿。YOLOE-26n-seg 以 `yoloe-indoor-classes.json` 的文本类别导出为 RKNN FP16。由于模型输出需要配合 OWL 深度，运行分割程序时传入 `detections.jsonl` 输出参数。

在 x86 Linux 中用 Ultralytics、CLIP、RKNN-Toolkit2 2.3.2 导出模型。RKNN 转换阶段使用 ONNX 1.16.1。

```bash
python3 native/export-yoloe-rknn.py yoloe-26n-seg.pt yoloe-indoor.rknn \
  --rknn-python /path/to/rknn-venv/bin/python
```

在板端准备 RTAB-Map、Rockchip 的 `rknn_api.h` 与 `librknnrt.so`，以及 NumPy、SciPy。启用 `-DBUILD_RKNN_SEGMENTATION=ON` 后会生成 `yoloe_rknn`。按原速并跑时使用三个终端，工作目录和输入路径一致。

```bash
OWL_SEMANTIC_GRAPH=1 OWL_RAW_POSES=odom.csv \
  ./build/owl_slam native/owlslam.params map.db map.bin < <(python3 native/pace-stream.py recording.owl)
./build/yoloe_rknn yoloe-indoor.rknn recording.owl 180 0 detections.jsonl
python3 native/semantic-objects.py recording.owl detections.jsonl odom.csv \
  native/yoloe-indoor-classes.json map.bin objects.json --watch
```

`semantic-objects.py RECORDING.owl detections.jsonl odom.csv yoloe-indoor-classes.json map.bin objects.json --watch` 按当前优化图重算物体位置，合并距离和掩膜几何吻合的重复观测，并将标签落在邻近点云表面。`yoloe-indoor-labels-zh.json` 为显示标签提供中文名称，结果中的 `class_name` 保留模型类别名。Foxglove 桥使用 `--semantics objects.json` 发布 `/owl/native/semantic_markers`，只显示与当前地图版本一致的标签。

实机需要歌尔 ARM64 SDK 取流。先用 SDK 输出已配准的 RGB-D 和对应实测内参，再用下列接口喂给核心。也可在厂商 SDK 的 C++ 回调直接调用同样的 RTAB-Map API，减少进程间复制。

- 流头只发一次，格式为 `OWL1 width height fx fy cx cy x y z roll pitch yaw\n`。外参为相机光学坐标到机器人基础坐标的变换，长度用米，角度用弧度。
- 每帧先发采集时间秒数字符串和换行，再发连续的 `W×H×3` 字节 BGR8 和 `W×H×4` 字节小端 Float32 深度。深度为光轴方向的米制 Z 值，超出 0.3–8 m 的像素视为无效。
- `capture-gazebo.py` 只用于仿真测试。在已经加载仿真 ROS 环境的终端中运行并驾驶一圈。它导出 OWL 流，后续回放独立运行。

当前验证覆盖 x86 WSL 和 Orange Pi 5 Pro 的 OWL 仿真录像回放。实体 OWL 的 Linux ARM64 SDK、实机标定、长时间运行的帧率和温升仍待验证。仿真参数中的相机外参由录制时读取，板端使用实际安装标定。

## F2M 可选陀螺仪航向

`OWL_GYRO` 可指向同一次录制的 `IMU1` 文件。程序将角速度旋转到车体坐标并积分相邻 RGB-D 帧之间的航向增量。F2M 用视觉速度预测平移，用这个航向增量作为特征配准的初值，最终位姿由 RGB-D 配准求解。加速度不参与计算。未设置 `OWL_GYRO` 时，F2M 使用默认的纯 RGB-D 路径。该选项面向录制数据回放。

在仿真中，可用 `OWL_GAZEBO_IMU_HZ=200` 重新生成机器人模型并重启 Gazebo，再通过 `capture-gazebo.py paired.owl --imu-csv imu.txt --imu-frame imu_link` 同时录制彩色、深度和 IMU。IMU 文件的首行为 `IMU1 x y z roll pitch yaw`，随后的每行是采集时间、三轴角速度和三轴加速度，使用 SI 单位。

```bash
OWL_GYRO=imu.txt ./build/owl_slam native/owlslam.params gyro.db gyro-map.bin < paired.owl
./build/owl_slam native/owlslam.params rgbd.db rgbd-map.bin < paired.owl
```

### House 长路线 A/B 对照

同一段 640×400、25 FPS、1,707 帧的 Gazebo Classic 录像。两组使用相同的 F2M、回环和建图参数。香橙派 5 Pro 上同时运行 RKNN YOLOE-26n-seg 和中文语义融合。下表的墙面散布是两段固定墙面抽样的法向坐标 P90−P10；位置与航向误差相对 Gazebo 真值计算，真值只参与评价。

| 指标 | 纯 RGB-D F2M | 加陀螺仪航向初值 |
|---|---:|---:|
| 跟踪帧数 | 1,707 / 1,707 | 1,707 / 1,707 |
| 原速接受的回环约束 | 6 | 6 |
| 轨迹位置 RMSE | 0.98 cm | 1.49 cm |
| 航向 RMSE | 0.33° | 0.51° |
| 两段墙面回环后的散布 | 5.76 / 0.95 cm | 6.00 / 0.77 cm |
| 板端原速最大积压 | 22.1 ms | 35.5 ms |
| 板端原速逐帧处理 P95 | 39.3 ms | 38.7 ms |
| 板端不限速完整 SLAM 吞吐，YOLOE 并跑 | 49.4 FPS | 48.9 FPS |

陀螺仪组首次回环前后保留了 19 个中文物体 ID，标签重新贴合校正后的点云，最大移动 5.4 cm。两组吞吐都高于 25 FPS 留 20% 余量对应的 31.25 FPS。此次陀螺仪初值未带来整体地图质量收益，因此保留纯 RGB-D F2M 为默认。

## 已验证的基线

Gazebo 同一圈录制 1363 帧，640×400，采集跨度 54.481 秒，平均 25.00 fps。两次从空数据库运行均跟踪 1363/1363 帧、生成 45 个优化图节点、接受 1 次视觉回环（1244 → 1）。数据库通过 SQLite quick_check。

| 本机 x86 测试 | 原速回放 | 不限速回放 |
|---|---:|---:|
| 视觉里程计 P95 | 16.41 ms | 14.56 ms |
| 建图单次 P95 | 58.20 ms | 64.92 ms |
| 核心进程峰值 RSS | 228 MiB | 223 MiB |

原速回放终点到起点的地图位姿距离约 2.35 cm，Gazebo 实际返回距离约 2.38 cm。其余路径误差尚未按逐帧真值计算。不限速回放导出的 5 cm 栅格和 161115 点彩色点云见 `results/`。RSS 是短程测试中单个 SLAM 进程的峰值，取流适配器、传感器 SDK 和长程地图增长要另外计入板端预算。
