# 矿用三雷达标靶定位系统

本仓库是一个 ROS1 工程，用于 TM16 三雷达采集、点云融合、圆柱标靶 XY 位移测量、设备姿态/四角距离估计、Web 点云驾驶舱和 Docker 部署。总体方案依据 `SLAM.md`：三路雷达先融合为 `/points_raw`，再由定位节点输出标靶位移和设备区域状态。

## 目录结构

- `catkin_ws/src/timoo*`：TM16 雷达驱动、点云转换和驱动消息定义。
- `catkin_ws/src/lidar_fusion`：同步 2 路或 3 路雷达点云，通过 TF 统一到 `base_link`，发布 `/points_raw`。
- `catkin_ws/src/target_localizer`：订阅 `/points_raw`，完成 ROI 裁剪、圆柱拟合、跟踪滤波、设备姿态/四角距离估计和 MODBUS TCP 真实传感器接入。
- `catkin_ws/src/mine_slam_web`：ROS 点云到 WebSocket 的桥接，以及浏览器自适应点云驾驶舱。
- `docker/runtime`：现场外挂配置目录，容器启动时挂载到 `/config`，可直接修改 launch 和 YAML，不需要重建镜像。

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```

现场部署优先使用 Docker：

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up -d
```

arm64 主机打包：

```bash
./docker/build_arm64.sh
```

## 运行

默认 Docker 启动会同时拉起 TM16 驱动、三雷达融合、圆柱标靶定位、设备状态估计、MODBUS TCP 参考数据节点、WebSocket 桥和静态 Web 服务。

```bash
docker compose -f docker/docker-compose.yml up -d
docker logs -f --tail 200 mine-lidar-runtime
```

关键话题：

- `/lidar1/timoo_points`, `/lidar2/timoo_points`, `/lidar3/timoo_points`
- `/points_raw`
- `/target_measurement`, `/target_xy`, `/target_marker`, `/target_cloud_roi`
- `/equipment_state`：点云计算出的俯仰角、横滚角、偏航角，以及左前、左后、右前、右后距离。
- `/sensor_reference`：MODBUS TCP 接入的惯导和 4 路毫米波真实参考值。
- `/diagnostics`

点云处理链路：

```text
三路 TM16 点云
  -> multi_lidar_fusion_node
  -> /points_raw
  -> target_localizer_node + equipment_state_node
  -> /target_xy + /equipment_state
```

Web 页面采用左右双路数据显示：左侧显示点云计算值，右侧显示 MODBUS TCP 真实值。视频区域已移除，中间区域保留融合点云显示。

## 配置

三雷达融合参数：

```text
docker/runtime/lidar_fusion/multi_lidar_fusion.yaml
```

三雷达 TF 外参关系不在融合 YAML 中，而是在 Docker 挂载 launch 中修改：

```text
docker/runtime/launch/lidar_static_tf.launch
```

Docker 启动时该文件会挂载为 `/config/launch/lidar_static_tf.launch` 并由 `bringup.launch` 自动加载。现场修改雷达安装外参时，只需要改这个文件中的 3 行 `static_transform_publisher`：

```xml
args="x y z yaw pitch roll base_link lidar1"
args="x y z yaw pitch roll lidar1 lidar2"
args="x y z yaw pitch roll lidar1 lidar3"
```

其中 `x/y/z` 单位为米，`yaw/pitch/roll` 单位为弧度，顺序必须保持为 `x y z yaw pitch roll parent_frame child_frame`。修改后重启容器使 TF 生效。

圆柱标靶定位、设备姿态、四角距离和 MODBUS TCP 参数：

```text
docker/runtime/target_localizer/target_localizer.yaml
```

常调参数包括 ROI 范围、圆柱半径范围、最小内点数、残差阈值、参考高度、建零点、前/后测量点距离、侧向 ROI、MODBUS TCP IP/端口/站号、寄存器地址和缩放系数。

前后测量点配置示例：

```yaml
front_sample_distance_m: 2.0
rear_sample_distance_m: 2.0
sample_window_x_m: 0.4
forward_sign: 1
left_sign: 1
```

## TCP 惯导与毫米波雷达配置

惯导和毫米波雷达真实值由 `modbus_sensor_reference_node` 通过 MODBUS TCP 读取，并统一发布到 `/sensor_reference`。现场惯导和毫米波雷达使用同一个 TCP 设备 IP；Docker 部署后可直接修改挂载到容器内的外挂 YAML，无需重建镜像。

```text
docker/runtime/target_localizer/target_localizer.yaml
```

源码本地运行时对应文件为：

```text
catkin_ws/src/target_localizer/config/target_localizer.yaml
```

`modbus.host` 和 `modbus.port` 是现场 TCP 设备的共用 IP 和端口，惯导和 4 路毫米波雷达都使用这两项。部署时只需要改这一处 IP/端口，再把需要接入的 `enabled` 改为 `true`：

```yaml
modbus:
  host: 192.168.1.20     # 现场 TCP 设备 IP，只需要改这一处
  port: 502              # 现场 TCP 端口，惯导和毫米波共用
  ins:
    enabled: true
    unit_id: 1           # 从站 ID
    timeout_ms: 500
    roll: {address: 0, scale: 0.01}
    pitch: {address: 1, scale: 0.01}
    yaw: {address: 2, scale: 0.01}
  mmwave:
    enabled: true
    unit_id: 1
    timeout_ms: 500
    left_front: {address: 10, scale: 1.0}
    left_rear: {address: 11, scale: 1.0}
    right_front: {address: 12, scale: 1.0}
    right_rear: {address: 13, scale: 1.0}
```

字段说明：

- `host`：TCP 设备 IP 地址。当前现场方案中惯导和毫米波共用 `modbus.host`，因此现场换 IP 时只修改这一项。
- `port`：MODBUS TCP 端口，常用 `502`。当前现场方案中惯导和毫米波共用 `modbus.port`，因此现场换端口时只修改这一项。
- `unit_id`：MODBUS 从站 ID；多设备挂在同一网关时需要分别配置。
- `address`：寄存器地址，必须按现场设备协议表填写。
- `scale`：寄存器原始值到显示值的换算系数。例如惯导角度原始值为 1234、`scale: 0.01` 时，显示为 12.34 度；毫米波距离通常按毫米输出，可使用 `scale: 1.0`。

Docker 运行时会挂载 `docker/runtime/target_localizer/target_localizer.yaml` 到容器配置目录。部署后可以在宿主机直接编辑这个文件，修改 TCP IP、端口、从站 ID、寄存器地址和缩放系数；保存后重启运行容器使配置生效：

```bash
docker compose -f docker/docker-compose.yml restart mine-lidar-runtime
```

如果传感器没有接入，可保持 `enabled: false`，网页右侧真实惯导和毫米波数据会显示为空值。

## 测试

```bash
source /opt/ros/noetic/setup.bash
cd catkin_ws
catkin_make run_tests_target_localizer -DCMAKE_BUILD_TYPE=Release
```

完整本地编译还需要 TM16 驱动依赖，例如 `libpcap-dev`；Docker 镜像已内置这些依赖。
