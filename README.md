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

MODBUS TCP 默认关闭，现场按真实设备开启并修改寄存器：

```yaml
modbus:
  ins:
    enabled: true
    host: 192.168.1.20
  mmwave:
    enabled: true
    host: 192.168.1.21
```

## 测试

```bash
source /opt/ros/noetic/setup.bash
cd catkin_ws
catkin_make run_tests_target_localizer -DCMAKE_BUILD_TYPE=Release
```

完整本地编译还需要 TM16 驱动依赖，例如 `libpcap-dev`；Docker 镜像已内置这些依赖。
