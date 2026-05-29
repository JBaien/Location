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
equipment_half_width_m: 1.2
min_valid_clearance_m: -0.2
max_ground_plane_rmse: 0.08
min_ground_normal_z: 0.85
max_wall_direction_diff_deg: 8.0
max_pointcloud_age_sec: 0.5
distance_filter_alpha: 0.3
max_distance_jump_m: 0.3
min_total_points: 100
```

这些参数含义如下：

- `front_sample_distance_m`：前测量点距离设备中心的前向距离，单位米，用于计算左前、右前距离。
- `rear_sample_distance_m`：后测量点距离设备中心的后向距离，单位米，用于计算左后、右后距离。
- `sample_window_x_m`：前/后测量点沿 X 方向的采样窗口宽度，单位米。例如前测量点在 2.0 m、窗口为 0.4 m 时，会取约 1.8 m 到 2.2 m 范围内的点。
- `forward_sign`：定义设备“前方”对应的 X 轴方向。`1` 表示 +X 为前方，`-1` 表示 -X 为前方。
- `left_sign`：定义设备“左侧”对应的 Y 轴方向。`1` 表示 +Y 为左侧，`-1` 表示 -Y 为左侧。
- `equipment_half_width_m`：设备半宽，单位米。四角距离会从雷达坐标到侧壁距离中扣除该值，显示为设备外轮廓到侧壁的剩余距离。
- `min_valid_clearance_m`：允许的最小有效剩余距离，单位米。轻微负值可保留用于提示接近或越界；明显低于该阈值会标记为无效。
- `max_wall_direction_diff_deg`：左右墙 PCA 主方向最大允许夹角，超过后偏航角无效。
- `max_pointcloud_age_sec`：输入点云最大允许延迟，超过后整体状态无效。
- `distance_filter_alpha`：距离一阶滤波系数，越小越平滑，越大越跟手。
- `max_distance_jump_m`：距离单帧最大允许跳变，超过后该方向距离无效。
- `min_total_points`：整帧融合点云最小点数，低于该值时整体状态无效。

## 设备姿态与四角距离计算

`equipment_state_node` 订阅融合后的 `/points_raw`。该点云必须已经通过三雷达 TF 外参统一到 `base_link` 设备坐标系，默认约定为 X 向前、Y 向左、Z 向上。节点从当前点云中估算设备相对巷道/底板的姿态趋势和左右前后剩余距离，并发布 `/equipment_state`。它不是严格 IMU 姿态解算，真实控制或安全联锁应优先使用惯导，并把点云结果作为环境参考或校验。

节点会检查输入点云的 `header.stamp`、`frame_id` 和点数。如果点云为空、超时，或 `frame_id` 不符合 `required_frame_id`，则 `/equipment_state` 整体标记为无效。当前部署中融合节点应输出 `frame_id=base_link`；如果后续改为其他坐标系，应先通过 TF 转换到设备坐标系再计算。

姿态角计算：

- 俯仰角和横滚角：在设备附近地面 ROI 内筛选底板点，先用 RANSAC 平面剔除浮煤、车体结构、线缆等离群点，再用内点拟合平面 `z = ax + by + c`。平面法向量按 `n = normalize([-a, -b, 1])` 理解，`a` 表示前后坡度，用于计算俯仰角；`b` 表示左右坡度，用于计算横滚角。最终正负号必须结合现场“前方抬高、左侧抬高”的实测结果校准。
- 地面平面需要满足 `min_ground_points`、`max_ground_plane_rmse` 和 `min_ground_normal_z`，否则俯仰/横滚会标记为无效。
- 偏航角：在侧壁 ROI 内筛选左右边界点，左右墙分别投影到 XY 平面后用 PCA 估计巷道主方向向量 `dir = [dx, dy]`，方向统一到 `dx >= 0` 后再平均，并用 `atan2(dy, dx)` 计算设备相对巷道方向的偏航角，避免 `y = kx + b` 斜率形式在异常角度下不稳定，也避免 PCA 的 180 度方向歧义造成跳变。
- 左右墙主方向夹角必须小于 `max_wall_direction_diff_deg`，否则认为侧壁方向不一致，偏航角无效。

四角距离计算：

- 根据 `front_sample_distance_m` 和 `rear_sample_distance_m` 确定前、后两个 X 向采样位置。
- 根据 `left_sign` 区分左侧和右侧，在侧向 ROI 内分别取左前、左后、右前、右后点云。
- 每个采样区会按 `sample_window_x_m` 限制前后宽度，并按 `side_y_abs_min/side_y_abs_max`、`side_z_min/side_z_max` 过滤有效侧壁点。
- 距离值取侧向距离的低分位数，默认 `distance_percentile: 0.1`，这样比直接取最小值更抗孤立噪点。
- 输出距离会扣除 `equipment_half_width_m`，因此网页显示的是设备外轮廓到侧壁的剩余距离，不是雷达坐标原点到侧壁的距离。
- 距离允许在 `min_valid_clearance_m` 范围内保留轻微负值，用于提示外轮廓已经接近或越界；明显异常负值会标记为无效。
- 距离值会经过轻量时间滤波，并使用 `max_distance_jump_m` 抑制孤立帧突变；跳变过大时该方向距离无效。
- 每个方向有效点数至少需要达到 `min_distance_points`，否则该方向距离无效，网页显示为空值。
- `/equipment_state` 的整体状态不简单等于所有单项均有效。`quality` 使用 `OK`、`DEGRADED`、`INVALID`、`LOST`：输入正常且主要姿态/距离都有效为 `OK`；部分有效为 `DEGRADED`；点云存在但关键拟合失败为 `INVALID`；点云为空、超时或坐标系错误为 `LOST`。
- `/equipment_state` 同时发布整体有效性和单项有效性，包括 `roll_valid`、`pitch_valid`、`yaw_valid`、`left_front_valid`、`left_rear_valid`、`right_front_valid`、`right_rear_valid`，并带有参与计算的点数、`quality` 状态和 `invalid_reason`。`invalid_reason` 只描述本节点输入、ROI、拟合和质量门控失败原因，不包含前端 WebSocket 或外部 TCP 通信错误。常见无效原因包括 `NO_POINTCLOUD`、`POINTCLOUD_STALE`、`FRAME_ID_INVALID`、`LOW_TOTAL_POINTS`、`LOW_GROUND_POINTS`、`HIGH_GROUND_RMSE`、`BAD_GROUND_NORMAL`、`LOW_WALL_POINTS`、`RANSAC_FAILED`、`PCA_FAILED`、`PCA_INCONSISTENT`、`LOW_DISTANCE_POINTS`、`CLEARANCE_INVALID`、`DISTANCE_JUMP_REJECTED`。
- 距离在 ROS 消息中优先使用米制字段，例如 `left_front_clearance_m`、`right_front_clearance_m`、`left_rear_clearance_m`、`right_rear_clearance_m`；现有 `*_mm` 字段仅用于兼容当前网页显示。
- 左侧距离使用 `y > 0` 点的 `y` 作为侧向距离；右侧距离使用 `y < 0` 点的 `-y`，等价于 `abs(y)`。最终统一计算 `clearance_m = wall_distance_m - equipment_half_width_m`。

## TCP 惯导与毫米波雷达配置

惯导和毫米波雷达真实值由 `modbus_sensor_reference_node` 通过 MODBUS TCP 读取，并统一发布到 `/sensor_reference`。现场惯导和毫米波雷达使用同一个 TCP 设备 IP；Docker 部署后可直接修改挂载到容器内的外挂 YAML，无需重建镜像。

```text
docker/runtime/target_localizer/target_localizer.yaml
```

源码本地运行时对应文件为：

```text
catkin_ws/src/target_localizer/config/target_localizer.yaml
```

`modbus.host`、`modbus.port`、`modbus.unit_id` 和 `modbus.timeout_ms` 是现场 TCP 设备的共用连接参数，惯导和 4 路毫米波雷达都使用这一组参数。节点内部只维护一个 TCP 连接，并顺序读取惯导和毫米波寄存器，不会为惯导和毫米波分别建立两个连接。部署时只需要改这一处连接参数，再把需要接入的 `enabled` 改为 `true`：

```yaml
modbus:
  host: 192.168.1.20     # 现场 TCP 设备 IP，只需要改这一处
  port: 502              # 现场 TCP 端口，惯导和毫米波共用
  unit_id: 1             # MODBUS 从站 ID，惯导和毫米波共用
  timeout_ms: 500        # TCP 读写超时，惯导和毫米波共用
  ins:
    enabled: true
    roll: {address: 0, scale: 0.01}
    pitch: {address: 1, scale: 0.01}
    yaw: {address: 2, scale: 0.01}
  mmwave:
    enabled: true
    left_front: {address: 10, scale: 1.0}
    left_rear: {address: 11, scale: 1.0}
    right_front: {address: 12, scale: 1.0}
    right_rear: {address: 13, scale: 1.0}
```

字段说明：

- `host`：TCP 设备 IP 地址。当前现场方案中惯导和毫米波共用 `modbus.host`，因此现场换 IP 时只修改这一项。
- `port`：MODBUS TCP 端口，常用 `502`。当前现场方案中惯导和毫米波共用 `modbus.port`，因此现场换端口时只修改这一项。
- `unit_id`：MODBUS 从站 ID。当前现场方案中惯导和毫米波共用 `modbus.unit_id`，因此只配置一次。
- `timeout_ms`：TCP 读写超时时间，单位毫秒。当前现场方案中惯导和毫米波共用 `modbus.timeout_ms`，因此只配置一次。
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
