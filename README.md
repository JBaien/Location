# 矿用三雷达标靶定位系统

本仓库是一个 ROS1 工程，用于三雷达采集、点云融合、圆柱标靶 XY 位移测量、设备姿态/四角距离估计、Web 点云驾驶舱和 Docker 部署。总体方案依据 `SLAM.md`：三路雷达先融合为 `/points_raw`，再由定位节点输出标靶位移和设备区域状态。

## 目录结构

- `catkin_ws/src/timoo*`：Timoo TM16 雷达驱动、点云转换和驱动消息定义。
- `catkin_ws/src/lidar_*`：tmlidar 驱动、点云转换、LaserScan 转换和驱动消息定义。
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

## Docker 镜像部署包

本仓库支持打包 Ubuntu 18.04 / ROS Melodic 运行镜像。当前生成的现场部署总包包括镜像、compose 文件、`docker/runtime` 外挂配置和校验文件：

```text
mine-lidar-deploy-amd64.tar.gz
mine-lidar-deploy-arm64.tar.gz
```

amd64 主机部署：

```bash
tar -xzf mine-lidar-deploy-amd64.tar.gz
cd deploy-amd64
sha256sum -c SHA256SUMS.txt
gunzip -c mine-lidar-runtime-melodic-amd64.tar.gz | docker load
cd docker
docker compose -f docker-compose.yml up -d
```

arm64 主机部署：

```bash
tar -xzf mine-lidar-deploy-arm64.tar.gz
cd deploy-arm64
sha256sum -c SHA256SUMS.txt
gunzip -c mine-lidar-runtime-melodic-arm64.tar.gz | docker load
cd docker
docker compose -f docker-compose.arm64.yml up -d
```

部署后现场可直接修改 `deploy-*/docker/runtime` 下的 launch/YAML，例如雷达 TF、融合参数、设备状态参数、MODBUS TCP IP 和端口；修改后重启容器即可生效。

## 运行

默认 Docker 启动会同时拉起 Timoo TM16 驱动、三雷达融合、圆柱标靶定位、设备状态估计、MODBUS TCP 参考数据节点、WebSocket 桥和静态 Web 服务。需要改用 tmlidar 驱动时，在启动参数或 `docker/runtime/launch/bringup.launch` 中设置 `driver_family:=tmlidar`；算法侧仍订阅融合后的 `/points_raw`。

```bash
docker compose -f docker/docker-compose.yml up -d
docker logs -f --tail 200 mine-lidar-runtime
```

关键话题（默认 Timoo 驱动）：

- `/lidar1/timoo_points`, `/lidar2/timoo_points`, `/lidar3/timoo_points`
- `/points_raw`
- `/target_measurement`, `/target_xy`, `/target_marker`, `/target_cloud_roi`
- `/equipment_state`：点云计算出的俯仰角、横滚角、偏航角，以及左前、左后、右前、右后距离。
- `/sensor_reference`：MODBUS TCP 接入的惯导和 4 路毫米波真实参考值。
- `/diagnostics`

tmlidar 驱动模式下，三路原始点云话题改为 `/lidar1/lidar_points`、`/lidar2/lidar_points`、`/lidar3/lidar_points`，融合输出和算法输入仍是 `/points_raw`。

点云处理链路：

```text
三路原始雷达点云
  -> multi_lidar_fusion_node
  -> /points_raw
  -> target_localizer_node + equipment_state_node
  -> /target_xy + /equipment_state
```

Web 页面采用左右双路数据显示：左侧显示点云计算值，右侧显示 MODBUS TCP 真实值。视频区域已移除，中间区域保留融合点云显示。

## 配置

三雷达融合参数：

```text
docker/runtime/lidar_fusion/multi_lidar_fusion_timoo.yaml
docker/runtime/lidar_fusion/multi_lidar_fusion_tmlidar.yaml
```

`driver_family:=timoo` 使用 `/lidarN/timoo_points`；`driver_family:=tmlidar` 使用 `/lidarN/lidar_points`。也可以通过 `driver_launch` 和 `fusion_config` 指定自定义驱动 launch 与融合 YAML。

驱动切换只需要改 `docker/runtime/launch/bringup.launch`：

```xml
<arg name="driver_family" default="tmlidar"/>
```

如果现场 `/config` 已经存在，容器启动脚本不会覆盖已有配置；升级后需要把新增的 `driver_tmlidar_multi3.launch`、`multi_lidar_fusion_tmlidar.yaml` 和对应录包 launch 同步到现场挂载目录。

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

该文件被 `target_localizer_node`、`equipment_state_node` 和 `modbus_sensor_reference_node` 共同读取。Docker 部署时修改宿主机挂载目录中的这个文件，重启容器后生效；源码本地运行时对应文件为 `catkin_ws/src/target_localizer/config/target_localizer.yaml`。

基础输入输出参数：

| 参数 | 单位/类型 | 含义 |
| --- | --- | --- |
| `input_topic` | ROS topic | 输入融合点云，默认 `/points_raw`。圆柱定位和设备状态估计都订阅该点云。 |
| `target_frame` | frame_id | 业务计算目标坐标系，默认 `base_link`。定位输出和设备状态计算都按该坐标系理解。 |
| `measurement_topic` | ROS topic | 发布圆柱检测原始测量值，包含中心、内点数、残差等。 |
| `xy_topic` | ROS topic | 发布业务用标靶 XY 位移，默认 `/target_xy`。 |
| `status_topic` | ROS topic | 发布标靶定位状态，默认 `/target_status`。 |
| `marker_topic` | ROS topic | 发布 RViz 可视化 Marker。 |
| `roi_topic` | ROS topic | 发布裁剪后的目标 ROI 点云，便于现场检查标靶是否进入有效区域。 |

圆柱标靶检测参数：

| 参数 | 单位/类型 | 含义 |
| --- | --- | --- |
| `roi_x_min`、`roi_x_max` | m | 目标圆柱 ROI 的 X 范围。当前默认 `-12.0` 到 `-4.0`，表示只在设备后方一定距离内找标靶。 |
| `roi_y_min`、`roi_y_max` | m | 目标圆柱 ROI 的 Y 范围，用于排除左右巷壁、设备结构和非目标点。 |
| `roi_z_min`、`roi_z_max` | m | 目标圆柱 ROI 的 Z 范围，用于排除地面、顶板和高处干扰。 |
| `voxel_leaf` | m | ROI 点云体素降采样叶子尺寸。值越大点越少、速度越快，但细节损失更大。 |
| `reflector_intensity_min` | 强度值 | 反光点最小强度阈值。圆柱拟合只使用 ROI 内 `intensity` 大于该值的点，默认 `180.0`。 |
| `sor_mean_k` | 点数 | 统计离群滤波的邻域点数。粉尘和孤立噪点多时可适当增大。 |
| `sor_stddev` | 倍数 | 统计离群滤波标准差阈值。值越小剔除越严格。 |
| `normal_k` | 点数 | 法线估计邻域点数，影响圆柱模型的法线约束稳定性。 |
| `max_iterations` | 次 | 圆柱 RANSAC 最大迭代次数。值越大越稳但耗时更高。 |
| `normal_distance_weight` | 权重 | PCL 圆柱分割中法线距离权重。值越大越依赖法线一致性。 |
| `cylinder_distance_threshold` | m | 点到圆柱模型的最大内点距离。值越小拟合越严格，遮挡或噪声大时可能丢失。 |
| `radius_min`、`radius_max` | m | 允许的圆柱半径范围，应按标靶实际半径和点云误差设置。 |
| `min_candidate_points` | 点数 | ROI 经过降采样和滤波后参与圆柱拟合的最小点数，低于该值直接认为无有效候选。 |
| `reference_z` | m | 从拟合圆柱轴线取业务中心点时使用的参考高度。输出的 `cx/cy` 是圆柱轴线在该高度处的位置。 |
| `zero_x`、`zero_y` | m | 建零点坐标。`/target_xy` 中的 `dx/dy` 按检测中心相对该点计算。 |

目标跟踪与质量门限：

| 参数 | 单位/类型 | 含义 |
| --- | --- | --- |
| `min_good_inliers` | 点数 | 判定单帧圆柱测量为 `GOOD` 的最小内点数。低于该值但仍可输出时通常为 `DEGRADED`。 |
| `good_residual_rms` | m | 判定单帧圆柱测量为 `GOOD` 的最大残差均方根。 |
| `lost_after_misses` | 帧 | 连续多少帧没有有效圆柱测量后进入 `LOST`。 |
| `hold_duration` | s | 短时丢失时按上一状态和速度保活的时间窗口。 |

设备状态输入检查参数：

| 参数 | 单位/类型 | 含义 |
| --- | --- | --- |
| `equipment_state_topic` | ROS topic | 发布点云估计的设备姿态和四角距离，默认 `/equipment_state`。 |
| `required_frame_id` | frame_id | 设备状态计算要求的输入点云坐标系，当前部署为 `base_link`。 |
| `enable_tf_transform` | bool | 输入点云不是 `required_frame_id` 时是否尝试 TF 转换。默认关闭，frame 不匹配直接无效；开启后 TF 失败标记为 `TF_LOOKUP_FAILED`。 |
| `max_pointcloud_age_sec` | s | 输入点云最大允许延迟，超过后整体状态无效。 |
| `min_total_points` | 点数 | 整帧融合点云最小点数，低于该值时整体状态无效。 |

地面和侧壁 ROI 参数：

| 参数 | 单位/类型 | 含义 |
| --- | --- | --- |
| `ground_x_min`、`ground_x_max` | m | 地面点 ROI 的 X 范围，用于估计横滚和俯仰。 |
| `ground_y_min`、`ground_y_max` | m | 地面点 ROI 的 Y 范围。 |
| `ground_z_min`、`ground_z_max` | m | 地面点 ROI 的 Z 范围。 |
| `wall_x_min`、`wall_x_max` | m | 侧壁点 ROI 的 X 范围，用于估计偏航和辅助距离判断。 |
| `wall_y_abs_min`、`wall_y_abs_max` | m | 侧壁点 ROI 的 `abs(y)` 范围。用绝对值同时筛选左右墙，排除车体附近点和过远点。 |
| `wall_z_min`、`wall_z_max` | m | 侧壁点 ROI 的 Z 范围。 |
| `max_ground_plane_rmse` | m | 地面平面拟合最大允许 RMSE，超过后横滚/俯仰无效。 |
| `min_ground_normal_z` | 比值 | 地面法向量 Z 分量最小值。值越接近 1，要求地面越接近水平。 |
| `max_wall_direction_diff_deg` | deg | 左右墙 PCA 主方向最大允许夹角，超过后认为侧壁方向不一致，偏航角无效。 |
| `min_ground_points` | 点数 | 地面 ROI 最小有效点数。 |
| `min_wall_points` | 点数 | 侧壁 ROI 最小有效点数。 |

四角距离参数：

| 参数 | 单位/类型 | 含义 |
| --- | --- | --- |
| `front_sample_distance_m` | m | 前测量点距离设备中心的前向距离，用于计算左前、右前距离。 |
| `rear_sample_distance_m` | m | 后测量点距离设备中心的后向距离，用于计算左后、右后距离。 |
| `sample_window_x_m` | m | 前/后测量点沿 X 方向的采样窗口宽度。例如前测量点为 `2.0`、窗口为 `0.4` 时，取约 `1.8` 到 `2.2` m 范围内的点。 |
| `side_y_abs_min`、`side_y_abs_max` | m | 四角距离采样时侧向点的 `abs(y)` 范围。 |
| `side_z_min`、`side_z_max` | m | 四角距离采样时侧向点的 Z 范围。 |
| `distance_percentile` | 0 到 1 | 侧向距离取值分位数，默认 `0.1`。比直接取最小值更抗孤立噪点。 |
| `equipment_half_width_m` | m | 设备半宽。输出清距为 `侧壁距离 - equipment_half_width_m`，表示设备外轮廓到侧壁的剩余距离。 |
| `min_valid_clearance_m` | m | 允许的最小有效剩余距离。轻微负值可用于提示接近或越界，明显低于该阈值会标记无效。 |
| `min_distance_points` | 点数 | 每个方向四角距离的最小有效点数。 |
| `distance_filter_alpha` | 0 到 1 | 距离一阶滤波系数。越小越平滑，越大越跟手。 |
| `max_distance_jump_m` | m | 距离单帧最大允许跳变，超过后该方向距离无效。 |
| `forward_sign` | `1` 或 `-1` | 定义设备“前方”对应的 X 轴方向。`1` 表示 +X 为前方，`-1` 表示 -X 为前方。 |
| `left_sign` | `1` 或 `-1` | 旧配置兼容项。当前算法在 `base_link` 下固定使用 +Y 为左侧、-Y 为右侧，不再用该参数切换左右方向。 |

## 设备姿态与四角距离计算

`equipment_state_node` 订阅融合后的 `/points_raw`。该点云必须已经通过三雷达 TF 外参统一到 `base_link` 设备坐标系，默认约定为 X 向前、Y 向左、Z 向上。节点从当前点云中估算设备相对巷道/底板的姿态趋势和左右前后剩余距离，并发布 `/equipment_state`。它不是严格 IMU 姿态解算，真实控制或安全联锁应优先使用惯导，并把点云结果作为环境参考或校验。

节点会检查输入点云的 `header.stamp`、`frame_id` 和点数。如果点云为空、超时，或 `frame_id` 不符合 `required_frame_id`，则 `/equipment_state` 整体标记为无效。当前部署中融合节点应输出 `frame_id=base_link`；如果后续改为其他坐标系，可通过 `enable_tf_transform: true` 尝试 TF 转换到 `required_frame_id` 后再计算，TF 转换失败时使用 `TF_LOOKUP_FAILED`，未开启转换时使用 `FRAME_ID_INVALID`。

姿态角计算：

- 俯仰角和横滚角：在设备附近地面 ROI 内筛选底板点，先用 RANSAC 平面剔除浮煤、车体结构、线缆等离群点，再用内点拟合平面 `z = ax + by + c`。平面法向量按 `n = normalize([-a, -b, 1])` 理解，`a` 表示前后坡度，用于计算俯仰角；`b` 表示左右坡度，用于计算横滚角。最终正负号必须结合现场“前方抬高、左侧抬高”的实测结果校准。
- 地面平面需要满足 `min_ground_points`、`max_ground_plane_rmse` 和 `min_ground_normal_z`，否则俯仰/横滚会标记为无效。
- 偏航角：在侧壁 ROI 内筛选左右边界点，左右墙分别投影到 XY 平面后用 PCA 估计巷道主方向向量 `dir = [dx, dy]`，方向统一到 `dx >= 0` 后再平均，并用 `atan2(dy, dx)` 计算设备相对巷道方向的偏航角，避免 `y = kx + b` 斜率形式在异常角度下不稳定，也避免 PCA 的 180 度方向歧义造成跳变。
- 左右墙主方向夹角必须小于 `max_wall_direction_diff_deg`，否则认为侧壁方向不一致，偏航角无效。

四角距离计算：

- 根据 `front_sample_distance_m` 和 `rear_sample_distance_m` 确定前、后两个 X 向采样位置。
- 在 `base_link` 坐标系中固定按 Y 轴正负区分左右：左侧取 `y > 0` 点，右侧取 `y < 0` 点；`left_sign` 仅作为旧配置保留，不再作为新逻辑的主要左右判定依据。
- 每个采样区会按 `sample_window_x_m` 限制前后宽度，并按 `side_y_abs_min/side_y_abs_max`、`side_z_min/side_z_max` 过滤有效侧壁点。
- 距离值取侧向距离的低分位数，默认 `distance_percentile: 0.1`，这样比直接取最小值更抗孤立噪点。
- 输出距离会扣除 `equipment_half_width_m`，因此网页显示的是设备外轮廓到侧壁的剩余距离，不是雷达坐标原点到侧壁的距离。
- 距离允许在 `min_valid_clearance_m` 范围内保留轻微负值，用于提示外轮廓已经接近或越界；明显异常负值会标记为无效。
- 距离值会经过轻量时间滤波，并使用 `max_distance_jump_m` 抑制孤立帧突变；跳变过大时该方向距离无效。滤波器只使用通过 ROI、点数、清距范围和跳变检查的有效原始距离更新；无效距离不会更新滤波器，第一帧有效距离只初始化滤波器，不触发跳变拒绝。
- 每个方向有效点数至少需要达到 `min_distance_points`，否则该方向距离无效，网页显示为空值。
- `/equipment_state` 的整体状态不简单等于所有单项均有效。`overall_status` 使用 `OK`、`DEGRADED`、`INVALID`、`LOST`：输入正常且主要姿态/距离都有效为 `OK`；部分有效为 `DEGRADED`；点云存在但关键拟合失败为 `INVALID`；点云为空、超时或坐标系错误为 `LOST`。旧字段 `quality` 保留为 `overall_status` 的兼容别名。
- `/equipment_state` 同时发布整体有效性和单项有效性，包括 `roll_valid`、`pitch_valid`、`yaw_valid`、`left_front_valid`、`left_rear_valid`、`right_front_valid`、`right_rear_valid`。各单项用独立的 `*_quality` 和 `*_invalid_reason` 描述局部计算质量，整体 `invalid_reason` 只描述本节点输入、ROI、拟合和质量门控失败原因，不包含前端 WebSocket 或外部 TCP 通信错误。常见无效原因包括 `NO_POINTCLOUD`、`POINTCLOUD_STALE`、`FRAME_ID_INVALID`、`TF_LOOKUP_FAILED`、`LOW_TOTAL_POINTS`、`LOW_GROUND_POINTS`、`HIGH_GROUND_RMSE`、`BAD_GROUND_NORMAL`、`LOW_WALL_POINTS`、`RANSAC_FAILED`、`PCA_FAILED`、`PCA_INCONSISTENT`、`LOW_DISTANCE_POINTS`、`CLEARANCE_INVALID`、`DISTANCE_JUMP_REJECTED`。
- 距离在 ROS 消息中优先使用米制字段，例如 `left_front_clearance_m`、`right_front_clearance_m`、`left_rear_clearance_m`、`right_rear_clearance_m`；现有 `*_mm` 字段仅用于兼容当前网页显示。
- 左侧距离使用 `y > 0` 点的 `y` 作为侧向距离；右侧距离使用 `y < 0` 点的 `-y`，等价于 `abs(y)`。最终统一计算 `clearance_m = wall_distance_m - equipment_half_width_m`。

## TCP 惯导与毫米波雷达配置

惯导和毫米波雷达真实值由 `modbus_sensor_reference_node` 通过 MODBUS TCP 读取，并统一发布到 `/sensor_reference`。现场惯导和毫米波雷达使用同一个 TCP 设备 IP；Docker 部署后可直接修改挂载到容器内的外挂 YAML，无需重建镜像。

`/sensor_reference` 复用 `EquipmentState` 消息，状态语义与 `/equipment_state` 保持一致：启用的惯导和毫米波都读取成功时为 `OK`；部分启用传感器读取成功时为 `DEGRADED`；全部读取失败为 `LOST`。未启用的单项使用 `SENSOR_DISABLED`，TCP 读取失败使用 `TCP_READ_FAILED`。

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

- `output_topic`：MODBUS 真实参考值输出话题，默认 `/sensor_reference`。
- `frame_id`：`/sensor_reference` 消息使用的坐标系名称，默认 `base_link`。
- `rate_hz`：MODBUS 轮询和发布频率，单位 Hz。
- `host`：TCP 设备 IP 地址。当前现场方案中惯导和毫米波共用 `modbus.host`，因此现场换 IP 时只修改这一项。
- `port`：MODBUS TCP 端口，常用 `502`。当前现场方案中惯导和毫米波共用 `modbus.port`，因此现场换端口时只修改这一项。
- `unit_id`：MODBUS 从站 ID。当前现场方案中惯导和毫米波共用 `modbus.unit_id`，因此只配置一次。
- `timeout_ms`：TCP 读写超时时间，单位毫秒。当前现场方案中惯导和毫米波共用 `modbus.timeout_ms`，因此只配置一次。
- `ins.enabled`：是否读取惯导寄存器。为 `false` 时惯导角度字段标记为 `SENSOR_DISABLED`。
- `mmwave.enabled`：是否读取 4 路毫米波寄存器。为 `false` 时四角真实距离字段标记为 `SENSOR_DISABLED`。
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
