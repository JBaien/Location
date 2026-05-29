# LiDAR Fusion

该包提供一个面向煤矿井下长巷道 SLAM 前端输入的多激光雷达点云融合节点。新节点以原有 `lidar_fusion` 两雷达 TF 融合思路为基础，改造成支持 2 路 / 3 路 TM16 点云输入，默认输出给 FAST-LIO / LIO-SAM 类前端使用的 `/points_raw`。

旧的 `lidar_fusion_node` 仍保留；工程推荐使用新的 `multi_lidar_fusion_node`。

## 功能

- 支持 2 路或 3 路 `sensor_msgs/PointCloud2` 输入。
- 通过参数配置 `lidar_num` 和 `lidar_topics`，不需要第 4 路 topic 占位。
- 通过 TF2 查询每路雷达到 `output_frame_id` 的变换。
- 默认输出 `/points_raw`，`frame_id=base_link`。
- 保留 `x / y / z / intensity / ring / time` 字段。
- 输出点云设置正确的 `header.stamp` 和 `header.frame_id`。
- 可选点内时间归一化，将不同雷达 scan 的点内 `time` 平移到同一个输出 header 时间基准。
- 可选 voxel filter。
- 周期性输出 diagnostics，便于现场排查同步、TF、空点云、点数异常。

## 坐标系约定

推荐 TF 树：

```text
base_link
  └── lidar1
        ├── lidar2
        └── lidar3
```

每个 TM16 driver 输出点云必须带不同 `frame_id`：

```text
/lidar1/timoo_points  frame_id: lidar1
/lidar2/timoo_points  frame_id: lidar2
/lidar3/timoo_points  frame_id: lidar3
```

静态外参由独立标定 launch 维护，不建议写死在融合节点里：

```xml
<node pkg="tf2_ros" type="static_transform_publisher"
      name="base_to_lidar1"
      args="x y z yaw pitch roll base_link lidar1" />
<node pkg="tf2_ros" type="static_transform_publisher"
      name="lidar1_to_lidar2"
      args="x y z yaw pitch roll lidar1 lidar2" />
<node pkg="tf2_ros" type="static_transform_publisher"
      name="lidar1_to_lidar3"
      args="x y z yaw pitch roll lidar1 lidar3" />
```

注意：ROS Noetic 下 `tf2_ros/static_transform_publisher` 的欧拉角参数顺序是 `yaw pitch roll`。`x y z yaw pitch roll` 必须替换为实测外参。井下强振动平台上，外参支架刚性和冲击后复检比算法补偿更关键。

如果希望直接由 `multi_lidar_fusion.launch` 发布上述静态 TF，可使用：

```bash
roslaunch lidar_fusion multi_lidar_fusion.launch \
  publish_static_tf:=true \
  base_to_lidar1_xyz_ypr:="0 0 0 0 0 0" \
  lidar1_to_lidar2_xyz_ypr:="x y z yaw pitch roll" \
  lidar1_to_lidar3_xyz_ypr:="x y z yaw pitch roll"
```

## 参数

默认配置文件：

```text
config/multi_lidar_fusion.yaml
```

参数示例：

```yaml
multi_lidar_fusion:
  lidar_num: 3

  lidar_topics:
    - /lidar1/timoo_points
    - /lidar2/timoo_points
    - /lidar3/timoo_points

  output_topic: /points_raw
  output_frame_id: base_link

  sync_queue_size: 20
  sync_slop: 0.05

  tf_timeout: 0.05

  drop_empty_cloud: true
  min_points_per_lidar: 10

  normalize_point_time: true

  enable_voxel_filter: false
  voxel_leaf_size: 0.05

  enable_diagnostics: true
  diagnostics_period: 1.0
```

关键参数说明：

- `lidar_num`：支持 `2` 或 `3`。
- `lidar_topics`：长度必须等于 `lidar_num`。
- `sync_slop`：多路点云最大同步时间差。TM16 未硬同步时可从 `0.05` 开始，现场应尽量压小。
- `tf_timeout`：TF 查询等待时间。静态 TF 正常时不应频繁超时。
- `normalize_point_time`：推荐开启。输出 header 使用同步组内最早时间戳，每路点的 `time` 加上该路 scan header 到输出 header 的时间偏移。
- `enable_voxel_filter`：默认关闭。SLAM 前端通常自己有降采样策略，融合节点不应默认破坏原始点云密度。

## 编译

在包含该包的 catkin 工作空间中编译：

```bash
cd /home/sf/Desktop/multi-sensor-slam-tookit/PointClouds_Fusion/lidar_fusion
catkin_make
source devel/setup.bash
```

如果该包被复制到其他 catkin 工作空间，则在对应工作空间根目录执行 `catkin_make`。

## 启动

先启动三台 TM16，并确认 topic 与 frame：

```bash
rostopic hz /lidar1/timoo_points
rostopic echo -n1 /lidar1/timoo_points/header
rostopic echo -n1 /lidar2/timoo_points/header
rostopic echo -n1 /lidar3/timoo_points/header
```

再启动静态 TF 外参，然后启动融合节点：

```bash
roslaunch lidar_fusion multi_lidar_fusion.launch
```

使用自定义配置：

```bash
roslaunch lidar_fusion multi_lidar_fusion.launch \
  config:=/path/to/your_multi_lidar_fusion.yaml
```

验证输出：

```bash
rostopic hz /points_raw
rostopic echo -n1 /points_raw/header
rosrun tf tf_echo base_link lidar1
rosrun tf tf_echo base_link lidar2
rosrun tf tf_echo base_link lidar3
```

## 接入 FAST-LIO / LIO-SAM

推荐将前端点云输入改为：

```text
/points_raw
```

输出坐标系：

```text
base_link
```

如果前端要求 LiDAR 坐标系而不是 `base_link`，建议统一在前端配置中明确 `T_lidar_imu` / `T_base_imu`，不要让融合节点和 LIO 前端同时做一遍不清晰的外参变换。

## 现场排查

常见问题：

1. `/points_raw` 没有输出
   - 检查三路输入 topic 是否都有频率。
   - 检查 `lidar_topics` 数量是否等于 `lidar_num`。
   - 检查 `sync_slop` 是否过小。

2. 日志提示 TF unavailable
   - 检查点云 `frame_id` 是否分别是 `lidar1 / lidar2 / lidar3`。
   - 检查是否存在 `base_link -> lidarX` 静态 TF。
   - 运行 `rosrun tf view_frames` 查看 TF 树。

3. FAST-LIO / LIO-SAM 畸变或抖动明显
   - 检查 TM16 是否硬件同步。
   - 检查 `/points_raw` 的 header 时间戳是否稳定递增。
   - 检查每路点云是否有 `time` 字段。
   - 检查强振动下外参支架是否松动或共振。

4. 点云拼接错位
   - 先停车采集静态 bag，确认三雷达外参。
   - 再低速行走复查，区分外参问题和时间同步问题。
   - 不建议在工作面强振动状态下直接调外参。

## Diagnostics

开启 `enable_diagnostics` 后，节点会周期输出：

- 同步回调次数
- 已发布帧数
- 每路最近点数和 frame
- 最近同步时间跨度
- 空点云、点数不足、字段缺失、TF、转换异常等丢帧计数

这些日志应和 rosbag 一起保存，建议现场录制：

```bash
rosbag record \
  /lidar1/timoo_points /lidar2/timoo_points /lidar3/timoo_points \
  /points_raw /tf /tf_static /imu/data
```
