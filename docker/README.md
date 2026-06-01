# Mine LiDAR Docker Runtime

这个目录用于把当前项目部署到只有 Docker 环境的 Ubuntu 18.04 主机上，目标是：

- 启动 3 个 TM16 雷达驱动
- 融合多雷达实时点云到 `/points_raw`
- 检测圆柱标靶并输出 `/target_xy`
- 启动 WebSocket 点云桥和 Web 页面
- 可选录制原始点云、融合点云和 TF，便于拷贝 bag 到研发电脑离线标定
- `launch` 和 `yaml` 放在镜像外，通过 volume 挂载修改，不需要重建镜像

## 构建

在项目根目录执行：

```bash
docker compose -f docker/docker-compose.yml build
```

默认构建源配置在 `docker/docker-compose.yml` 的 `build.args` 中：

```yaml
  DOCKER_REGISTRY: docker.1ms.run
NODE_IMAGE_TAG: 20-bookworm
UBUNTU_APT_MIRROR: https://mirrors.tuna.tsinghua.edu.cn/ubuntu
ROS_APT_MIRROR: https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu
NPM_REGISTRY: https://registry.npmmirror.com
```

如果默认 Docker 镜像站不可用，可以把 `DOCKER_REGISTRY` 改回：

```yaml
DOCKER_REGISTRY: docker.io
```

也可以在主机上配置 Docker Hub registry mirror，例如 `/etc/docker/daemon.json`：

```json
{
  "registry-mirrors": [
    "https://docker.mirrors.ustc.edu.cn"
  ]
}
```

修改后重启 Docker：

```bash
sudo systemctl restart docker
```

如果目标主机没有 compose 插件，也可以用：

```bash
docker build -f docker/Dockerfile -t mine-lidar-runtime:melodic .
```

手动指定镜像源：

```bash
docker build -f docker/Dockerfile -t mine-lidar-runtime:melodic \
  --build-arg DOCKER_REGISTRY=docker.1ms.run \
  --build-arg NODE_IMAGE_TAG=20-bookworm \
  --build-arg UBUNTU_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ubuntu \
  --build-arg ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu \
  --build-arg NPM_REGISTRY=https://registry.npmmirror.com \
  .
```

## 构建 arm64 镜像

仓库提供了单独的 arm64 构建文件：

```bash
docker compose -f docker/docker-compose.arm64.yml build
```

生成的镜像名为：

```text
mine-lidar-runtime:melodic-arm64
```

如果在 x86_64 研发电脑上交叉构建，需要先启用 buildx/QEMU，然后指定 `linux/arm64`：

```bash
docker buildx create --use --name mine-lidar-builder
docker buildx build --platform linux/arm64 \
  -f docker/Dockerfile.arm64 \
  -t mine-lidar-runtime:melodic-arm64 \
  --load \
  .
```

也可以直接使用脚本构建并导出压缩包：

```bash
./docker/build_arm64.sh
```

脚本默认使用代理：

```text
http://192.168.146.1:7890
```

如需换代理：

```bash
PROXY=http://代理IP:端口 ./docker/build_arm64.sh
```

如果要直接打包给 arm64 现场主机加载：

```bash
docker save mine-lidar-runtime:melodic-arm64 | gzip > mine-lidar-runtime-melodic-arm64.tar.gz
```

在 arm64 主机上加载：

```bash
gunzip -c mine-lidar-runtime-melodic-arm64.tar.gz | docker load
```

arm64 版使用 Ubuntu 18.04 的 `ubuntu-ports` 软件源，默认配置在 `docker/docker-compose.arm64.yml` 中：

```yaml
DOCKER_REGISTRY: docker.1ms.run
UBUNTU_APT_MIRROR: https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
```

这里没有继续使用 `docker.mirrors.ustc.edu.cn`，因为当前环境下该域名解析失败，会卡在拉取 `node` 或 `ros` 基础镜像之前。

## 现场部署包

推荐把完整部署包拷贝到现场主机。部署包包含镜像压缩包、compose 文件、`runtime` 外挂配置和校验文件：

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

如果只分发 arm64 镜像和 `docker/` 目录，也可以放到 `/opt/mine-lidar`：

```text
/opt/mine-lidar/
├── mine-lidar-runtime-melodic-arm64.tar.gz
└── docker/
    ├── docker-compose.arm64.yml
    └── runtime/
```

加载后应能看到：

```text
mine-lidar-runtime:melodic-arm64
```

部署后现场可直接修改 `deploy-*/docker/runtime` 或 `/opt/mine-lidar/docker/runtime` 下的 launch/YAML，例如雷达 TF、融合参数、设备状态参数、MODBUS TCP IP 和端口；修改后重启容器即可生效。

如果现场主机没有 compose 插件，可以直接用 `docker run`：

```bash
cd /opt/mine-lidar
docker rm -f mine-lidar-runtime-arm64 2>/dev/null || true
docker run -d \
  --name mine-lidar-runtime-arm64 \
  --restart unless-stopped \
  --privileged \
  --network host \
  -e ROS_MASTER_URI=http://127.0.0.1:11311 \
  -e ROS_IP=127.0.0.1 \
  -e WEB_HTTP_PORT=8080 \
  -v /opt/mine-lidar/docker/runtime:/config \
  -v /opt/mine-lidar/docker/runtime/bags:/data/bags \
  mine-lidar-runtime:melodic-arm64 \
  bringup
```

## 启动

推荐使用 host 网络，雷达 UDP 驱动更稳：

```bash
docker compose -f docker/docker-compose.yml up -d
```

arm64 镜像使用对应 compose 文件启动：

```bash
docker compose -f docker/docker-compose.arm64.yml up -d
```

现场 arm64 主机推荐在部署目录执行：

```bash
cd /opt/mine-lidar
docker compose -f docker/docker-compose.arm64.yml up -d
```

停止：

```bash
docker compose -f docker/docker-compose.arm64.yml down
```

重启：

```bash
docker compose -f docker/docker-compose.arm64.yml restart
```

查看运行状态：

```bash
docker ps --filter name=mine-lidar-runtime-arm64
docker logs -f --tail 200 mine-lidar-runtime-arm64
```

浏览器访问：

```text
http://主机IP:8080
```

WebSocket 端口：

- 点云：`9001`
- 状态：`9002`

## 外挂配置

默认挂载目录：

```text
docker/runtime:/config
docker/runtime/bags:/data/bags
```

现场主要修改这些文件：

- `docker/runtime/launch/driver_tm16_multi3.launch`
- `docker/runtime/launch/driver_tmlidar_multi3.launch`
- `docker/runtime/launch/bringup.launch`
- `docker/runtime/lidar_fusion/multi_lidar_fusion_timoo.yaml`
- `docker/runtime/lidar_fusion/multi_lidar_fusion_tmlidar.yaml`
- `docker/runtime/target_localizer/target_localizer.yaml`
- `docker/runtime/web/web_viewer.yaml`

外参建议先写在 `bringup.launch` 中的：

```xml
<arg name="base_to_lidar1_xyz_ypr" value="0 0 0 0 0 0"/>
<arg name="lidar1_to_lidar2_xyz_ypr" value="0 0 0 0 0 0"/>
<arg name="lidar1_to_lidar3_xyz_ypr" value="0 0 0 0 0 0"/>
```

这里的顺序是：

```text
x y z yaw pitch roll parent_frame child_frame
```

修改雷达端口、frame 和过滤配置：

```bash
vim /opt/mine-lidar/docker/runtime/launch/driver_tm16_multi3.launch
docker compose -f /opt/mine-lidar/docker/docker-compose.arm64.yml restart
```

如需改用 tmlidar 驱动，把 `docker/runtime/launch/bringup.launch` 中的 `driver_family` 设为 `tmlidar`，并修改：

```bash
vim /opt/mine-lidar/docker/runtime/launch/driver_tmlidar_multi3.launch
docker compose -f /opt/mine-lidar/docker/docker-compose.arm64.yml restart
```

常用字段：

```xml
<arg name="frame_id" value="lidar1" />
<arg name="port" value="2368" />
<arg name="status_port" value="7603" />
<arg name="min_range" value="0.4" />
<arg name="max_range" value="150.0" />
<arg name="hide_line" value="0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15" />
```

修改融合参数：

```bash
vim /opt/mine-lidar/docker/runtime/lidar_fusion/multi_lidar_fusion_timoo.yaml
docker compose -f /opt/mine-lidar/docker/docker-compose.arm64.yml restart
```

重点参数：

```yaml
lidar_topics:
  - /lidar1/timoo_points
  - /lidar2/timoo_points
  - /lidar3/timoo_points
output_topic: /points_raw
output_frame_id: base_link
sync_slop: 0.05
min_points_per_lidar: 10
```

tmlidar 驱动对应的融合配置是 `multi_lidar_fusion_tmlidar.yaml`，输入话题为 `/lidar1/lidar_points`、`/lidar2/lidar_points`、`/lidar3/lidar_points`。

修改圆柱标靶定位参数：

```bash
vim /opt/mine-lidar/docker/runtime/target_localizer/target_localizer.yaml
docker compose -f /opt/mine-lidar/docker/docker-compose.arm64.yml restart
```

重点参数：

```yaml
input_topic: /points_raw
xy_topic: /target_xy
roi_x_min: -12.0
roi_x_max: -4.0
radius_min: 0.095
radius_max: 0.155
min_good_inliers: 25
```

该文件同时控制三类节点：圆柱标靶定位、点云设备姿态/四角距离估计、MODBUS TCP 真实参考值。完整参数含义见仓库根目录 `README.md` 中“圆柱标靶定位、设备姿态、四角距离和 MODBUS TCP 参数”小节。现场排查时可按下面分组理解：

| 参数分组 | 主要字段 | 用途 |
| --- | --- | --- |
| 输入输出话题 | `input_topic`、`target_frame`、`measurement_topic`、`xy_topic`、`equipment_state_topic` | 选择输入点云、输出标靶位移和设备状态的话题。 |
| 圆柱 ROI | `roi_x_min/max`、`roi_y_min/max`、`roi_z_min/max` | 限定只在哪个空间盒子内寻找圆柱标靶。 |
| 圆柱拟合 | `reflector_intensity_min`、`voxel_leaf`、`sor_mean_k`、`normal_k`、`max_iterations`、`cylinder_distance_threshold`、`radius_min/max` | 先按反光强度提取标靶候选点，再控制降采样、去噪、法线估计和 RANSAC 圆柱拟合。 |
| 标靶输出 | `reference_z`、`zero_x`、`zero_y`、`min_good_inliers`、`good_residual_rms`、`lost_after_misses` | 控制圆柱中心取值高度、建零点和跟踪质量判定。 |
| 设备状态 | `required_frame_id`、`enable_tf_transform`、`max_pointcloud_age_sec`、`min_total_points` | 检查输入点云坐标系、时延和总点数。 |
| 地面/侧壁 | `ground_*`、`wall_*`、`max_ground_plane_rmse`、`min_ground_normal_z`、`max_wall_direction_diff_deg` | 从点云估计横滚、俯仰和偏航。 |
| 四角距离 | `front_sample_distance_m`、`rear_sample_distance_m`、`sample_window_x_m`、`side_*`、`equipment_half_width_m`、`distance_filter_alpha` | 计算左前、左后、右前、右后剩余距离。 |
| MODBUS 参考值 | `modbus.host`、`modbus.port`、`modbus.unit_id`、`modbus.ins.*`、`modbus.mmwave.*` | 读取惯导和毫米波真实参考值，发布到 `/sensor_reference`。 |

修改 Web 显示和 WebSocket 主题：

```bash
vim /opt/mine-lidar/docker/runtime/web/web_viewer.yaml
docker compose -f /opt/mine-lidar/docker/docker-compose.arm64.yml restart
```

容器启动时会把镜像内默认配置复制到 `/config`。如果宿主机 `/opt/mine-lidar/docker/runtime` 已有同名文件，不会覆盖现场修改。

## 自启动

如果使用 compose，`docker-compose.arm64.yml` 已经设置：

```yaml
restart: unless-stopped
```

Docker 服务启动后容器会自动恢复。先确保 Docker 本身开机自启：

```bash
sudo systemctl enable docker
sudo systemctl restart docker
```

如果需要用 systemd 固定管理 compose，创建服务：

先确认 Docker 命令路径：

```bash
command -v docker
```

下面示例使用 `/usr/bin/docker`。如果现场输出是 `/snap/bin/docker`，把服务文件里的 `/usr/bin/docker` 替换成 `/snap/bin/docker`。

```bash
sudo tee /etc/systemd/system/mine-lidar.service >/dev/null <<'EOF'
[Unit]
Description=Mine LiDAR Docker Runtime
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target

[Service]
Type=oneshot
WorkingDirectory=/opt/mine-lidar
RemainAfterExit=yes
ExecStart=/usr/bin/docker compose -f docker/docker-compose.arm64.yml up -d
ExecStop=/usr/bin/docker compose -f docker/docker-compose.arm64.yml down
TimeoutStartSec=0

[Install]
WantedBy=multi-user.target
EOF
```

启用并启动：

```bash
sudo systemctl daemon-reload
sudo systemctl enable mine-lidar.service
sudo systemctl start mine-lidar.service
```

查看服务状态：

```bash
systemctl status mine-lidar.service
journalctl -u mine-lidar.service -f
```

如果现场没有 compose 插件，也可以用纯 Docker systemd 服务：

```bash
sudo tee /etc/systemd/system/mine-lidar-docker-run.service >/dev/null <<'EOF'
[Unit]
Description=Mine LiDAR Docker Runtime by docker run
Requires=docker.service
After=docker.service network-online.target
Wants=network-online.target

[Service]
Restart=always
RestartSec=5
ExecStartPre=-/usr/bin/docker rm -f mine-lidar-runtime-arm64
ExecStart=/usr/bin/docker run --name mine-lidar-runtime-arm64 --privileged --network host \
  -e ROS_MASTER_URI=http://127.0.0.1:11311 \
  -e ROS_IP=127.0.0.1 \
  -e WEB_HTTP_PORT=8080 \
  -v /opt/mine-lidar/docker/runtime:/config \
  -v /opt/mine-lidar/docker/runtime/bags:/data/bags \
  mine-lidar-runtime:melodic-arm64 bringup
ExecStop=/usr/bin/docker stop mine-lidar-runtime-arm64

[Install]
WantedBy=multi-user.target
EOF
```

## 运行检查

进入容器：

```bash
docker exec -it mine-lidar-runtime-arm64 bash
```

查看 ROS 节点：

```bash
source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash
rosnode list
rostopic list
```

正常运行时至少能看到：

```text
/mine_web_bridge_node
/multi_lidar_fusion_node
/target_localizer_node
/points_raw
/target_xy
/tf
/tf_static
```

如果启用了默认 Timoo 雷达驱动，还应看到：

```text
/lidar1/timoo_points
/lidar2/timoo_points
/lidar3/timoo_points
```

如果在 `bringup.launch` 中设置 `driver_family:=tmlidar`，对应原始点云话题为：

```text
/lidar1/lidar_points
/lidar2/lidar_points
/lidar3/lidar_points
```

检查点云频率：

```bash
rostopic hz /lidar1/timoo_points
rostopic hz /lidar2/timoo_points
rostopic hz /lidar3/timoo_points
rostopic hz /points_raw
rostopic hz /target_xy
```

检查融合节点日志：

```bash
docker logs -f --tail 200 mine-lidar-runtime-arm64
```

如果日志里长期显示：

```text
callbacks=0, published=0
```

通常表示没有收到三路原始点云，优先检查雷达供电、网线、主机 IP、UDP 端口、host 网络，以及当前 `driver_family` 对应的 `driver_tm16_multi3.launch` 或 `driver_tmlidar_multi3.launch`。

## 录制点云

启动时开启录制：

```bash
docker compose -f docker/docker-compose.yml run --rm mine-lidar \
  bringup enable_record:=true
```

arm64：

```bash
cd /opt/mine-lidar
docker compose -f docker/docker-compose.arm64.yml run --rm mine-lidar \
  bringup enable_record:=true
```

或直接修改 `/opt/mine-lidar/docker/runtime/launch/bringup.launch`：

```xml
<arg name="enable_record" default="true"/>
```

再重启服务。

bag 会写到：

```text
docker/runtime/bags
```

默认 Timoo 驱动录制：

- `/lidar1/timoo_points`
- `/lidar2/timoo_points`
- `/lidar3/timoo_points`
- `/points_raw`
- `/target_cloud_roi`
- `/target_measurement`
- `/target_xy`
- `/target_marker`
- `/diagnostics`
- `/tf`
- `/tf_static`
- `/rosout`

tmlidar 驱动录制时，三路原始点云改为 `/lidar1/lidar_points`、`/lidar2/lidar_points`、`/lidar3/lidar_points`。

用于多雷达标定时，必须保留三个原始雷达点云和 `/tf_static`，不要只录融合后的 `/points_raw`。

## 工程注意

1. 雷达 UDP 驱动建议使用 `network_mode: host`，不要只做端口映射。
2. 井下现场录 bag 时，优先选择静止、低速、强振动三类片段各录一段。
3. 标定数据最好在交叉口、硐室、设备密集区采，不要只在长直巷道中间采。
4. 如果静止点云就双层，优先查外参方向和 frame_id。
5. 如果静止正常、运动后撕裂，优先查时间同步和点云去畸变。
