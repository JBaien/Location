# Mine LiDAR Target Localization

ROS1 workspace for TM16 three-LiDAR capture, fused point cloud publishing, cylindrical target XY localization, Web point cloud viewing, and Docker deployment.

## Structure

- `catkin_ws/src/timoo*`: TM16 driver, point cloud conversion, and driver messages.
- `catkin_ws/src/lidar_fusion`: synchronizes 2 or 3 LiDAR point clouds and publishes `/points_raw` in `base_link`.
- `catkin_ws/src/target_localizer`: detects the cylindrical target in `/points_raw` and publishes `/target_xy`.
- `catkin_ws/src/mine_slam_web`: WebSocket bridge and browser point cloud viewer.
- `docker/runtime`: field-editable launch and YAML files mounted at `/config`.

## Build

```bash
source /opt/ros/noetic/setup.bash
cd catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
```

On deployment hosts, prefer Docker:

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up -d
```

For arm64 packaging:

```bash
./docker/build_arm64.sh
```

## Run

Default Docker bringup starts the TM16 drivers, multi-LiDAR fusion, target localization, Web bridge, and static Web server.

```bash
docker compose -f docker/docker-compose.yml up -d
docker logs -f --tail 200 mine-lidar-runtime
```

Important topics:

- `/lidar1/timoo_points`, `/lidar2/timoo_points`, `/lidar3/timoo_points`
- `/points_raw`
- `/target_measurement`, `/target_xy`, `/target_marker`, `/target_cloud_roi`
- `/diagnostics`

## Tests

```bash
source /opt/ros/noetic/setup.bash
cd catkin_ws
catkin_make run_tests_target_localizer -DCMAKE_BUILD_TYPE=Release
```

The full local build also requires TM16 driver dependencies such as `libpcap-dev`; the Docker image installs them.
