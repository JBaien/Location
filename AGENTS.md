# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS1 multi-LiDAR target localization stack. `SLAM.md` is the design reference; implementation lives under `catkin_ws/src`.

- `catkin_ws/src/timoo*`: TM16 driver, point cloud conversion, and messages.
- `catkin_ws/src/lidar_fusion`: multi-LiDAR synchronization and `/points_raw` fusion.
- `catkin_ws/src/target_localizer`: cylindrical target detection and `/target_xy` output.
- `catkin_ws/src/mine_slam_web`: WebSocket bridge and browser viewer.
- `docker/runtime`: field-editable launch/YAML mounted into containers.

## Build, Test, and Development Commands

Use ROS/catkin and Docker from the repository root:

- `source /opt/ros/noetic/setup.bash && cd catkin_ws && catkin_make -DCMAKE_BUILD_TYPE=Release`: build locally.
- `cd catkin_ws && catkin_make run_tests_target_localizer -DCMAKE_BUILD_TYPE=Release`: run target-localizer tests.
- `docker compose -f docker/docker-compose.yml build`: build x86 runtime image.
- `./docker/build_arm64.sh`: build and export the arm64 deployment image.

## Coding Style & Naming Conventions

Use C++14 for ROS nodes unless a package already requires newer C++. Keep headers in `include/<package>/`, sources in `src/`, launch files in `launch/`, and YAML in `config/`. ROS topics, nodes, params, and files use `snake_case`; classes use `PascalCase`.

For Markdown, use ATX headings, concise paragraphs, and fenced code blocks with language tags. Preserve existing Chinese technical terminology in `SLAM.md`.

## Testing Guidelines

Add focused gtests for geometry, tracker, filtering, and message-level behavior. Use rosbag replay for integration checks covering static, dynamic, and occluded target scenes. Keep hardware-dependent tests out of default unit targets.

## Commit & Pull Request Guidelines

Local Git history is not available in this workspace, so no existing convention can be inferred. Use short imperative commits, optionally scoped: `target: add cylinder tracker` or `docker: mount localization config`.

Pull requests should include a concise summary, affected files, validation performed, and any assumptions. For visual documentation changes, include rendered screenshots or exported diagrams when practical.

## Security & Configuration Tips

Do not commit site-specific calibration secrets, private rosbag data, or machine-local absolute paths unless they are intentional examples. Keep reusable parameters in YAML and document units explicitly, especially meters, radians, milliseconds, and ROS frame names.
