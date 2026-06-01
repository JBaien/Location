# Graph Report - /home/sf/Desktop/Location  (2026-06-01)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 897 nodes · 1261 edges · 60 communities (47 shown, 13 thin omitted)
- Extraction: 98% EXTRACTED · 2% INFERRED · 0% AMBIGUOUS · INFERRED: 24 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `f3e4efc9`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- [[_COMMUNITY_Mine Web Bridge|Mine Web Bridge]]
- [[_COMMUNITY_Modbus Sensor Reference|Modbus Sensor Reference]]
- [[_COMMUNITY_Binary Cloud Client|Binary Cloud Client]]
- [[_COMMUNITY_Timoo Packet Calibration|Timoo Packet Calibration]]
- [[_COMMUNITY_Equipment State Snapshot|Equipment State Snapshot]]
- [[_COMMUNITY_WebSocket Server|WebSocket Server]]
- [[_COMMUNITY_Equipment State Estimation|Equipment State Estimation]]
- [[_COMMUNITY_Driver Diagnostics|Driver Diagnostics]]
- [[_COMMUNITY_Equipment Geometry Clearance|Equipment Geometry Clearance]]
- [[_COMMUNITY_Point Cloud Node|Point Cloud Node]]
- [[_COMMUNITY_Target Localizer|Target Localizer]]
- [[_COMMUNITY_Cylinder Detection Config|Cylinder Detection Config]]
- [[_COMMUNITY_Cloud Encode Web|Cloud Encode Web]]
- [[_COMMUNITY_Map Transform Snapshot|Map Transform Snapshot]]
- [[_COMMUNITY_Timoo Cloud Node|Timoo Cloud Node]]
- [[_COMMUNITY_JSON Status Builder|JSON Status Builder]]
- [[_COMMUNITY_TypeScript Config|TypeScript Config]]
- [[_COMMUNITY_Cloud Path Tracking|Cloud Path Tracking]]
- [[_COMMUNITY_Vue Three.js Package|Vue Three.js Package]]
- [[_COMMUNITY_Cylinder Fitting Tracker|Cylinder Fitting Tracker]]
- [[_COMMUNITY_Equipment Geometry Tests|Equipment Geometry Tests]]
- [[_COMMUNITY_Transform Node|Transform Node]]
- [[_COMMUNITY_GPS Time Conversion|GPS Time Conversion]]
- [[_COMMUNITY_Progressive Reveal State|Progressive Reveal State]]
- [[_COMMUNITY_Target XY Snapshot|Target XY Snapshot]]
- [[_COMMUNITY_Point Cloud Downsampler|Point Cloud Downsampler]]
- [[_COMMUNITY_Laser Calibration|Laser Calibration]]
- [[_COMMUNITY_Lidar Fusion Node|Lidar Fusion Node]]
- [[_COMMUNITY_Odometry Handling|Odometry Handling]]
- [[_COMMUNITY_Equipment State Callbacks|Equipment State Callbacks]]
- [[_COMMUNITY_Target Tracker|Target Tracker]]
- [[_COMMUNITY_Multi Lidar Fusion|Multi Lidar Fusion]]
- [[_COMMUNITY_Status Client|Status Client]]
- [[_COMMUNITY_Key-Value Diagnostics|Key-Value Diagnostics]]
- [[_COMMUNITY_Entrypoint Script|Entrypoint Script]]
- [[_COMMUNITY_Calibration Generation|Calibration Generation]]
- [[_COMMUNITY_ARM64 Build Script|ARM64 Build Script]]
- [[_COMMUNITY_Cylinder Geometry Test|Cylinder Geometry Test]]
- [[_COMMUNITY_Timoo Packages|Timoo Packages]]
- [[_COMMUNITY_Documentation Files|Documentation Files]]
- [[_COMMUNITY_Settings and Hooks|Settings and Hooks]]
- [[_COMMUNITY_Input Header|Input Header]]
- [[_COMMUNITY_Ring Sequence|Ring Sequence]]
- [[_COMMUNITY_Vite Environment|Vite Environment]]
- [[_COMMUNITY_Lidar Fusion Package|Lidar Fusion Package]]
- [[_COMMUNITY_Mine SLAM Web Package|Mine SLAM Web Package]]
- [[_COMMUNITY_Timoo Laser Scan Package|Timoo Laser Scan Package]]
- [[_COMMUNITY_Timoo Metapackage|Timoo Metapackage]]
- [[_COMMUNITY_Web README|Web README]]
- [[_COMMUNITY_Docker README|Docker README]]
- [[_COMMUNITY_Runtime Lidar Fusion Config|Runtime Lidar Fusion Config]]

## God Nodes (most connected - your core abstractions)
1. `mine web bridge node` - 132 edges
2. `EquipmentStateSnapshot` - 53 edges
3. `target localizer node` - 51 edges
4. `modbus sensor reference node` - 42 edges
5. `equipment state node` - 31 edges
6. `NodeConfig` - 29 edges
7. `WebSocketSession` - 24 edges
8. `WebSocketServer` - 22 edges
9. `compilerOptions` - 16 edges
10. `string` - 15 edges

## Surprising Connections (you probably didn't know these)
- `target localizer node` --conceptually_related_to--> `Cylinder target concept`  [INFERRED]
  catkin_ws/src/target_localizer/CMakeLists.txt → SLAM.md
- `target localizer node` --conceptually_related_to--> `ROI cropping concept`  [INFERRED]
  catkin_ws/src/target_localizer/CMakeLists.txt → SLAM.md
- `mine web bridge node` --references--> `shared_ptr`  [EXTRACTED]
  catkin_ws/src/mine_slam_web/CMakeLists.txt → catkin_ws/src/mine_slam_web/src/mine_web_bridge_node.cpp
- `mine web bridge node` --references--> `uint8_t`  [EXTRACTED]
  catkin_ws/src/mine_slam_web/CMakeLists.txt → catkin_ws/src/mine_slam_web/src/mine_web_bridge_node.cpp
- `target localizer node` --defines--> `Measurement`  [EXTRACTED]
  catkin_ws/src/target_localizer/CMakeLists.txt → catkin_ws/src/target_localizer/src/target_tracker.cpp

## Import Cycles
- None detected.

## Communities (60 total, 13 thin omitted)

### Community 0 - "Mine Web Bridge"
Cohesion: 0.03
Nodes (67): atomic, NodeHandle, Subscriber, Timer, mine web bridge node, OpenSSL library, runtime web viewer config, clock_sub_ (+59 more)

### Community 1 - "Modbus Sensor Reference"
Cohesion: 0.05
Nodes (52): Boost C++ libraries, EquipmentState, io_service, NodeHandle, Publisher, string, T, TimerEvent (+44 more)

### Community 2 - "Binary Cloud Client"
Cohesion: 0.06
Nodes (13): CloudHandler, ConnectionHandler, clamp(), colorForPoint(), ColorMode, ColorPoint, distanceRamp(), PathLayer (+5 more)

### Community 3 - "Timoo Packet Calibration"
Cohesion: 0.05
Nodes (41): namespace, namespace, namespace, namespace, ConstPtr, shared_ptr, string, TransformListener (+33 more)

### Community 4 - "Equipment State Snapshot"
Cohesion: 0.04
Nodes (46): EquipmentStateSnapshot, attitude_valid, distances_valid, ground_plane_rmse, ground_points, invalid_reason, left_front_clearance_m, left_front_invalid_reason (+38 more)

### Community 5 - "WebSocket Server"
Cohesion: 0.09
Nodes (40): namespace, array, atomic, io_service, shared_ptr, size_t, string, uint16_t (+32 more)

### Community 6 - "Equipment State Estimation"
Cohesion: 0.06
Nodes (34): EquipmentGeometryConfig, EquipmentGeometryResult, NodeHandle, PointCloud2, PointCloud2ConstPtr, Publisher, size_t, string (+26 more)

### Community 7 - "Driver Diagnostics"
Cohesion: 0.07
Nodes (29): namespace, NodeHandle, string, TimerEvent, shared_ptr, namespace, shared_ptr, NodeHandle (+21 more)

### Community 8 - "Equipment Geometry Clearance"
Cohesion: 0.12
Nodes (32): EquipmentGeometryConfig, EquipmentGeometryResult, PointCloud, string, vector, Vector3d, PointXYZI, distanceInvalidReason() (+24 more)

### Community 9 - "Point Cloud Node"
Cohesion: 0.12
Nodes (27): namespace, NodeHandle, PointCloud, Ptr, string, T, Time, TimerEvent (+19 more)

### Community 10 - "Target Localizer"
Cohesion: 0.07
Nodes (27): namespace, NodeHandle, Publisher, Subscriber, TrackerConfig, uint64_t, /diagnostics topic, Eigen library (+19 more)

### Community 11 - "Cylinder Detection Config"
Cohesion: 0.07
Nodes (28): NodeConfig, cylinder_distance_threshold, input_topic, marker_topic, max_iterations, measurement_topic, min_candidate_points, normal_distance_weight (+20 more)

### Community 12 - "Cloud Encode Web"
Cohesion: 0.16
Nodes (20): namespace, PointCloud2, size_t, string, T, Time, uint64_t, uint8_t (+12 more)

### Community 13 - "Map Transform Snapshot"
Cohesion: 0.12
Nodes (17): array, vector, MapTransformSnapshot, rotation, translation, PathPoint, stamp, PoseState (+9 more)

### Community 14 - "Timoo Cloud Node"
Cohesion: 0.10
Nodes (14): namespace, shared_ptr, ConstPtr, NodeHandle, string, timooStatus, CloudNodeConfig, conv_ (+6 more)

### Community 15 - "JSON Status Builder"
Cohesion: 0.15
Nodes (4): string, TimerEvent, ostringstream, StringConstPtr

### Community 16 - "TypeScript Config"
Cohesion: 0.11
Nodes (18): compilerOptions, allowJs, allowSyntheticDefaultImports, esModuleInterop, forceConsistentCasingInFileNames, isolatedModules, jsx, lib (+10 more)

### Community 17 - "Cloud Path Tracking"
Cohesion: 0.14
Nodes (9): PointCloud2, PointCloud2ConstPtr, shared_ptr, size_t, Time, uint8_t, ClockConstPtr, CloudType (+1 more)

### Community 18 - "Vue Three.js Package"
Cohesion: 0.11
Nodes (17): dependencies, three, vite, @vitejs/plugin-vue, vue, devDependencies, @types/three, typescript (+9 more)

### Community 19 - "Cylinder Fitting Tracker"
Cohesion: 0.20
Nodes (9): CylinderModel, PointCloud2ConstPtr, Ptr, TrackerOutput, Vector3d, CloudT, Header, statusText() (+1 more)

### Community 20 - "Equipment Geometry Tests"
Cohesion: 0.12
Nodes (14): namespace, EmitsGoodThenLostAfterConsecutiveMisses, EquipmentGeometryTest, EstimatesRollPitchFromGroundPlane, EstimatesYawFromTunnelWalls, IntersectsAxisAtReferenceHeight, KeepsSmallNegativeClearanceButRejectsLargeNegative, MeasuresFourSideDistancesAtConfiguredStations (+6 more)

### Community 21 - "Transform Node"
Cohesion: 0.12
Nodes (11): namespace, ConstPtr, NodeHandle, string, shared_ptr, tf_, processScan(), reconfigure_callback() (+3 more)

### Community 22 - "GPS Time Conversion"
Cohesion: 0.23
Nodes (11): Time, timooPacket, timooPacket, poll(), getPacket(), gpsInfo, gps_status, gps_time (+3 more)

### Community 23 - "Progressive Reveal State"
Cohesion: 0.18
Nodes (11): ProgressiveRevealState, enabled, face_wall_s, filter_front_unrevealed_point_count, hidden_reflector_count, hidden_unrevealed_point_count, machine_s, published_face_point_count (+3 more)

### Community 24 - "Target XY Snapshot"
Cohesion: 0.18
Nodes (11): TargetXySnapshot, center_x_mm, center_y_mm, dx_mm, dy_mm, seen, stamp_ns, status (+3 more)

### Community 25 - "Point Cloud Downsampler"
Cohesion: 0.24
Nodes (10): namespace, size_t, uint64_t, int64_t, mine_slam_web(), accept(), acceptedCount(), splitmix64() (+2 more)

### Community 26 - "Laser Calibration"
Cohesion: 0.22
Nodes (10): namespace, string, T, LaserCorrection, operator>>(), read(), write(), Node (+2 more)

### Community 27 - "Lidar Fusion Node"
Cohesion: 0.33
Nodes (9): ConstPtr, NodeHandle, PointCloud2, string, cloudCallback(), fusePointClouds(), LidarFusion(), loadParameters() (+1 more)

### Community 28 - "Odometry Handling"
Cohesion: 0.28
Nodes (4): Odometry, OdometryConstPtr, OdomSource, distance()

### Community 29 - "Equipment State Callbacks"
Cohesion: 0.25
Nodes (3): uint64_t, EquipmentStateConstPtr, TargetXYConstPtr

### Community 30 - "Target Tracker"
Cohesion: 0.25
Nodes (7): TrackerConfig, TrackerOutput, Measurement, markMissed(), TargetTracker(), update(), TargetTracker

### Community 31 - "Multi Lidar Fusion"
Cohesion: 0.25
Nodes (7): /lidar1/timoo_points topic, lidar_fusion config, lidar_fusion/README.md, multi lidar fusion node, Point Cloud Library (PCL), Point cloud fusion concept, /points_raw topic

### Community 33 - "Key-Value Diagnostics"
Cohesion: 0.38
Nodes (6): string, T, Time, KeyValue, kv(), toString()

### Community 34 - "Entrypoint Script"
Cohesion: 0.40
Nodes (5): ROS_HOME, ROS_IP, ROS_MASTER_URI, serve_web(), entrypoint.sh script

### Community 35 - "Calibration Generation"
Cohesion: 0.33
Nodes (5): addLaserCalibration(), # TODO: make sure all required fields are present., handle XML calibration error, Define key and corresponding value for laser_num, xmlError()

### Community 36 - "ARM64 Build Script"
Cohesion: 0.40
Nodes (4): HTTP_PROXY, HTTPS_PROXY, NO_PROXY, build_arm64.sh script

### Community 37 - "Cylinder Geometry Test"
Cohesion: 0.60
Nodes (5): CylinderModel, Vector3d, CylinderGeometryTest, centerAtReferenceHeight(), radialResidual()

### Community 38 - "Timoo Packages"
Cohesion: 0.40
Nodes (5): libpcap library, timoo_driver package, timoo_msgs package, timoo_pointcloud package, yaml-cpp library

## Knowledge Gaps
- **422 isolated node(s):** `PreToolUse`, `class`, `namespace`, `NodeHandle`, `string` (+417 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **13 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `mine web bridge node` connect `Mine Web Bridge` to `Modbus Sensor Reference`, `Equipment State Snapshot`, `WebSocket Server`, `Equipment State Estimation`, `Driver Diagnostics`, `Target Localizer`, `Map Transform Snapshot`, `JSON Status Builder`, `Cloud Path Tracking`, `Progressive Reveal State`, `Target XY Snapshot`, `Odometry Handling`, `Equipment State Callbacks`, `Multi Lidar Fusion`?**
  _High betweenness centrality (0.348) - this node is a cross-community bridge._
- **Why does `target localizer node` connect `Target Localizer` to `Modbus Sensor Reference`, `Key-Value Diagnostics`, `Cylinder Geometry Test`, `Equipment State Estimation`, `Documentation Files`, `Cylinder Detection Config`, `Cylinder Fitting Tracker`, `Target Tracker`, `Multi Lidar Fusion`?**
  _High betweenness centrality (0.151) - this node is a cross-community bridge._
- **Why does `WebSocketServer` connect `WebSocket Server` to `Mine Web Bridge`, `Driver Diagnostics`?**
  _High betweenness centrality (0.108) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `target localizer node` (e.g. with `Cylinder target concept` and `Kalman filter concept`) actually correct?**
  _`target localizer node` has 4 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `equipment state node` (e.g. with `Equipment state estimation concept` and `ROI cropping concept`) actually correct?**
  _`equipment state node` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `PreToolUse`, `class`, `namespace` to the rest of the system?**
  _425 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Mine Web Bridge` be split into smaller, more focused modules?**
  _Cohesion score 0.029411764705882353 - nodes in this community are weakly interconnected._