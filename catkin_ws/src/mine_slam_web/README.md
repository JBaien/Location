# Mine SLAM Web Viewer

Web viewer for the mine tunnel perception stack. The package is isolated from
the SLAM algorithm path: it subscribes to ROS topics, publishes point clouds
over a binary WebSocket, and publishes small status updates over a JSON
WebSocket.

The current-cloud layer supports point rendering and a real-time scan surface.
The surface is generated independently for each `lidar_id`. Adjacent physical
`ring` values are matched by the source-lidar `azimuth` calculated before TF;
`range` is also retained in the source lidar frame for discontinuity filtering.
The browser therefore does not infer scan columns from transformed XYZ or from
a low-precision float timestamp. Quad planarity, normal agreement, edge length
and range continuity reject connections across occlusions.

## Backend

```bash
source devel/setup.bash
roslaunch mine_slam_web mine_web_viewer.launch
```

Default endpoints:

- Binary point cloud: `ws://localhost:9001/cloud`
- JSON status: `ws://localhost:9002/status`

Subscribed topics are configured in `config/web_viewer.yaml`.

The backend skips point-cloud encoding when no cloud WebSocket client is
connected.

## Frontend

```bash
cd src/mine_slam_web/web
npm install
npm run dev
```

Open `http://localhost:5173`.

The display drawer provides independent switches for:

- real-time points;
- real-time surface;
- stable map;
- path.

Enabling both real-time layers displays semi-transparent triangles underneath
the original points.

## Binary Cloud Packet

Large point-cloud payloads use binary WebSocket frames, not JSON.

Header, little-endian, version 3:

```cpp
struct CloudPacketHeader {
  uint32_t magic;       // 0x4D504344, "MPCD"
  uint16_t version;     // 3
  uint16_t cloud_type;  // 1=current, 2=stable
  uint64_t stamp_ns;
  uint32_t point_count;
  uint32_t fields_mask;
};
```

Point, little-endian, 32 bytes:

```cpp
struct WebPoint {
  float x;
  float y;
  float z;
  float intensity;
  float time;
  float azimuth;   // radians in [0, 2*pi), source lidar frame
  float range;     // metres from source lidar origin
  uint16_t ring;
  uint8_t lidar_id;
  uint8_t class_id;  // 1=stable, 2=current, 3=reflector
};
```

Backend and frontend must be deployed together after a protocol change. The
version-3 browser can still parse the legacy version-1 and version-2 packets,
but scan-mesh generation is fail-closed unless `lidar_id`, `ring` and
`azimuth` are present.

For a two-TM16 real-time scan, use no additional Web-only voxel filtering when
bandwidth permits:

```yaml
current_cloud:
  send_rate_hz: 0.0
  voxel_size_m: 0.0
  max_points: 200000
```
