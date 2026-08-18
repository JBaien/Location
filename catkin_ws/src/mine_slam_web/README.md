# Mine SLAM Web Viewer

Web viewer for the mine tunnel perception stack. The package is isolated from
the SLAM algorithm path: it subscribes to ROS topics, publishes point clouds
over a binary WebSocket, and publishes small status updates over a JSON
WebSocket.

The current-cloud layer supports both point rendering and a real-time scan-line
surface. The surface is generated independently for each `lidar_id`, using the
original `ring` and per-point `time` fields. Long edges and range jumps are
rejected so points across occlusion boundaries are not connected. Current-scan
voxel limiting is keyed by `lidar_id + ring`, preventing one scan line or sensor
from deleting neighboring scan lines during WebSocket downsampling.

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

Header, little-endian, version 2:

```cpp
struct CloudPacketHeader {
  uint32_t magic;       // 0x4D504344, "MPCD"
  uint16_t version;     // 2
  uint16_t cloud_type;  // 1=current, 2=stable
  uint64_t stamp_ns;
  uint32_t point_count;
  uint32_t fields_mask;
};
```

Point, little-endian, 24 bytes:

```cpp
struct WebPoint {
  float x;
  float y;
  float z;
  float intensity;
  float time;
  uint16_t ring;
  uint8_t lidar_id;
  uint8_t class_id;  // 1=stable, 2=current, 3=reflector
};
```

Backend and frontend should be deployed together because older frontends reject
version-2 packets. The new browser remains able to parse the legacy version-1,
21-byte point packet,
but scan-mesh generation is disabled for that packet because it does not carry
`ring` and `time` separately.
