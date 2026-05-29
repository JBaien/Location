# Mine SLAM Web Viewer

First-version Web viewer for the mine tunnel SLAM stack. This package is isolated from the SLAM algorithm path: it subscribes to ROS topics, publishes point clouds over a binary WebSocket, and publishes small status updates over a JSON WebSocket.

## Backend

```bash
source devel/setup.bash
roslaunch mine_slam_web mine_web_viewer.launch
```

Default endpoints:

- Binary point cloud: `ws://localhost:9001/cloud`
- JSON status: `ws://localhost:9002/status`

Subscribed topics are configured in `config/web_viewer.yaml`:

- `/cloud_registered`
- `/slam/stable_map`
- `/slam/global_odom`, with `/Odometry` fallback

The backend skips point cloud encoding when no cloud WebSocket client is connected.

## Frontend

```bash
cd src/mine_slam_web/web
npm install
npm run dev
```

Open `http://localhost:5173`.

## Binary Cloud Packet

All large point cloud payloads use binary WebSocket frames, not JSON.

Header, little-endian:

```cpp
struct CloudPacketHeader {
  uint32_t magic;       // 0x4D504344, "MPCD"
  uint16_t version;     // 1
  uint16_t cloud_type;  // 1=current, 2=stable
  uint64_t stamp_ns;
  uint32_t point_count;
  uint32_t fields_mask;
};
```

Point, little-endian, 21 bytes:

```cpp
struct WebPoint {
  float x;
  float y;
  float z;
  float intensity;
  uint8_t lidar_id;
  uint8_t class_id;  // 1=stable, 2=current, 3=reflector
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
```
