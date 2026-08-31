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
- Local TSDF mesh: `ws://localhost:9003/mesh`

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
- local TSDF surface;
- stable map;
- path.

Enabling both real-time layers displays semi-transparent triangles underneath
the original points.

## Mesh Diagnostics

The display drawer shows live point, azimuth-match and candidate-quad coverage,
plus Mesh build/apply time and the most frequent rejection reason. It also shows
the runtime voxel size, point cap, current-cloud publisher count and whether the
encoder reached the configured cap. The bridge emits a throttled ROS warning
when the current-cloud topic has more than one publisher.

The browser retains the latest 180 current-cloud frames without logging every
rejected quad. Use the Console to inspect or export them:

```js
const debug = window.__MINE_SLAM_VIEWER_DEBUG__;
console.table(debug.meshFrames().slice(-100).map((frame) => ({
  frame: frame.packet_sequence,
  stamp_ns: frame.stamp_ns,
  points: frame.point_count,
  point_coverage: frame.point_coverage,
  quad_coverage: frame.quad_coverage,
  build_ms: frame.mesh_build_ms,
  apply_ms: frame.mesh_apply_ms,
  held: frame.held
})));
console.table(debug.meshDebug().reject_samples);
```

`meshFrames()` includes stable zero-filled rejection counters, per-ring and
per-ring-pair matching data, range validity, bounded rejection samples and
separate CPU timings for geometry allocation, attribute setup, recoloring and
buffer swap/disposal. `run_normal` is a strip-break counter; quads removed from
a short strip are counted as `isolated_run` and grouped by their flush cause.

`bridgeFrames()` retains timestamped raw/encoded point-count snapshots. These
arrive over the status WebSocket and are therefore correlation snapshots, not a
guaranteed one-to-one field in the binary cloud packet. Match them to Mesh
frames by `stamp_ns` when available. Geometry timing measures browser CPU work;
it does not claim to measure completion of the deferred GPU upload.

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

## Local TSDF Mesh Packet

The bridge accepts a `visualization_msgs/Marker::TRIANGLE_LIST` on
`/local_tsdf_mesh/mesh` and publishes an independent atomic snapshot. A new
snapshot replaces the previous geometry in one browser frame; `DELETE`,
`DELETEALL`, or an empty `ADD` clears it. Slow clients retain only the newest
pending snapshot, and a newly connected client receives the cached latest
snapshot directly.
If the ROS Mesh publisher disappears or stops producing Markers for
`local_tsdf_mesh/stale_timeout_sec`, the bridge replaces its cache with a clear
packet so reconnecting browsers cannot resurrect an obsolete surface.

The packet uses little-endian fields and a 64-byte header:

```cpp
struct MeshPacketHeader {
  uint32_t magic;             // 0x4D4D5348, "MMSH"
  uint16_t version;           // 1
  uint8_t operation;          // 1=replace all, 2=clear
  uint8_t flags;              // bit 0=RGBA, bit 1=uint32 indices
  uint64_t revision;
  uint64_t stamp_ns;
  uint32_t vertex_count;
  uint32_t index_count;
  uint32_t frame_id_offset;
  uint32_t frame_id_bytes;
  uint32_t positions_offset;  // packed float32 XYZ
  uint32_t colors_offset;     // packed uint8 RGBA
  uint32_t indices_offset;    // optional packed uint32
  uint32_t packet_bytes;
  uint32_t reserved0;
  uint32_t reserved1;
};
```

All offsets are measured from the start of the packet. The current Marker
encoder emits a non-indexed triangle list, so `index_count` is zero and every
three consecutive vertices form one triangle. The browser validates all
section boundaries, finite positions, and optional index bounds before
replacing its current geometry.

The mesh coordinates and `frame_id` must match the displayed current cloud.
The local TSDF producer publishes in the latest cloud-local frame, so keep
`current_cloud/transform_body_to_map: false` when overlaying these two layers.
The status stream reports both frame ids and the browser shows a warning when
they differ. The stable-map and scan-strip surface layers are hidden by default
to avoid presenting mixed-frame or duplicate surfaces as one reconstruction.

For a two-TM16 real-time scan, use no additional Web-only voxel filtering when
bandwidth permits:

```yaml
current_cloud:
  send_rate_hz: 0.0
  voxel_size_m: 0.0
  max_points: 200000
```
