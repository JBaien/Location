# Local TSDF Mesh

`local_tsdf_mesh` creates a short-lived surface around the latest fused LiDAR
scan without requiring a global SLAM map. It registers each scan to the last
accepted scan with PCL Generalized ICP, integrates signed-distance samples into
a sparse rolling volume, extracts an indexed mesh with marching tetrahedra, and
publishes the mesh as a ROS `visualization_msgs/Marker` `TRIANGLE_LIST`.

## Topics

- Input: `/points_raw` (`sensor_msgs/PointCloud2`)
- Mesh: `/local_tsdf_mesh/mesh` (`visualization_msgs/Marker`)
- State and metrics: `/local_tsdf_mesh/status`
  (`diagnostic_msgs/DiagnosticArray`)

The marker namespace is `local_tsdf_mesh`, its id is `0`, and every three
points form one triangle. The internal TSDF lives in the first accepted scan's
reference coordinates. Before publishing, vertices are transformed into the
latest integrated cloud frame; `header.frame_id` and `header.stamp` therefore
match that cloud. Reset and empty-mesh events publish `Marker::DELETE`.

## Safety behaviour

Registration is accepted only after convergence, finite-transform, fitness,
inlier-ratio, translation and rotation gates. A constant-motion consistency
gate is then applied. Repetitive tunnel walls can make ICP converge to a false
translation with excellent fitness, so `allow_unverified_bootstrap` defaults to
`false`. Without a trusted prior, every first-pair result is then reported as
unobservable. With `safe_single_frame_fallback=true`, GICP is skipped and each
incoming scan atomically replaces a current-frame-only TSDF. It remains useful
for display without ever mixing timestamps. A separate
`allow_near_identity_bootstrap` switch controls verified static-scanner use.
When experimental multi-frame registration is explicitly enabled, a rejected
registration holds the last mesh; after the configured number of consecutive
failures the volume is reset from the current scan.

The status message exposes `state`, `reason`, registration quality, prediction
error, active frames/voxels, eviction counts, triangle count, reset counters and
decode/registration/integration/extraction timings.

## Multi-LiDAR sensor origins

Ray carving must start at the physical source sensor, not at one shared origin.
The node groups points by `lidar_id` and reads an x/y/z origin triple for each id
from `sensors/origins_xyz`. The triples must be expressed in the incoming cloud
frame and updated whenever fusion extrinsics or `output_frame_id` change. With
`require_configured_origin=true`, an unknown `lidar_id` rejects integration
instead of silently carving from the wrong origin. `require_lidar_id=true` also
rejects clouds that omit the field; set it to `false` only for a verified
single-LiDAR input whose sole origin is entry zero.
Set `expected_input_frame` to that same fusion frame so a runtime frame change
cannot silently reuse an origin table expressed in a different coordinate
system.

## Run

```bash
source /opt/ros/noetic/setup.bash
cd catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch local_tsdf_mesh local_tsdf_mesh.launch
```

The deployment image uses ROS Melodic/PCL 1.8; the implementation is C++14 and
uses APIs available there as well.

## Current limits

- This is a local 0.5--2 second rolling surface, not a loop-closed global map.
- ICP-only translation in a feature-poor straight tunnel is fundamentally
  weakly observable. The safe default intentionally stays in single-frame mode
  until better geometry or an independently validated motion prior is available.
- The first version uses scan-level rigid registration; it does not deskew each
  point with the fused `time` field.
- Sparse TSDF extraction only emits tetrahedra whose four corner samples have
  sufficient weight. Completely unobserved areas remain open rather than being
  filled heuristically.
