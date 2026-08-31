#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/Point.h>

#include "local_tsdf_mesh/sparse_tsdf.h"

namespace local_tsdf_mesh {

struct TriangleListPointResult {
  std::vector<geometry_msgs::Point> points;
  std::size_t rejected_triangles = 0U;
};

// Transforms each indexed vertex once, then expands valid triangles in their
// original index/corner order for a visualization_msgs/Marker TRIANGLE_LIST.
// A triangle containing an invalid index or non-finite transformed vertex is
// rejected atomically.
TriangleListPointResult buildTriangleListPoints(
    const IndexedMesh& mesh, const Eigen::Matrix4f& output_from_mesh);

}  // namespace local_tsdf_mesh
