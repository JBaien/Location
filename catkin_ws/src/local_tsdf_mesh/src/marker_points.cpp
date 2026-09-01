#include "local_tsdf_mesh/marker_points.h"

#include <array>
#include <cstdint>
#include <vector>

namespace local_tsdf_mesh {

TriangleListPointResult buildTriangleListPoints(
    const IndexedMesh& mesh, const Eigen::Matrix4f& output_from_mesh) {
  TriangleListPointResult result;
  result.points.reserve(mesh.triangles.size() * 3U);

  std::vector<geometry_msgs::Point> transformed_vertices(mesh.vertices.size());
  std::vector<std::uint8_t> valid_vertices(mesh.vertices.size(), 0U);
  for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
    const Eigen::Vector3f& vertex = mesh.vertices[index];
    const Eigen::Vector3f transformed =
        (output_from_mesh *
         Eigen::Vector4f(vertex.x(), vertex.y(), vertex.z(), 1.0f))
            .head<3>();
    if (!transformed.allFinite()) {
      continue;
    }
    geometry_msgs::Point& point = transformed_vertices[index];
    point.x = transformed.x();
    point.y = transformed.y();
    point.z = transformed.z();
    valid_vertices[index] = 1U;
  }

  for (const Eigen::Vector3i& triangle : mesh.triangles) {
    std::array<std::size_t, 3> indices;
    bool valid_triangle = true;
    for (int corner = 0; corner < 3; ++corner) {
      const int vertex_index = triangle[corner];
      if (vertex_index < 0 ||
          static_cast<std::size_t>(vertex_index) >= mesh.vertices.size()) {
        valid_triangle = false;
        break;
      }
      const std::size_t index = static_cast<std::size_t>(vertex_index);
      if (valid_vertices[index] == 0U) {
        valid_triangle = false;
        break;
      }
      indices[static_cast<std::size_t>(corner)] = index;
    }
    if (!valid_triangle) {
      ++result.rejected_triangles;
      continue;
    }
    for (const std::size_t index : indices) {
      result.points.push_back(transformed_vertices[index]);
    }
  }
  return result;
}

}  // namespace local_tsdf_mesh
