#include <array>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/Point.h>
#include <gtest/gtest.h>

#include "local_tsdf_mesh/marker_points.h"

namespace local_tsdf_mesh {
namespace {

std::vector<geometry_msgs::Point> legacyTriangleListPoints(
    const IndexedMesh& mesh, const Eigen::Matrix4f& output_from_mesh,
    std::size_t& rejected_triangles) {
  rejected_triangles = 0U;
  std::vector<geometry_msgs::Point> points;
  points.reserve(mesh.triangles.size() * 3U);
  for (const Eigen::Vector3i& triangle : mesh.triangles) {
    bool valid_triangle = true;
    std::array<geometry_msgs::Point, 3> transformed_points;
    for (int corner = 0; corner < 3; ++corner) {
      const int vertex_index = triangle[corner];
      if (vertex_index < 0 ||
          static_cast<std::size_t>(vertex_index) >= mesh.vertices.size()) {
        valid_triangle = false;
        break;
      }
      const Eigen::Vector3f& vertex =
          mesh.vertices[static_cast<std::size_t>(vertex_index)];
      const Eigen::Vector3f transformed =
          (output_from_mesh *
           Eigen::Vector4f(vertex.x(), vertex.y(), vertex.z(), 1.0f))
              .head<3>();
      if (!transformed.allFinite()) {
        valid_triangle = false;
        break;
      }
      geometry_msgs::Point& point =
          transformed_points[static_cast<std::size_t>(corner)];
      point.x = transformed.x();
      point.y = transformed.y();
      point.z = transformed.z();
    }
    if (!valid_triangle) {
      ++rejected_triangles;
      continue;
    }
    points.insert(points.end(), transformed_points.begin(),
                  transformed_points.end());
  }
  return points;
}

TEST(MarkerPointsTest, MatchesLegacyOrderAndAtomicTriangleRejection) {
  IndexedMesh mesh;
  mesh.vertices = {
      Eigen::Vector3f(0.25f, -1.0f, 2.0f),
      Eigen::Vector3f(3.0f, 0.5f, -0.75f),
      Eigen::Vector3f(-2.0f, 1.25f, 0.125f),
      Eigen::Vector3f(0.0f, 0.0f, 4.0f),
      Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), 1.0f, 2.0f)};
  mesh.triangles = {
      Eigen::Vector3i(0, 1, 2), Eigen::Vector3i(2, 1, 3),
      Eigen::Vector3i(0, 4, 2), Eigen::Vector3i(-1, 0, 1),
      Eigen::Vector3i(0, 1, 99), Eigen::Vector3i(3, 2, 0)};

  const Eigen::Matrix4f output_from_mesh =
      (Eigen::Translation3f(1.25f, -0.75f, 2.5f) *
       Eigen::AngleAxisf(0.37f, Eigen::Vector3f(0.2f, 0.7f, -0.4f).normalized()))
          .matrix();

  std::size_t legacy_rejected = 0U;
  const std::vector<geometry_msgs::Point> legacy = legacyTriangleListPoints(
      mesh, output_from_mesh, legacy_rejected);
  const TriangleListPointResult optimized =
      buildTriangleListPoints(mesh, output_from_mesh);

  ASSERT_EQ(optimized.rejected_triangles, legacy_rejected);
  ASSERT_EQ(optimized.rejected_triangles, 3U);
  ASSERT_EQ(optimized.points.size(), legacy.size());
  ASSERT_EQ(optimized.points.size(), 9U);
  for (std::size_t index = 0; index < legacy.size(); ++index) {
    EXPECT_DOUBLE_EQ(optimized.points[index].x, legacy[index].x);
    EXPECT_DOUBLE_EQ(optimized.points[index].y, legacy[index].y);
    EXPECT_DOUBLE_EQ(optimized.points[index].z, legacy[index].z);
  }
}

}  // namespace
}  // namespace local_tsdf_mesh

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
