#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>
#include <visualization_msgs/Marker.h>

#include "mine_slam_web/mesh_encoder.h"

namespace mine_slam_web {
namespace {

visualization_msgs::Marker makeTriangleMarker() {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time(12, 34);
  marker.ns = "local_tsdf_mesh";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.r = 0.25f;
  marker.color.g = 0.50f;
  marker.color.b = 0.75f;
  marker.color.a = 1.0f;
  marker.points.resize(3);
  marker.points[0].x = 1.0;
  marker.points[1].y = 1.0;
  marker.points[2].z = 1.0;
  return marker;
}

MeshPacketHeader readHeader(const std::vector<std::uint8_t>& buffer) {
  MeshPacketHeader header{};
  EXPECT_GE(buffer.size(), sizeof(header));
  if (buffer.size() >= sizeof(header)) {
    std::memcpy(&header, buffer.data(), sizeof(header));
  }
  return header;
}

float readFloat(const std::vector<std::uint8_t>& buffer, std::size_t offset) {
  float value = 0.0f;
  EXPECT_LE(offset + sizeof(value), buffer.size());
  if (offset + sizeof(value) <= buffer.size()) {
    std::memcpy(&value, buffer.data() + offset, sizeof(value));
  }
  return value;
}

TEST(MeshEncoder, EncodesAtomicTriangleListSnapshot) {
  auto marker = makeTriangleMarker();
  marker.pose.position.x = 2.0;
  const auto result =
      encodeTriangleListMarker(marker, 7, MeshEncodeOptions{});

  ASSERT_TRUE(result.ok()) << result.error;
  EXPECT_EQ(result.vertex_count, 3U);
  EXPECT_EQ(result.triangle_count, 1U);
  const auto header = readHeader(result.buffer);
  EXPECT_EQ(header.magic, kMeshPacketMagic);
  EXPECT_EQ(header.version, kMeshPacketVersion);
  EXPECT_EQ(header.operation, MESH_REPLACE_ALL);
  EXPECT_EQ(header.flags, MESH_HAS_RGBA);
  EXPECT_EQ(header.revision, 7U);
  EXPECT_EQ(header.vertex_count, 3U);
  EXPECT_EQ(header.index_count, 0U);
  EXPECT_EQ(header.packet_bytes, result.buffer.size());
  EXPECT_EQ(header.indices_offset, header.packet_bytes);
  EXPECT_FLOAT_EQ(readFloat(result.buffer, header.positions_offset), 3.0f);
  EXPECT_EQ(result.buffer[header.colors_offset], 64U);
  EXPECT_EQ(result.buffer[header.colors_offset + 1U], 128U);
  EXPECT_EQ(result.buffer[header.colors_offset + 2U], 191U);
  EXPECT_EQ(result.buffer[header.colors_offset + 3U], 255U);
}

TEST(MeshEncoder, EncodesClearPacketWithStableHeader) {
  const auto buffer =
      encodeMeshClearPacket(ros::Time(3, 5), "map", 9, MeshEncodeOptions{});
  ASSERT_FALSE(buffer.empty());
  const auto header = readHeader(buffer);
  EXPECT_EQ(header.operation, MESH_CLEAR);
  EXPECT_EQ(header.revision, 9U);
  EXPECT_EQ(header.vertex_count, 0U);
  EXPECT_EQ(header.positions_offset, buffer.size());
  EXPECT_EQ(header.colors_offset, buffer.size());
  EXPECT_EQ(header.indices_offset, buffer.size());
}

TEST(MeshEncoder, AppliesPacketLimitToClearPackets) {
  MeshEncodeOptions too_small;
  too_small.max_packet_bytes = sizeof(MeshPacketHeader) - 1U;
  EXPECT_TRUE(
      encodeMeshClearPacket(ros::Time(3, 5), "", 9, too_small).empty());

  MeshEncodeOptions exact_limit;
  exact_limit.max_packet_bytes = sizeof(MeshPacketHeader);
  EXPECT_EQ(encodeMeshClearPacket(ros::Time(3, 5), "", 9, exact_limit).size(),
            sizeof(MeshPacketHeader));

  MeshEncodeOptions frame_too_large;
  frame_too_large.max_packet_bytes = sizeof(MeshPacketHeader) + 3U;
  EXPECT_TRUE(encodeMeshClearPacket(ros::Time(3, 5), "map", 9,
                                    frame_too_large)
                  .empty());
}

TEST(MeshEncoder, RejectsIncompleteAndNonFiniteTrianglesAtomically) {
  auto incomplete = makeTriangleMarker();
  incomplete.points.pop_back();
  const auto incomplete_result =
      encodeTriangleListMarker(incomplete, 1, MeshEncodeOptions{});
  EXPECT_FALSE(incomplete_result.ok());
  EXPECT_TRUE(incomplete_result.buffer.empty());

  auto non_finite = makeTriangleMarker();
  non_finite.points[1].x = std::numeric_limits<double>::infinity();
  const auto non_finite_result =
      encodeTriangleListMarker(non_finite, 2, MeshEncodeOptions{});
  EXPECT_FALSE(non_finite_result.ok());
  EXPECT_TRUE(non_finite_result.buffer.empty());
}

TEST(MeshEncoder, RejectsConfiguredVertexAndPacketLimits) {
  const auto marker = makeTriangleMarker();
  MeshEncodeOptions vertex_limit;
  vertex_limit.max_vertices = 2;
  EXPECT_FALSE(encodeTriangleListMarker(marker, 1, vertex_limit).ok());

  MeshEncodeOptions packet_limit;
  packet_limit.max_packet_bytes = sizeof(MeshPacketHeader);
  EXPECT_FALSE(encodeTriangleListMarker(marker, 1, packet_limit).ok());
}

}  // namespace
}  // namespace mine_slam_web

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
