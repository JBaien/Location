#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <ros/time.h>
#include <visualization_msgs/Marker.h>

namespace mine_slam_web {

constexpr std::uint32_t kMeshPacketMagic = 0x4D4D5348;  // MMSH
constexpr std::uint16_t kMeshPacketVersion = 1;

enum MeshPacketOperation : std::uint8_t {
  MESH_REPLACE_ALL = 1,
  MESH_CLEAR = 2,
};

enum MeshPacketFlags : std::uint8_t {
  MESH_HAS_RGBA = 1u << 0,
  MESH_HAS_UINT32_INDICES = 1u << 1,
};

#pragma pack(push, 1)
struct MeshPacketHeader {
  std::uint32_t magic;
  std::uint16_t version;
  std::uint8_t operation;
  std::uint8_t flags;
  std::uint64_t revision;
  std::uint64_t stamp_ns;
  std::uint32_t vertex_count;
  std::uint32_t index_count;
  std::uint32_t frame_id_offset;
  std::uint32_t frame_id_bytes;
  std::uint32_t positions_offset;
  std::uint32_t colors_offset;
  std::uint32_t indices_offset;
  std::uint32_t packet_bytes;
  std::uint32_t reserved0;
  std::uint32_t reserved1;
};
#pragma pack(pop)

static_assert(sizeof(MeshPacketHeader) == 64,
              "MeshPacketHeader layout changed unexpectedly");

struct MeshEncodeOptions {
  std::size_t max_vertices = 600000;
  std::size_t max_packet_bytes = 16U * 1024U * 1024U;
};

struct MeshEncodeResult {
  std::vector<std::uint8_t> buffer;
  std::size_t vertex_count = 0;
  std::size_t triangle_count = 0;
  std::string frame_id;
  std::string error;

  bool ok() const { return error.empty() && !buffer.empty(); }
};

MeshEncodeResult encodeTriangleListMarker(
    const visualization_msgs::Marker& marker,
    std::uint64_t revision,
    const MeshEncodeOptions& options);

std::vector<std::uint8_t> encodeMeshClearPacket(
    const ros::Time& stamp,
    const std::string& frame_id,
    std::uint64_t revision,
    const MeshEncodeOptions& options);

}  // namespace mine_slam_web
