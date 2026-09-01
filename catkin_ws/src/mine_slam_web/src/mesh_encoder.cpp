#include "mine_slam_web/mesh_encoder.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace mine_slam_web {
namespace {

constexpr std::size_t kPositionBytesPerVertex = sizeof(float) * 3U;
constexpr std::size_t kColorBytesPerVertex = sizeof(std::uint8_t) * 4U;

std::uint64_t stampToNs(const ros::Time& stamp) {
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL +
         static_cast<std::uint64_t>(stamp.nsec);
}

bool checkedAdd(std::size_t left, std::size_t right, std::size_t& result) {
  if (right > std::numeric_limits<std::size_t>::max() - left) {
    return false;
  }
  result = left + right;
  return true;
}

bool checkedMultiply(std::size_t left,
                     std::size_t right,
                     std::size_t& result) {
  if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) {
    return false;
  }
  result = left * right;
  return true;
}

bool alignToFour(std::size_t value, std::size_t& result) {
  std::size_t padded = 0;
  if (!checkedAdd(value, 3U, padded)) {
    return false;
  }
  result = padded & ~std::size_t{3U};
  return true;
}

std::uint8_t colorByte(float value) {
  if (!std::isfinite(value)) {
    return 0;
  }
  const float clamped = std::max(0.0f, std::min(1.0f, value));
  return static_cast<std::uint8_t>(std::lround(clamped * 255.0f));
}

bool isFinite(double value) {
  return std::isfinite(value);
}

struct Quaternion {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};

bool normalizedQuaternion(const geometry_msgs::Quaternion& source,
                          Quaternion& result) {
  if (!isFinite(source.x) || !isFinite(source.y) || !isFinite(source.z) ||
      !isFinite(source.w)) {
    return false;
  }
  const double norm = std::sqrt(source.x * source.x + source.y * source.y +
                                source.z * source.z + source.w * source.w);
  if (norm <= 1e-12) {
    return false;
  }
  result.x = source.x / norm;
  result.y = source.y / norm;
  result.z = source.z / norm;
  result.w = source.w / norm;
  return true;
}

void rotatePoint(const Quaternion& q,
                 double x,
                 double y,
                 double z,
                 double& out_x,
                 double& out_y,
                 double& out_z) {
  const double xx = q.x * q.x;
  const double yy = q.y * q.y;
  const double zz = q.z * q.z;
  const double xy = q.x * q.y;
  const double xz = q.x * q.z;
  const double yz = q.y * q.z;
  const double wx = q.w * q.x;
  const double wy = q.w * q.y;
  const double wz = q.w * q.z;
  out_x = (1.0 - 2.0 * (yy + zz)) * x + 2.0 * (xy - wz) * y +
          2.0 * (xz + wy) * z;
  out_y = 2.0 * (xy + wz) * x + (1.0 - 2.0 * (xx + zz)) * y +
          2.0 * (yz - wx) * z;
  out_z = 2.0 * (xz - wy) * x + 2.0 * (yz + wx) * y +
          (1.0 - 2.0 * (xx + yy)) * z;
}

bool fitsU32(std::size_t value) {
  return value <= std::numeric_limits<std::uint32_t>::max();
}

void writeFloat(std::vector<std::uint8_t>& buffer,
                std::size_t offset,
                float value) {
  std::memcpy(buffer.data() + offset, &value, sizeof(value));
}

}  // namespace

MeshEncodeResult encodeTriangleListMarker(
    const visualization_msgs::Marker& marker,
    std::uint64_t revision,
    const MeshEncodeOptions& options) {
  MeshEncodeResult result;
  result.frame_id = marker.header.frame_id;

  if (marker.type != visualization_msgs::Marker::TRIANGLE_LIST) {
    result.error = "marker is not TRIANGLE_LIST";
    return result;
  }
  if (marker.action != visualization_msgs::Marker::ADD) {
    result.error = "marker action is not ADD/MODIFY";
    return result;
  }
  if (marker.points.empty()) {
    result.error = "triangle list is empty";
    return result;
  }
  if (marker.points.size() % 3U != 0U) {
    result.error = "triangle list point count is not divisible by three";
    return result;
  }
  if (marker.points.size() > options.max_vertices ||
      !fitsU32(marker.points.size())) {
    result.error = "triangle list exceeds vertex limit";
    return result;
  }
  if (!fitsU32(marker.header.frame_id.size())) {
    result.error = "mesh frame id is too long";
    return result;
  }
  if (!marker.colors.empty() && marker.colors.size() != marker.points.size()) {
    result.error = "marker colors must be empty or match the point count";
    return result;
  }
  if (!isFinite(marker.scale.x) || !isFinite(marker.scale.y) ||
      !isFinite(marker.scale.z) || !isFinite(marker.pose.position.x) ||
      !isFinite(marker.pose.position.y) || !isFinite(marker.pose.position.z)) {
    result.error = "marker pose or scale is non-finite";
    return result;
  }

  Quaternion rotation;
  if (!normalizedQuaternion(marker.pose.orientation, rotation)) {
    result.error = "marker orientation is invalid";
    return result;
  }

  std::size_t frame_end = 0;
  std::size_t positions_offset = 0;
  std::size_t position_bytes = 0;
  std::size_t colors_offset = 0;
  std::size_t color_bytes = 0;
  std::size_t packet_bytes = 0;
  if (!checkedAdd(sizeof(MeshPacketHeader), marker.header.frame_id.size(),
                  frame_end) ||
      !alignToFour(frame_end, positions_offset) ||
      !checkedMultiply(marker.points.size(), kPositionBytesPerVertex,
                       position_bytes) ||
      !checkedAdd(positions_offset, position_bytes, colors_offset) ||
      !checkedMultiply(marker.points.size(), kColorBytesPerVertex,
                       color_bytes) ||
      !checkedAdd(colors_offset, color_bytes, packet_bytes) ||
      !fitsU32(positions_offset) || !fitsU32(colors_offset) ||
      !fitsU32(packet_bytes)) {
    result.error = "mesh packet size overflow";
    return result;
  }
  if (options.max_packet_bytes > 0 && packet_bytes > options.max_packet_bytes) {
    result.error = "mesh packet exceeds byte limit";
    return result;
  }

  result.buffer.assign(packet_bytes, 0);
  MeshPacketHeader header{};
  header.magic = kMeshPacketMagic;
  header.version = kMeshPacketVersion;
  header.operation = MESH_REPLACE_ALL;
  header.flags = MESH_HAS_RGBA;
  header.revision = revision;
  header.stamp_ns = stampToNs(marker.header.stamp);
  header.vertex_count = static_cast<std::uint32_t>(marker.points.size());
  header.index_count = 0;
  header.frame_id_offset = sizeof(MeshPacketHeader);
  header.frame_id_bytes =
      static_cast<std::uint32_t>(marker.header.frame_id.size());
  header.positions_offset = static_cast<std::uint32_t>(positions_offset);
  header.colors_offset = static_cast<std::uint32_t>(colors_offset);
  header.indices_offset = static_cast<std::uint32_t>(packet_bytes);
  header.packet_bytes = static_cast<std::uint32_t>(packet_bytes);
  std::memcpy(result.buffer.data(), &header, sizeof(header));
  if (!marker.header.frame_id.empty()) {
    std::memcpy(result.buffer.data() + header.frame_id_offset,
                marker.header.frame_id.data(), marker.header.frame_id.size());
  }

  auto* colors = result.buffer.data() + header.colors_offset;
  for (std::size_t index = 0; index < marker.points.size(); ++index) {
    const auto& source = marker.points[index];
    if (!isFinite(source.x) || !isFinite(source.y) || !isFinite(source.z)) {
      result.buffer.clear();
      result.error = "triangle list contains a non-finite point";
      return result;
    }
    const double scaled_x = source.x * marker.scale.x;
    const double scaled_y = source.y * marker.scale.y;
    const double scaled_z = source.z * marker.scale.z;
    double rotated_x = 0.0;
    double rotated_y = 0.0;
    double rotated_z = 0.0;
    rotatePoint(rotation, scaled_x, scaled_y, scaled_z, rotated_x, rotated_y,
                rotated_z);
    const double x = rotated_x + marker.pose.position.x;
    const double y = rotated_y + marker.pose.position.y;
    const double z = rotated_z + marker.pose.position.z;
    if (!isFinite(x) || !isFinite(y) || !isFinite(z) ||
        std::abs(x) > std::numeric_limits<float>::max() ||
        std::abs(y) > std::numeric_limits<float>::max() ||
        std::abs(z) > std::numeric_limits<float>::max()) {
      result.buffer.clear();
      result.error = "transformed triangle list point is invalid";
      return result;
    }
    const std::size_t position_offset =
        static_cast<std::size_t>(header.positions_offset) +
        index * kPositionBytesPerVertex;
    writeFloat(result.buffer, position_offset, static_cast<float>(x));
    writeFloat(result.buffer, position_offset + sizeof(float),
               static_cast<float>(y));
    writeFloat(result.buffer, position_offset + sizeof(float) * 2U,
               static_cast<float>(z));

    const auto& color = marker.colors.empty() ? marker.color
                                               : marker.colors[index];
    colors[index * 4U] = colorByte(color.r);
    colors[index * 4U + 1U] = colorByte(color.g);
    colors[index * 4U + 2U] = colorByte(color.b);
    colors[index * 4U + 3U] = colorByte(color.a);
  }

  result.vertex_count = marker.points.size();
  result.triangle_count = marker.points.size() / 3U;
  return result;
}

std::vector<std::uint8_t> encodeMeshClearPacket(
    const ros::Time& stamp,
    const std::string& frame_id,
    std::uint64_t revision,
    const MeshEncodeOptions& options) {
  std::size_t frame_end = 0;
  std::size_t packet_bytes = 0;
  if (!fitsU32(frame_id.size()) ||
      !checkedAdd(sizeof(MeshPacketHeader), frame_id.size(), frame_end) ||
      !alignToFour(frame_end, packet_bytes) || !fitsU32(packet_bytes)) {
    return {};
  }
  if (options.max_packet_bytes > 0 && packet_bytes > options.max_packet_bytes) {
    return {};
  }
  std::vector<std::uint8_t> buffer(packet_bytes, 0);
  MeshPacketHeader header{};
  header.magic = kMeshPacketMagic;
  header.version = kMeshPacketVersion;
  header.operation = MESH_CLEAR;
  header.revision = revision;
  header.stamp_ns = stampToNs(stamp);
  header.frame_id_offset = sizeof(MeshPacketHeader);
  header.frame_id_bytes = static_cast<std::uint32_t>(frame_id.size());
  header.positions_offset = static_cast<std::uint32_t>(packet_bytes);
  header.colors_offset = static_cast<std::uint32_t>(packet_bytes);
  header.indices_offset = static_cast<std::uint32_t>(packet_bytes);
  header.packet_bytes = static_cast<std::uint32_t>(packet_bytes);
  std::memcpy(buffer.data(), &header, sizeof(header));
  if (!frame_id.empty()) {
    std::memcpy(buffer.data() + header.frame_id_offset, frame_id.data(),
                frame_id.size());
  }
  return buffer;
}

}  // namespace mine_slam_web
