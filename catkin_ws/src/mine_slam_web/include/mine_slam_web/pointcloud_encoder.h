#pragma once

#include <cstdint>
#include <vector>

#include <sensor_msgs/PointCloud2.h>

namespace mine_slam_web {

constexpr std::uint32_t kCloudPacketMagic = 0x4D504344;  // MPCD
constexpr std::uint16_t kCloudPacketVersion = 2;

enum CloudType : std::uint16_t {
  CLOUD_CURRENT = 1,
  CLOUD_STABLE = 2,
};

enum CloudFieldMask : std::uint32_t {
  FIELD_INTENSITY = 1u << 0,
  FIELD_LIDAR_ID = 1u << 1,
  FIELD_CLASS_ID = 1u << 2,
  FIELD_RGB = 1u << 3,  // Reserved for the legacy version-1 packet.
  FIELD_RING = 1u << 4,
  FIELD_TIME = 1u << 5,
};

#pragma pack(push, 1)
struct CloudPacketHeader {
  std::uint32_t magic;
  std::uint16_t version;
  std::uint16_t cloud_type;
  std::uint64_t stamp_ns;
  std::uint32_t point_count;
  std::uint32_t fields_mask;
};

// Version-2 point layout, 24 bytes. Ring and time are retained so the browser
// can reconstruct a scan-line mesh without running a global surface algorithm.
struct WebPoint {
  float x;
  float y;
  float z;
  float intensity;
  float time;
  std::uint16_t ring;
  std::uint8_t lidar_id;
  std::uint8_t class_id;
};
#pragma pack(pop)

static_assert(sizeof(CloudPacketHeader) == 24,
              "CloudPacketHeader layout changed unexpectedly");
static_assert(sizeof(WebPoint) == 24,
              "Version-2 WebPoint layout changed unexpectedly");

struct CloudEncodeOptions {
  CloudType cloud_type = CLOUD_CURRENT;
  std::size_t max_points = 200000;
  double voxel_size_m = 0.08;
  float reflector_intensity_threshold = 180.0f;
  bool transform_to_map = false;
  double rotation[9] = {1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0};
  double translation[3] = {0.0, 0.0, 0.0};
};

struct CloudEncodeResult {
  std::vector<std::uint8_t> buffer;
  std::size_t raw_points = 0;
  std::size_t encoded_points = 0;
  std::uint32_t fields_mask = 0;
};

CloudEncodeResult encodePointCloud2(const sensor_msgs::PointCloud2& cloud,
                                    const CloudEncodeOptions& options);

}  // namespace mine_slam_web
