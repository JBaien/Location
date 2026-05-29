#include "mine_slam_web/pointcloud_encoder.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#include "mine_slam_web/pointcloud_downsampler.h"

namespace mine_slam_web {
namespace {

const sensor_msgs::PointField* findField(const sensor_msgs::PointCloud2& cloud,
                                         const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

template <typename T>
T readRaw(const std::uint8_t* ptr) {
  T value{};
  std::memcpy(&value, ptr, sizeof(T));
  return value;
}

float readNumericAsFloat(const std::uint8_t* ptr, const sensor_msgs::PointField& field) {
  switch (field.datatype) {
    case sensor_msgs::PointField::INT8:
      return static_cast<float>(readRaw<std::int8_t>(ptr));
    case sensor_msgs::PointField::UINT8:
      return static_cast<float>(readRaw<std::uint8_t>(ptr));
    case sensor_msgs::PointField::INT16:
      return static_cast<float>(readRaw<std::int16_t>(ptr));
    case sensor_msgs::PointField::UINT16:
      return static_cast<float>(readRaw<std::uint16_t>(ptr));
    case sensor_msgs::PointField::INT32:
      return static_cast<float>(readRaw<std::int32_t>(ptr));
    case sensor_msgs::PointField::UINT32:
      return static_cast<float>(readRaw<std::uint32_t>(ptr));
    case sensor_msgs::PointField::FLOAT32:
      return readRaw<float>(ptr);
    case sensor_msgs::PointField::FLOAT64:
      return static_cast<float>(readRaw<double>(ptr));
    default:
      return 0.0f;
  }
}

std::uint8_t readNumericAsU8(const std::uint8_t* ptr, const sensor_msgs::PointField& field) {
  const float value = readNumericAsFloat(ptr, field);
  if (!std::isfinite(value)) {
    return 0;
  }
  return static_cast<std::uint8_t>(std::max(0.0f, std::min(255.0f, value)));
}

void appendBytes(std::vector<std::uint8_t>& dst, const void* src, std::size_t size) {
  const auto* begin = static_cast<const std::uint8_t*>(src);
  dst.insert(dst.end(), begin, begin + size);
}

std::uint64_t stampToNs(const ros::Time& stamp) {
  return static_cast<std::uint64_t>(stamp.sec) * 1000000000ULL + static_cast<std::uint64_t>(stamp.nsec);
}

}  // namespace

CloudEncodeResult encodePointCloud2(const sensor_msgs::PointCloud2& cloud,
                                    const CloudEncodeOptions& options) {
  CloudEncodeResult result;
  result.raw_points = static_cast<std::size_t>(cloud.width) * static_cast<std::size_t>(cloud.height);

  const auto* x_field = findField(cloud, "x");
  const auto* y_field = findField(cloud, "y");
  const auto* z_field = findField(cloud, "z");
  if (x_field == nullptr || y_field == nullptr || z_field == nullptr || cloud.point_step == 0) {
    return result;
  }

  const auto* intensity_field = findField(cloud, "intensity");
  const auto* lidar_field = findField(cloud, "lidar_id");
  if (lidar_field == nullptr) {
    lidar_field = findField(cloud, "ring");
  }

  result.fields_mask = FIELD_CLASS_ID;
  if (intensity_field != nullptr) {
    result.fields_mask |= FIELD_INTENSITY;
  }
  if (lidar_field != nullptr) {
    result.fields_mask |= FIELD_LIDAR_ID;
  }
  result.fields_mask |= FIELD_RGB;

  std::vector<WebPoint> points;
  points.reserve(std::min<std::size_t>(result.raw_points, options.max_points));
  VoxelLimiter limiter(options.voxel_size_m, options.max_points);

  const std::uint8_t base_class = options.cloud_type == CLOUD_STABLE ? 1 : 2;
  const std::size_t row_step = cloud.row_step;
  const std::size_t point_step = cloud.point_step;

  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * row_step;
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const std::size_t offset = row_offset + static_cast<std::size_t>(col) * point_step;
      if (offset + point_step > cloud.data.size()) {
        break;
      }
      const auto* ptr = cloud.data.data() + offset;
      WebPoint point{};
      point.x = readNumericAsFloat(ptr + x_field->offset, *x_field);
      point.y = readNumericAsFloat(ptr + y_field->offset, *y_field);
      point.z = readNumericAsFloat(ptr + z_field->offset, *z_field);
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (options.transform_to_map) {
        const double raw_x = point.x;
        const double raw_y = point.y;
        const double raw_z = point.z;
        point.x = static_cast<float>(options.rotation[0] * raw_x +
                                     options.rotation[1] * raw_y +
                                     options.rotation[2] * raw_z +
                                     options.translation[0]);
        point.y = static_cast<float>(options.rotation[3] * raw_x +
                                     options.rotation[4] * raw_y +
                                     options.rotation[5] * raw_z +
                                     options.translation[1]);
        point.z = static_cast<float>(options.rotation[6] * raw_x +
                                     options.rotation[7] * raw_y +
                                     options.rotation[8] * raw_z +
                                     options.translation[2]);
      }
      if (!limiter.accept(point.x, point.y, point.z)) {
        continue;
      }

      point.intensity = intensity_field != nullptr ? readNumericAsFloat(ptr + intensity_field->offset, *intensity_field) : 0.0f;
      point.lidar_id = lidar_field != nullptr ? readNumericAsU8(ptr + lidar_field->offset, *lidar_field) : 0;
      point.class_id = point.intensity >= options.reflector_intensity_threshold ? 3 : base_class;
      point.r = 0;
      point.g = 0;
      point.b = 0;
      points.push_back(point);
    }
  }

  result.encoded_points = points.size();

  CloudPacketHeader header{};
  header.magic = kCloudPacketMagic;
  header.version = kCloudPacketVersion;
  header.cloud_type = static_cast<std::uint16_t>(options.cloud_type);
  header.stamp_ns = stampToNs(cloud.header.stamp);
  header.point_count = static_cast<std::uint32_t>(std::min<std::size_t>(points.size(), std::numeric_limits<std::uint32_t>::max()));
  header.fields_mask = result.fields_mask;

  result.buffer.reserve(sizeof(CloudPacketHeader) + points.size() * sizeof(WebPoint));
  appendBytes(result.buffer, &header, sizeof(header));
  if (!points.empty()) {
    appendBytes(result.buffer, points.data(), points.size() * sizeof(WebPoint));
  }
  return result;
}

}  // namespace mine_slam_web
