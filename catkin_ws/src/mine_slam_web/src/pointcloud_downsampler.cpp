#include "mine_slam_web/pointcloud_downsampler.h"

#include <cmath>

namespace mine_slam_web {
namespace {

std::uint64_t splitmix64(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30U)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27U)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31U);
}

}  // namespace

VoxelLimiter::VoxelLimiter(double voxel_size_m, std::size_t max_points)
    : voxel_size_m_(voxel_size_m), max_points_(max_points) {
  if (max_points_ > 0) {
    occupied_.reserve(max_points_);
  }
}

bool VoxelLimiter::accept(float x, float y, float z) {
  if (max_points_ > 0 && accepted_count_ >= max_points_) {
    return false;
  }

  if (voxel_size_m_ <= 0.0) {
    ++accepted_count_;
    return true;
  }

  const std::uint64_t ix = static_cast<std::uint64_t>(voxelIndex(x));
  const std::uint64_t iy = static_cast<std::uint64_t>(voxelIndex(y));
  const std::uint64_t iz = static_cast<std::uint64_t>(voxelIndex(z));
  const std::uint64_t key = splitmix64(ix) ^ (splitmix64(iy) << 1U) ^ (splitmix64(iz) << 2U);
  const auto inserted = occupied_.insert(key).second;
  if (!inserted) {
    return false;
  }
  ++accepted_count_;
  return true;
}

std::size_t VoxelLimiter::acceptedCount() const {
  return accepted_count_;
}

std::int64_t VoxelLimiter::voxelIndex(float value) const {
  return static_cast<std::int64_t>(std::floor(static_cast<double>(value) / voxel_size_m_));
}

}  // namespace mine_slam_web
