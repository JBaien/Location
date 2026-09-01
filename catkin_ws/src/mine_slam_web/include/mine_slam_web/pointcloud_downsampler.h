#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_set>

namespace mine_slam_web {

class VoxelLimiter {
 public:
  VoxelLimiter(double voxel_size_m, std::size_t max_points);

  bool accept(float x, float y, float z, std::uint32_t group_id = 0);
  std::size_t acceptedCount() const;

 private:
  std::int64_t voxelIndex(float value) const;

  double voxel_size_m_;
  std::size_t max_points_;
  std::unordered_set<std::uint64_t> occupied_;
  std::size_t accepted_count_ = 0;
};

}  // namespace mine_slam_web
