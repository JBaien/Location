#include "local_tsdf_mesh/mesh_cleanup.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <future>
#include <limits>
#include <new>
#include <numeric>
#include <stdexcept>
#include <system_error>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace local_tsdf_mesh {
namespace {

using TriangleKey = std::array<int, 3>;
using EdgeKey = std::uint64_t;

struct QuantizedVertexKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const QuantizedVertexKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }

  bool operator<(const QuantizedVertexKey& other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

struct QuantizedVertexKeyHash {
  std::size_t operator()(const QuantizedVertexKey& key) const {
    auto mix = [](std::size_t seed, std::uint64_t value) {
      seed ^= static_cast<std::size_t>(value) + 0x9e3779b97f4a7c15ULL +
              (seed << 6U) + (seed >> 2U);
      return seed;
    };
    std::size_t seed = 0;
    seed = mix(seed, static_cast<std::uint64_t>(key.x));
    seed = mix(seed, static_cast<std::uint64_t>(key.y));
    return mix(seed, static_cast<std::uint64_t>(key.z));
  }
};

struct QuantizedVertexCache {
  std::vector<QuantizedVertexKey> keys;
  std::vector<std::uint8_t> valid;
};

struct TriangleKeyHash {
  std::size_t operator()(const TriangleKey& key) const {
    auto mix = [](std::size_t seed, std::uint32_t value) {
      seed ^= static_cast<std::size_t>(value) + 0x9e3779b9U +
              (seed << 6U) + (seed >> 2U);
      return seed;
    };
    std::size_t seed = 0;
    seed = mix(seed, static_cast<std::uint32_t>(key[0]));
    seed = mix(seed, static_cast<std::uint32_t>(key[1]));
    return mix(seed, static_cast<std::uint32_t>(key[2]));
  }
};

struct CandidateTriangle {
  Eigen::Vector3i indices = Eigen::Vector3i::Zero();
  TriangleKey canonical{{0, 0, 0}};
  double area = 0.0;
};

struct EquivalentCandidateTriangle {
  Eigen::Vector3i output_indices = Eigen::Vector3i::Zero();
  Eigen::Vector3i topology_indices = Eigen::Vector3i::Zero();
  TriangleKey canonical{{0, 0, 0}};
};

struct ComponentSummary {
  std::size_t triangle_count = 0;
  double area = 0.0;
  Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
      -std::numeric_limits<double>::infinity());
};

struct EdgeTopology {
  std::size_t count = 0;
  std::size_t first_triangle = 0;
};

struct VertexLinkFan {
  double area = 0.0;
  std::size_t triangle_count = 0U;
  TriangleKey canonical_min{{std::numeric_limits<int>::max(),
                             std::numeric_limits<int>::max(),
                             std::numeric_limits<int>::max()}};
};

struct VertexLinkRecord {
  std::size_t triangle = 0U;
  int first = -1;
  int second = -1;
  int fan = -1;
};

struct VertexLinkWorkspace {
  explicit VertexLinkWorkspace(std::size_t vertex_count)
      : fan_by_root(vertex_count, -1), fan_stamp(vertex_count, 0U) {}

  std::vector<VertexLinkRecord> link_edges;
  std::vector<int> fan_by_root;
  std::vector<std::size_t> fan_stamp;
  std::vector<VertexLinkFan> fans;
};

struct VertexLinkConnectivityWorkspace {
  explicit VertexLinkConnectivityWorkspace(std::size_t vertex_count)
      : parent(vertex_count, 0U),
        rank(vertex_count, 0U),
        stamp(vertex_count, 0U) {}

  std::vector<std::size_t> parent;
  std::vector<std::uint8_t> rank;
  std::vector<std::size_t> stamp;
  std::vector<std::size_t> touched;
  std::size_t generation = 0U;
};

struct VertexLinkValidationWorkspace {
  explicit VertexLinkValidationWorkspace(std::size_t vertex_count)
      : parent(vertex_count, 0U),
        rank(vertex_count, 0U),
        degree(vertex_count, 0U),
        stamp(vertex_count, 0U) {}

  std::vector<std::size_t> parent;
  std::vector<std::uint8_t> rank;
  std::vector<std::uint8_t> degree;
  std::vector<std::size_t> stamp;
  std::vector<std::size_t> touched;
  std::size_t generation = 0U;
};

const Eigen::Vector3i& candidateTopology(const CandidateTriangle& candidate) {
  return candidate.indices;
}

const Eigen::Vector3i& candidateTopology(
    const EquivalentCandidateTriangle& candidate) {
  return candidate.topology_indices;
}

bool finitePoint(const Eigen::Vector3f& point) {
  return std::isfinite(point.x()) && std::isfinite(point.y()) &&
         std::isfinite(point.z());
}

bool quantizedVertexKey(const Eigen::Vector3f& point, double tolerance,
                        QuantizedVertexKey& key) {
  if (!finitePoint(point) || !(tolerance > 0.0)) {
    return false;
  }
  const double limit =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) - 1.0;
  const Eigen::Vector3d scaled = point.cast<double>() / tolerance;
  if (!std::isfinite(scaled.x()) || !std::isfinite(scaled.y()) ||
      !std::isfinite(scaled.z()) || std::fabs(scaled.x()) > limit ||
      std::fabs(scaled.y()) > limit || std::fabs(scaled.z()) > limit) {
    return false;
  }
  key.x = static_cast<std::int64_t>(std::llround(scaled.x()));
  key.y = static_cast<std::int64_t>(std::llround(scaled.y()));
  key.z = static_cast<std::int64_t>(std::llround(scaled.z()));
  return true;
}

TriangleKey canonicalTriangle(const Eigen::Vector3i& triangle) {
  TriangleKey key{{triangle.x(), triangle.y(), triangle.z()}};
  std::sort(key.begin(), key.end());
  return key;
}

TriangleKey orientedTriangle(const Eigen::Vector3i& triangle) {
  return TriangleKey{{triangle.x(), triangle.y(), triangle.z()}};
}

EdgeKey edgeKey(int first, int second) {
  const std::uint32_t low = static_cast<std::uint32_t>(std::min(first, second));
  const std::uint32_t high =
      static_cast<std::uint32_t>(std::max(first, second));
  return (static_cast<std::uint64_t>(low) << 32U) |
         static_cast<std::uint64_t>(high);
}

std::array<EdgeKey, 3> triangleEdges(const Eigen::Vector3i& triangle) {
  return {{edgeKey(triangle.x(), triangle.y()),
           edgeKey(triangle.y(), triangle.z()),
           edgeKey(triangle.z(), triangle.x())}};
}

std::array<int, 2> otherTriangleVertices(const Eigen::Vector3i& triangle,
                                         int vertex) {
  std::array<int, 2> result{{-1, -1}};
  std::size_t count = 0U;
  for (int corner = 0; corner < 3; ++corner) {
    if (triangle[corner] == vertex) {
      continue;
    }
    if (count < result.size()) {
      result[count++] = triangle[corner];
    }
  }
  return result;
}

template <typename Candidate, typename AreaOf>
const std::vector<VertexLinkFan>& vertexLinkFans(
    int vertex, const std::vector<std::size_t>& incident,
    std::size_t incident_begin, std::size_t incident_end,
    const std::vector<std::uint8_t>& active,
    const std::vector<Candidate>& candidates,
    VertexLinkWorkspace& workspace,
    const VertexLinkConnectivityWorkspace& connectivity, AreaOf& area_of) {
  workspace.link_edges.clear();
  workspace.fans.clear();
  const std::size_t incident_size = incident_end - incident_begin;
  workspace.link_edges.reserve(incident_size);
  workspace.fans.reserve(incident_size);
  const auto find_root = [&connectivity](std::size_t value) {
    while (connectivity.parent[value] != value) {
      value = connectivity.parent[value];
    }
    return value;
  };
  for (std::size_t position = incident_begin; position < incident_end;
       ++position) {
    const std::size_t triangle_index = incident[position];
    if (active[triangle_index] == 0U) {
      continue;
    }
    const std::array<int, 2> edge =
        otherTriangleVertices(candidateTopology(candidates[triangle_index]),
                              vertex);
    if (edge[0] < 0 || edge[1] < 0 || edge[0] == edge[1]) {
      continue;
    }
    const std::size_t root =
        find_root(static_cast<std::size_t>(edge[0]));
    if (workspace.fan_stamp[root] != connectivity.generation) {
      workspace.fan_stamp[root] = connectivity.generation;
      workspace.fan_by_root[root] =
          static_cast<int>(workspace.fans.size());
      workspace.fans.emplace_back();
    }
    const int fan_index = workspace.fan_by_root[root];
    workspace.link_edges.push_back(
        VertexLinkRecord{triangle_index, edge[0], edge[1], fan_index});
    VertexLinkFan& fan =
        workspace.fans[static_cast<std::size_t>(fan_index)];
    fan.area += area_of(triangle_index);
    ++fan.triangle_count;
    fan.canonical_min =
        std::min(fan.canonical_min, candidates[triangle_index].canonical);
  }
  return workspace.fans;
}

template <typename Candidate>
bool hasMultipleVertexLinkFans(
    int vertex, const std::vector<std::size_t>& incident,
    std::size_t incident_begin, std::size_t incident_end,
    const std::vector<std::uint8_t>& active,
    const std::vector<Candidate>& candidates,
    VertexLinkConnectivityWorkspace& workspace) {
  ++workspace.generation;
  if (workspace.generation == 0U) {
    std::fill(workspace.stamp.begin(), workspace.stamp.end(), 0U);
    workspace.generation = 1U;
  }
  workspace.touched.clear();
  const auto touch = [&workspace](std::size_t value) {
    if (workspace.stamp[value] == workspace.generation) {
      return;
    }
    workspace.stamp[value] = workspace.generation;
    workspace.parent[value] = value;
    workspace.rank[value] = 0U;
    workspace.touched.push_back(value);
  };
  const auto find_root = [&workspace](std::size_t value) {
    std::size_t root = value;
    while (workspace.parent[root] != root) {
      root = workspace.parent[root];
    }
    while (workspace.parent[value] != value) {
      const std::size_t next = workspace.parent[value];
      workspace.parent[value] = root;
      value = next;
    }
    return root;
  };
  const auto unite = [&workspace, &find_root](std::size_t first,
                                              std::size_t second) {
    first = find_root(first);
    second = find_root(second);
    if (first == second) {
      return;
    }
    if (workspace.rank[first] < workspace.rank[second]) {
      std::swap(first, second);
    }
    workspace.parent[second] = first;
    if (workspace.rank[first] == workspace.rank[second]) {
      ++workspace.rank[first];
    }
  };
  for (std::size_t position = incident_begin; position < incident_end;
       ++position) {
    const std::size_t triangle_index = incident[position];
    if (active[triangle_index] == 0U) {
      continue;
    }
    const std::array<int, 2> edge = otherTriangleVertices(
        candidateTopology(candidates[triangle_index]), vertex);
    if (edge[0] < 0 || edge[1] < 0 || edge[0] == edge[1]) {
      continue;
    }
    const std::size_t first = static_cast<std::size_t>(edge[0]);
    const std::size_t second = static_cast<std::size_t>(edge[1]);
    touch(first);
    touch(second);
    unite(first, second);
  }
  if (workspace.touched.empty()) {
    return false;
  }
  const std::size_t first_root = find_root(workspace.touched.front());
  return std::any_of(
      workspace.touched.begin() + 1U, workspace.touched.end(),
      [&find_root, first_root](std::size_t value) {
        return find_root(value) != first_root;
      });
}

bool validVertexLink(
    const std::vector<std::array<int, 2>>& links, std::size_t link_begin,
    std::size_t link_end, VertexLinkValidationWorkspace& workspace,
    std::size_t& boundary_neighbor_count,
    std::size_t& nonmanifold_neighbor_count) {
  if (link_begin == link_end) {
    return true;
  }
  ++workspace.generation;
  if (workspace.generation == 0U) {
    std::fill(workspace.stamp.begin(), workspace.stamp.end(), 0U);
    workspace.generation = 1U;
  }
  workspace.touched.clear();
  const auto touch = [&workspace](std::size_t value) {
    if (workspace.stamp[value] == workspace.generation) {
      return;
    }
    workspace.stamp[value] = workspace.generation;
    workspace.parent[value] = value;
    workspace.rank[value] = 0U;
    workspace.degree[value] = 0U;
    workspace.touched.push_back(value);
  };
  const auto find_root = [&workspace](std::size_t value) {
    std::size_t root = value;
    while (workspace.parent[root] != root) {
      root = workspace.parent[root];
    }
    while (workspace.parent[value] != value) {
      const std::size_t next = workspace.parent[value];
      workspace.parent[value] = root;
      value = next;
    }
    return root;
  };
  const auto unite = [&workspace, &find_root](std::size_t first,
                                              std::size_t second) {
    first = find_root(first);
    second = find_root(second);
    if (first == second) {
      return;
    }
    if (workspace.rank[first] < workspace.rank[second]) {
      std::swap(first, second);
    }
    workspace.parent[second] = first;
    if (workspace.rank[first] == workspace.rank[second]) {
      ++workspace.rank[first];
    }
  };
  // `links` is built only from canonical, non-degenerate, unique faces. Two
  // identical link edges at this center would therefore imply two faces with
  // the same canonical triangle key, which the preceding face map has already
  // removed. Direct global vertex ids are consequently a complete link graph;
  // no per-vertex edge sort or neighbor-to-local-index map is required.
  for (std::size_t index = link_begin; index < link_end; ++index) {
    const int raw_first = links[index][0];
    const int raw_second = links[index][1];
    if (raw_first < 0 || raw_second < 0 || raw_first == raw_second) {
      return false;
    }
    const std::size_t first = static_cast<std::size_t>(raw_first);
    const std::size_t second = static_cast<std::size_t>(raw_second);
    touch(first);
    touch(second);
    workspace.degree[first] = static_cast<std::uint8_t>(
        std::min<std::uint8_t>(3U, workspace.degree[first] + 1U));
    workspace.degree[second] = static_cast<std::uint8_t>(
        std::min<std::uint8_t>(3U, workspace.degree[second] + 1U));
    unite(first, second);
  }
  std::size_t link_components = 0U;
  std::size_t degree_one = 0U;
  std::size_t degree_above_two = 0U;
  for (const std::size_t neighbor : workspace.touched) {
    if (find_root(neighbor) == neighbor) {
      ++link_components;
    }
    if (workspace.degree[neighbor] == 1U) {
      ++degree_one;
    } else if (workspace.degree[neighbor] > 2U) {
      ++degree_above_two;
    }
  }
  boundary_neighbor_count += degree_one;
  nonmanifold_neighbor_count += degree_above_two;
  if (link_components != 1U || degree_above_two != 0U ||
      (degree_one != 0U && degree_one != 2U)) {
    return false;
  }
  return degree_one == 2U ||
         std::all_of(workspace.touched.begin(), workspace.touched.end(),
                     [&workspace](std::size_t neighbor) {
                       return workspace.degree[neighbor] == 2U;
                     });
}

bool betterVertexLinkFan(const VertexLinkFan& left,
                         const VertexLinkFan& right) {
  if (left.area != right.area) {
    return left.area > right.area;
  }
  if (left.triangle_count != right.triangle_count) {
    return left.triangle_count > right.triangle_count;
  }
  return left.canonical_min < right.canonical_min;
}

template <typename Candidate, typename AreaOf>
void repairNonmanifoldVertexLinks(
    std::size_t vertex_count, const std::vector<Candidate>& candidates,
    std::vector<std::size_t>& accepted_indices,
    std::size_t& rejected_triangles, std::size_t& vertices_before,
    std::size_t& vertices_after, AreaOf area_of,
    const std::vector<std::size_t>* initial_vertices = nullptr,
    std::vector<std::size_t>* removed_triangles = nullptr) {
  std::vector<std::uint8_t> active(candidates.size(), 0U);
  std::vector<std::size_t> incident_offsets(vertex_count + 1U, 0U);
  VertexLinkWorkspace workspace(vertex_count);
  VertexLinkConnectivityWorkspace connectivity(vertex_count);
  for (const std::size_t index : accepted_indices) {
    active[index] = 1U;
    for (int corner = 0; corner < 3; ++corner) {
      const std::size_t vertex = static_cast<std::size_t>(
          candidateTopology(candidates[index])[corner]);
      ++incident_offsets[vertex + 1U];
    }
  }
  for (std::size_t vertex = 0U; vertex < vertex_count; ++vertex) {
    incident_offsets[vertex + 1U] += incident_offsets[vertex];
  }
  std::vector<std::size_t> incident(incident_offsets.back());
  std::vector<std::size_t> incident_cursors(
      incident_offsets.begin(), incident_offsets.end() - 1U);
  for (const std::size_t index : accepted_indices) {
    for (int corner = 0; corner < 3; ++corner) {
      const std::size_t vertex = static_cast<std::size_t>(
          candidateTopology(candidates[index])[corner]);
      incident[incident_cursors[vertex]++] = index;
    }
  }
  std::deque<std::size_t> pending;
  std::vector<std::uint8_t> queued(vertex_count, 0U);
  const auto enqueue = [&pending, &queued](std::size_t vertex) {
    if (queued[vertex] == 0U) {
      queued[vertex] = 1U;
      pending.push_back(vertex);
    }
  };
  const auto inspect_initial_vertex = [&](std::size_t vertex) {
    if (vertex >= vertex_count) {
      return;
    }
    if (hasMultipleVertexLinkFans(
            static_cast<int>(vertex), incident, incident_offsets[vertex],
            incident_offsets[vertex + 1U], active, candidates,
            connectivity)) {
      ++vertices_before;
      enqueue(vertex);
    }
  };
  if (initial_vertices == nullptr) {
    for (std::size_t vertex = 0; vertex < vertex_count; ++vertex) {
      inspect_initial_vertex(vertex);
    }
  } else {
    for (const std::size_t vertex : *initial_vertices) {
      inspect_initial_vertex(vertex);
    }
  }
  if (vertices_before == 0U) {
    vertices_after = 0U;
    return;
  }

  // Each conflict resolution removes at least one active triangle. Removed
  // triangles never return, so processing is bounded even though neighboring
  // vertices are revisited when their links change.
  while (!pending.empty()) {
    const std::size_t vertex = pending.front();
    pending.pop_front();
    queued[vertex] = 0U;
    if (!hasMultipleVertexLinkFans(
            static_cast<int>(vertex), incident, incident_offsets[vertex],
            incident_offsets[vertex + 1U], active, candidates,
            connectivity)) {
      continue;
    }
    const std::vector<VertexLinkFan>& fans = vertexLinkFans(
        static_cast<int>(vertex), incident, incident_offsets[vertex],
        incident_offsets[vertex + 1U], active, candidates, workspace,
        connectivity, area_of);
    if (fans.size() <= 1U) {
      continue;
    }
    std::size_t retained = 0U;
    for (std::size_t fan = 1U; fan < fans.size(); ++fan) {
      if (betterVertexLinkFan(fans[fan], fans[retained])) {
        retained = fan;
      }
    }
    for (const VertexLinkRecord& edge : workspace.link_edges) {
      if (edge.fan < 0 || static_cast<std::size_t>(edge.fan) == retained) {
        continue;
      }
      const std::size_t triangle_index = edge.triangle;
      if (active[triangle_index] == 0U) {
        continue;
      }
      active[triangle_index] = 0U;
      if (removed_triangles != nullptr) {
        removed_triangles->push_back(triangle_index);
      }
      ++rejected_triangles;
      for (int corner = 0; corner < 3; ++corner) {
        enqueue(static_cast<std::size_t>(
            candidateTopology(candidates[triangle_index])[corner]));
      }
    }
  }

  accepted_indices.erase(
      std::remove_if(accepted_indices.begin(), accepted_indices.end(),
                     [&active](std::size_t index) {
                       return active[index] == 0U;
                     }),
      accepted_indices.end());
  // Every initially invalid link is queued. Removing a triangle can only
  // change the links at its three corners, and those corners are queued before
  // processing continues. Therefore an empty queue is a complete fixed point:
  // vertices outside the queue never changed, while every changed vertex was
  // rechecked after its final incident-face removal.
  vertices_after = 0U;
}

class DisjointSet {
 public:
  explicit DisjointSet(std::size_t size) { reset(size); }

  void reset(std::size_t size) {
    parent_.resize(size);
    rank_.assign(size, 0U);
    for (std::size_t index = 0; index < size; ++index) {
      parent_[index] = index;
    }
  }

  std::size_t find(std::size_t value) {
    std::size_t root = value;
    while (parent_[root] != root) {
      root = parent_[root];
    }
    while (parent_[value] != value) {
      const std::size_t next = parent_[value];
      parent_[value] = root;
      value = next;
    }
    return root;
  }

  void unite(std::size_t left, std::size_t right) {
    left = find(left);
    right = find(right);
    if (left == right) {
      return;
    }
    if (rank_[left] < rank_[right]) {
      std::swap(left, right);
    }
    parent_[right] = left;
    if (rank_[left] == rank_[right]) {
      ++rank_[left];
    }
  }

 private:
  std::vector<std::size_t> parent_;
  std::vector<std::uint8_t> rank_;
};

void validateConfig(const MeshCleanupConfig& config) {
  const bool valid = std::isfinite(config.minimum_triangle_area) &&
                     std::isfinite(config.maximum_triangle_area) &&
                     std::isfinite(config.maximum_edge_length) &&
                     std::isfinite(config.vertex_merge_tolerance) &&
                     std::isfinite(config.maximum_small_component_area) &&
                     std::isfinite(config.maximum_small_component_extent) &&
                     config.minimum_triangle_area >= 0.0 &&
                     config.maximum_triangle_area >
                         config.minimum_triangle_area &&
                     config.maximum_edge_length > 0.0 &&
                     config.vertex_merge_tolerance >= 0.0 &&
                     config.minimum_component_triangles > 0U &&
                     config.maximum_small_component_area >= 0.0 &&
                     config.maximum_small_component_extent >= 0.0;
  if (!valid) {
    throw std::invalid_argument("invalid mesh cleanup configuration");
  }
}

bool meshesShareEquivalentVertex(
    const IndexedMesh& first, const IndexedMesh& second,
    double vertex_equivalence_tolerance) {
  if (!(vertex_equivalence_tolerance > 0.0) || first.vertices.empty() ||
      second.vertices.empty()) {
    return false;
  }
  const IndexedMesh* indexed = &first;
  const IndexedMesh* probed = &second;
  if (indexed->vertices.size() > probed->vertices.size()) {
    std::swap(indexed, probed);
  }
  std::unordered_set<QuantizedVertexKey, QuantizedVertexKeyHash> keys;
  keys.reserve(indexed->vertices.size());
  for (const Eigen::Vector3f& vertex : indexed->vertices) {
    QuantizedVertexKey key;
    if (quantizedVertexKey(vertex, vertex_equivalence_tolerance, key)) {
      keys.insert(key);
    }
  }
  for (const Eigen::Vector3f& vertex : probed->vertices) {
    QuantizedVertexKey key;
    if (quantizedVertexKey(vertex, vertex_equivalence_tolerance, key) &&
        keys.count(key) != 0U) {
      return true;
    }
  }
  return false;
}

bool quantizedCachesShareVertex(const QuantizedVertexCache& first,
                                const QuantizedVertexCache& second) {
  const QuantizedVertexCache* indexed = &first;
  const QuantizedVertexCache* probed = &second;
  if (indexed->keys.size() > probed->keys.size()) {
    std::swap(indexed, probed);
  }
  std::unordered_set<QuantizedVertexKey, QuantizedVertexKeyHash> keys;
  keys.reserve(indexed->keys.size());
  for (std::size_t index = 0U; index < indexed->keys.size(); ++index) {
    if (indexed->valid[index] != 0U) {
      keys.insert(indexed->keys[index]);
    }
  }
  for (std::size_t index = 0U; index < probed->keys.size(); ++index) {
    if (probed->valid[index] != 0U &&
        keys.count(probed->keys[index]) != 0U) {
      return true;
    }
  }
  return false;
}

bool directSupportValidationContract(const MeshCleanupConfig& config) {
  return config.vertex_merge_tolerance == 0.0 &&
         config.minimum_component_triangles == 1U &&
         config.maximum_small_component_area == 0.0 &&
         config.maximum_small_component_extent == 0.0;
}

struct SupportGeometryValidation {
  SupportGeometryValidation(std::size_t vertex_count,
                            const MeshCleanupConfig& geometry_config)
      : config(geometry_config), vertex_used(vertex_count, 0U) {}

  const MeshCleanupConfig& config;
  std::vector<std::uint8_t> vertex_used;
  std::size_t triangles = 0U;
  std::size_t degenerate = 0U;
  std::size_t large_faces = 0U;
};

MeshTopologyValidationResult validateMeshTopologyImpl(
    const IndexedMesh& mesh, double vertex_equivalence_tolerance,
    SupportGeometryValidation* geometry,
    QuantizedVertexCache* quantized_vertices);

bool validateSupportedStripWithoutMutation(
    const IndexedMesh& mesh, const MeshCleanupConfig& config,
    double vertex_equivalence_tolerance, MeshCleanupStats& stats,
    MeshTopologyValidationResult& topology,
    QuantizedVertexCache& quantized_vertices) {
  validateConfig(config);
  stats = MeshCleanupStats();
  stats.input_vertices = mesh.vertices.size();
  stats.input_triangles = mesh.triangles.size();
  SupportGeometryValidation geometry(mesh.vertices.size(), config);
  topology = validateMeshTopologyImpl(mesh, vertex_equivalence_tolerance,
                                      &geometry, &quantized_vertices);
  stats.rejected_invalid_index = topology.invalid_indices;
  stats.rejected_nonfinite = topology.nonfinite_triangles;
  stats.rejected_duplicate = topology.duplicate_triangles;
  stats.rejected_nonmanifold = topology.nonmanifold_edges;
  stats.rejected_nonmanifold_vertex = topology.nonmanifold_vertices;
  stats.boundary_edges_before_cleanup = topology.boundary_edges;
  stats.boundary_edges_after_cleanup = topology.boundary_edges;
  stats.nonmanifold_edges_before_cleanup = topology.nonmanifold_edges;
  stats.nonmanifold_edges_after_cleanup = topology.nonmanifold_edges;
  stats.nonmanifold_vertices_before_cleanup = topology.nonmanifold_vertices;
  stats.nonmanifold_vertices_after_cleanup = topology.nonmanifold_vertices;

  stats.rejected_large_face = geometry.large_faces;
  stats.rejected_degenerate =
      std::max(topology.degenerate_triangles, geometry.degenerate);
  stats.output_triangles = geometry.triangles;
  stats.output_vertices = static_cast<std::size_t>(
      std::count(geometry.vertex_used.begin(), geometry.vertex_used.end(),
                 1U));
  const bool unchanged =
      topology.valid && stats.rejectedTotal() == 0U &&
      stats.output_vertices == stats.input_vertices &&
      stats.output_triangles == stats.input_triangles;
  return unchanged;
}

struct SupportValidationWork {
  bool unchanged = false;
  MeshCleanupStats stats;
  MeshTopologyValidationResult topology;
  QuantizedVertexCache quantized_vertices;
  double elapsed_ms = 0.0;
};

SupportValidationWork validateSupportReadOnly(
    const IndexedMesh& mesh, const MeshCleanupConfig& config,
    double vertex_equivalence_tolerance) {
  using Clock = std::chrono::steady_clock;
  const Clock::time_point start = Clock::now();
  SupportValidationWork work;
  if (directSupportValidationContract(config)) {
    work.unchanged = validateSupportedStripWithoutMutation(
        mesh, config, vertex_equivalence_tolerance, work.stats,
        work.topology, work.quantized_vertices);
  } else {
    MeshCleanupResult checked = cleanupMeshTopology(mesh, config);
    work.stats = checked.stats;
    work.unchanged = checked.stats.rejectedTotal() == 0U &&
                     checked.stats.merged_vertices == 0U &&
                     checked.mesh.vertices.size() == mesh.vertices.size() &&
                     checked.mesh.triangles.size() == mesh.triangles.size();
    work.topology =
        validateMeshTopology(mesh, vertex_equivalence_tolerance);
  }
  work.elapsed_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  return work;
}

class SupportValidationFutureGuard {
 public:
  explicit SupportValidationFutureGuard(
      std::future<SupportValidationWork>& future)
      : future_(future) {}

  ~SupportValidationFutureGuard() {
    if (future_.valid()) {
      future_.wait();
    }
  }

 private:
  std::future<SupportValidationWork>& future_;
};

std::array<float, 9> sortedTriangleGeometryKey(
    const IndexedMesh& mesh, const Eigen::Vector3i& triangle) {
  std::array<std::array<float, 3>, 3> points;
  for (int corner = 0; corner < 3; ++corner) {
    const Eigen::Vector3f& point =
        mesh.vertices[static_cast<std::size_t>(triangle[corner])];
    points[static_cast<std::size_t>(corner)] =
        {{point.x(), point.y(), point.z()}};
  }
  std::sort(points.begin(), points.end());
  std::array<float, 9> result;
  for (std::size_t point = 0U; point < points.size(); ++point) {
    for (std::size_t axis = 0U; axis < points[point].size(); ++axis) {
      result[point * 3U + axis] = points[point][axis];
    }
  }
  return result;
}

std::array<float, 9> orientedTriangleGeometryKey(
    const IndexedMesh& mesh, const Eigen::Vector3i& triangle) {
  std::array<float, 9> best;
  bool have_best = false;
  for (int rotation = 0; rotation < 3; ++rotation) {
    std::array<float, 9> candidate;
    for (int corner = 0; corner < 3; ++corner) {
      const Eigen::Vector3f& point = mesh.vertices[static_cast<std::size_t>(
          triangle[(corner + rotation) % 3])];
      candidate[static_cast<std::size_t>(corner) * 3U] = point.x();
      candidate[static_cast<std::size_t>(corner) * 3U + 1U] = point.y();
      candidate[static_cast<std::size_t>(corner) * 3U + 2U] = point.z();
    }
    if (!have_best || candidate < best) {
      best = candidate;
      have_best = true;
    }
  }
  return best;
}

bool equivalentCandidateLess(const IndexedMesh& mesh,
                             const EquivalentCandidateTriangle& left,
                             const EquivalentCandidateTriangle& right) {
  if (left.canonical != right.canonical) {
    return left.canonical < right.canonical;
  }
  // Canonical topology keys are unique after duplicate collapse, so the
  // geometry tie-breakers are needed only on the exceptional duplicate path.
  // Computing them lazily avoids storing and populating 18 floats for every
  // ordinary marching-tetra triangle.
  const std::array<float, 9> left_sorted =
      sortedTriangleGeometryKey(mesh, left.output_indices);
  const std::array<float, 9> right_sorted =
      sortedTriangleGeometryKey(mesh, right.output_indices);
  if (left_sorted != right_sorted) {
    return left_sorted < right_sorted;
  }
  const std::array<float, 9> left_oriented =
      orientedTriangleGeometryKey(mesh, left.output_indices);
  const std::array<float, 9> right_oriented =
      orientedTriangleGeometryKey(mesh, right.output_indices);
  if (left_oriented != right_oriented) {
    return left_oriented < right_oriented;
  }
  return orientedTriangle(left.output_indices) <
         orientedTriangle(right.output_indices);
}

struct EquivalentTopologyRepairResult {
  IndexedMesh mesh;
  MeshTopologyValidationResult topology;
  QuantizedVertexCache quantized_vertices;
};

EquivalentTopologyRepairResult repairEquivalentBaseTopology(
    IndexedMesh input, double vertex_equivalence_tolerance,
    MeshCleanupStats& stats) {
  if (!std::isfinite(vertex_equivalence_tolerance) ||
      vertex_equivalence_tolerance <= 0.0) {
    throw std::invalid_argument(
        "equivalent topology tolerance must be finite and positive");
  }
  EquivalentTopologyRepairResult result;
  result.mesh.triangle_limit_reached = input.triangle_limit_reached;

  const std::size_t no_provisional_vertex =
      std::numeric_limits<std::size_t>::max();
  std::vector<std::size_t> provisional_vertex(
      input.vertices.size(), no_provisional_vertex);
  std::vector<QuantizedVertexKey> unique_keys;
  unique_keys.reserve(input.vertices.size());
  std::unordered_map<QuantizedVertexKey, std::size_t,
                     QuantizedVertexKeyHash>
      provisional_id_by_key;
  provisional_id_by_key.reserve(input.vertices.size());
  bool has_equivalent_vertices = false;
  std::size_t unquantized_vertices = 0U;
  for (std::size_t index = 0U; index < input.vertices.size(); ++index) {
    QuantizedVertexKey key;
    if (!quantizedVertexKey(input.vertices[index],
                            vertex_equivalence_tolerance, key)) {
      ++unquantized_vertices;
      continue;
    }
    const auto inserted =
        provisional_id_by_key.emplace(key, unique_keys.size());
    if (inserted.second) {
      unique_keys.push_back(key);
    } else {
      has_equivalent_vertices = true;
    }
    provisional_vertex[index] = inserted.first->second;
  }
  if (!has_equivalent_vertices) {
    result.mesh = std::move(input);
    result.quantized_vertices.keys.resize(result.mesh.vertices.size());
    result.quantized_vertices.valid.assign(result.mesh.vertices.size(), 0U);
    for (std::size_t index = 0U; index < provisional_vertex.size(); ++index) {
      if (provisional_vertex[index] == no_provisional_vertex) {
        continue;
      }
      result.quantized_vertices.keys[index] =
          unique_keys[provisional_vertex[index]];
      result.quantized_vertices.valid[index] = 1U;
    }
    result.topology.valid = true;
    result.topology.reason = "accepted";
    result.topology.boundary_edges = stats.boundary_edges_after_cleanup;
    return result;
  }

  std::vector<std::size_t> sorted_provisional(unique_keys.size());
  std::iota(sorted_provisional.begin(), sorted_provisional.end(), 0U);
  std::sort(sorted_provisional.begin(), sorted_provisional.end(),
            [&unique_keys](std::size_t left, std::size_t right) {
              return unique_keys[left] < unique_keys[right];
            });
  std::vector<int> sorted_id_by_provisional(unique_keys.size(), -1);
  for (std::size_t index = 0U; index < sorted_provisional.size(); ++index) {
    sorted_id_by_provisional[sorted_provisional[index]] =
        static_cast<int>(index);
  }
  const std::size_t topology_vertex_count =
      unique_keys.size() + unquantized_vertices;
  if (topology_vertex_count >
      static_cast<std::size_t>(std::numeric_limits<int>::max())) {
    result.topology.reason = "invalid_index";
    result.topology.invalid_indices = 1U;
    return result;
  }
  std::vector<int> topology_vertex(input.vertices.size(), -1);
  std::vector<std::size_t> topology_source_count(topology_vertex_count, 0U);
  std::size_t next_unquantized = unique_keys.size();
  for (std::size_t index = 0U; index < input.vertices.size(); ++index) {
    if (provisional_vertex[index] == no_provisional_vertex) {
      topology_vertex[index] = static_cast<int>(next_unquantized++);
      ++topology_source_count[static_cast<std::size_t>(
          topology_vertex[index])];
      continue;
    }
    if (provisional_vertex[index] >= sorted_id_by_provisional.size()) {
      result.topology.reason = "invalid_index";
      result.topology.invalid_indices = 1U;
      return result;
    }
    topology_vertex[index] =
        sorted_id_by_provisional[provisional_vertex[index]];
    ++topology_source_count[static_cast<std::size_t>(
        topology_vertex[index])];
  }
  std::vector<std::uint8_t> repair_seed(topology_vertex_count, 0U);
  for (std::size_t vertex = 0U; vertex < topology_source_count.size();
       ++vertex) {
    repair_seed[vertex] = topology_source_count[vertex] > 1U ? 1U : 0U;
  }
  const auto mark_repair_triangle = [&repair_seed](
                                        const Eigen::Vector3i& triangle) {
    for (int corner = 0; corner < 3; ++corner) {
      const int vertex = triangle[corner];
      if (vertex >= 0 &&
          static_cast<std::size_t>(vertex) < repair_seed.size()) {
        repair_seed[static_cast<std::size_t>(vertex)] = 1U;
      }
    }
  };

  std::vector<EquivalentCandidateTriangle> candidates;
  candidates.reserve(input.triangles.size());
  std::unordered_map<TriangleKey, std::size_t, TriangleKeyHash>
      candidate_by_key;
  candidate_by_key.reserve(input.triangles.size());
  std::unordered_map<EdgeKey, std::uint8_t> topology;
  topology.reserve(input.triangles.size() * 2U + 1U);
  std::size_t active_boundary_edges = 0U;
  bool has_nonmanifold_edge = false;
  const auto add_topology_edge =
      [&topology, &active_boundary_edges,
       &has_nonmanifold_edge](EdgeKey edge) {
        std::uint8_t& count = topology[edge];
        if (count == 0U) {
          ++active_boundary_edges;
        } else if (count == 1U) {
          --active_boundary_edges;
        } else {
          has_nonmanifold_edge = true;
        }
        count = static_cast<std::uint8_t>(
            std::min<std::uint8_t>(3U, count + 1U));
      };
  for (const Eigen::Vector3i& triangle : input.triangles) {
    if (triangle.minCoeff() < 0 ||
        static_cast<std::size_t>(triangle.maxCoeff()) >=
            input.vertices.size()) {
      ++stats.rejected_invalid_index;
      continue;
    }
    const Eigen::Vector3i topology_triangle(
        topology_vertex[static_cast<std::size_t>(triangle.x())],
        topology_vertex[static_cast<std::size_t>(triangle.y())],
        topology_vertex[static_cast<std::size_t>(triangle.z())]);
    if (topology_triangle.x() == topology_triangle.y() ||
        topology_triangle.y() == topology_triangle.z() ||
        topology_triangle.x() == topology_triangle.z()) {
      ++stats.rejected_equivalent_degenerate;
      mark_repair_triangle(topology_triangle);
      continue;
    }
    EquivalentCandidateTriangle candidate;
    candidate.output_indices = triangle;
    candidate.topology_indices = topology_triangle;
    candidate.canonical = canonicalTriangle(topology_triangle);
    const auto inserted =
        candidate_by_key.emplace(candidate.canonical, candidates.size());
    if (!inserted.second) {
      ++stats.rejected_equivalent_duplicate;
      mark_repair_triangle(topology_triangle);
      EquivalentCandidateTriangle& retained =
          candidates[inserted.first->second];
      if (equivalentCandidateLess(input, candidate, retained)) {
        retained = candidate;
      }
      continue;
    }
    for (const EdgeKey edge : triangleEdges(candidate.topology_indices)) {
      add_topology_edge(edge);
    }
    candidates.push_back(candidate);
  }

  std::vector<std::size_t> accepted_indices;
  accepted_indices.reserve(candidates.size());
  if (!has_nonmanifold_edge) {
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      accepted_indices.push_back(index);
    }
  } else {
    std::vector<std::size_t> canonical_order(candidates.size());
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      canonical_order[index] = index;
    }
    std::sort(canonical_order.begin(), canonical_order.end(),
              [&input, &candidates](std::size_t left, std::size_t right) {
                return equivalentCandidateLess(input, candidates[left],
                                               candidates[right]);
              });
    for (auto& edge : topology) {
      edge.second = 0U;
    }
    active_boundary_edges = 0U;
    has_nonmanifold_edge = false;
    for (const std::size_t index : canonical_order) {
      const auto edges =
          triangleEdges(candidates[index].topology_indices);
      const bool reject = std::any_of(
          edges.begin(), edges.end(),
          [&topology](EdgeKey edge) {
            const auto found = topology.find(edge);
            return found != topology.end() && found->second >= 2U;
          });
      if (reject) {
        ++stats.rejected_equivalent_fan;
        mark_repair_triangle(candidates[index].topology_indices);
        continue;
      }
      accepted_indices.push_back(index);
      for (const EdgeKey edge : edges) {
        add_topology_edge(edge);
      }
    }
  }

  std::size_t equivalent_vertices_before = 0U;
  std::size_t equivalent_vertices_after = 0U;
  std::vector<std::size_t> repair_vertices;
  repair_vertices.reserve(topology_vertex_count);
  for (std::size_t vertex = 0U; vertex < repair_seed.size(); ++vertex) {
    if (repair_seed[vertex] != 0U) {
      repair_vertices.push_back(vertex);
    }
  }
  std::vector<std::size_t> removed_by_vertex_repair;
  std::vector<double> candidate_area(candidates.size(), 0.0);
  std::vector<std::uint8_t> candidate_area_computed(candidates.size(), 0U);
  const auto equivalent_area =
      [&input, &candidates, &candidate_area,
       &candidate_area_computed](std::size_t index) {
        if (candidate_area_computed[index] == 0U) {
          const Eigen::Vector3i& triangle =
              candidates[index].output_indices;
          const Eigen::Vector3f& first = input.vertices[
              static_cast<std::size_t>(triangle.x())];
          const Eigen::Vector3f& second = input.vertices[
              static_cast<std::size_t>(triangle.y())];
          const Eigen::Vector3f& third = input.vertices[
              static_cast<std::size_t>(triangle.z())];
          candidate_area[index] =
              0.5 * ((second - first).cast<double>())
                        .cross((third - first).cast<double>())
                        .norm();
          candidate_area_computed[index] = 1U;
        }
        return candidate_area[index];
      };
  repairNonmanifoldVertexLinks(
      topology_vertex_count, candidates, accepted_indices,
      stats.rejected_equivalent_fan, equivalent_vertices_before,
      equivalent_vertices_after, equivalent_area, &repair_vertices,
      &removed_by_vertex_repair);
  for (const std::size_t index : removed_by_vertex_repair) {
    for (const EdgeKey edge :
         triangleEdges(candidates[index].topology_indices)) {
      const auto found = topology.find(edge);
      if (found == topology.end() || found->second == 0U) {
        continue;
      }
      if (found->second == 2U) {
        ++active_boundary_edges;
      } else if (found->second == 1U) {
        --active_boundary_edges;
      }
      --found->second;
    }
  }
  result.topology.boundary_edges = active_boundary_edges;
  result.topology.nonmanifold_edges = has_nonmanifold_edge ? 1U : 0U;
  result.topology.nonmanifold_vertices = equivalent_vertices_after;
  result.topology.valid = result.topology.nonmanifold_edges == 0U &&
                          result.topology.nonmanifold_vertices == 0U;
  result.topology.reason =
      result.topology.nonmanifold_edges > 0U
          ? "nonmanifold_edge"
          : (result.topology.nonmanifold_vertices > 0U
                 ? "nonmanifold_vertex"
                 : "accepted");
  if (!result.topology.valid) {
    return result;
  }

  std::sort(accepted_indices.begin(), accepted_indices.end(),
            [&input, &candidates](std::size_t left, std::size_t right) {
              return equivalentCandidateLess(input, candidates[left],
                                             candidates[right]);
            });
  std::vector<std::uint8_t> vertex_used(input.vertices.size(), 0U);
  for (const std::size_t index : accepted_indices) {
    for (int corner = 0; corner < 3; ++corner) {
      vertex_used[static_cast<std::size_t>(
          candidates[index].output_indices[corner])] = 1U;
    }
  }
  std::vector<int> output_vertex(input.vertices.size(), -1);
  result.mesh.vertices.reserve(input.vertices.size());
  result.quantized_vertices.keys.reserve(input.vertices.size());
  result.quantized_vertices.valid.reserve(input.vertices.size());
  for (std::size_t index = 0U; index < input.vertices.size(); ++index) {
    if (vertex_used[index] == 0U) {
      continue;
    }
    output_vertex[index] = static_cast<int>(result.mesh.vertices.size());
    result.mesh.vertices.push_back(input.vertices[index]);
    if (provisional_vertex[index] == no_provisional_vertex) {
      result.quantized_vertices.keys.emplace_back();
      result.quantized_vertices.valid.push_back(0U);
    } else {
      result.quantized_vertices.keys.push_back(
          unique_keys[provisional_vertex[index]]);
      result.quantized_vertices.valid.push_back(1U);
    }
  }
  result.mesh.triangles.reserve(accepted_indices.size());
  for (const std::size_t index : accepted_indices) {
    const Eigen::Vector3i& triangle = candidates[index].output_indices;
    result.mesh.triangles.emplace_back(
        output_vertex[static_cast<std::size_t>(triangle.x())],
        output_vertex[static_cast<std::size_t>(triangle.y())],
        output_vertex[static_cast<std::size_t>(triangle.z())]);
  }
  stats.output_vertices = result.mesh.vertices.size();
  stats.output_triangles = result.mesh.triangles.size();
  stats.boundary_edges_after_cleanup = result.topology.boundary_edges;
  stats.nonmanifold_edges_after_cleanup = 0U;
  stats.nonmanifold_vertices_after_cleanup = 0U;
  return result;
}

}  // namespace

std::size_t MeshCleanupStats::rejectedTotal() const {
  return rejected_invalid_index + rejected_nonfinite + rejected_degenerate +
         rejected_duplicate + rejected_large_face + rejected_nonmanifold +
         rejected_nonmanifold_vertex + rejected_equivalent_degenerate +
         rejected_equivalent_duplicate + rejected_equivalent_fan +
         rejected_small_component;
}

MeshCleanupConfig marchingTetraCleanupConfig(float voxel_size) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0f) {
    throw std::invalid_argument("mesh cleanup voxel size must be finite and positive");
  }
  const double voxel = static_cast<double>(voxel_size);
  MeshCleanupConfig config;
  config.minimum_triangle_area = voxel * voxel * 1e-8;
  config.maximum_triangle_area = voxel * voxel * 2.0;
  config.maximum_edge_length = voxel * std::sqrt(3.0) * (1.0 + 1e-5);
  config.vertex_merge_tolerance = voxel * 1e-5;
  config.minimum_component_triangles = 3U;
  config.maximum_small_component_area = voxel * voxel * 2.0;
  config.maximum_small_component_extent = voxel * 3.0;
  return config;
}

MeshCleanupConfig supportedStripValidationConfig(
    float voxel_size, float maximum_mesh_edge_voxels) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0f ||
      !std::isfinite(maximum_mesh_edge_voxels) ||
      maximum_mesh_edge_voxels <= 0.0f) {
    throw std::invalid_argument(
        "supported strip validation scale must be finite and positive");
  }
  const double voxel = static_cast<double>(voxel_size);
  const double maximum_edge =
      voxel * static_cast<double>(maximum_mesh_edge_voxels) * (1.0 + 1e-5);
  MeshCleanupConfig config;
  config.minimum_triangle_area = voxel * voxel * 1e-8;
  config.maximum_triangle_area =
      std::sqrt(3.0) * 0.25 * maximum_edge * maximum_edge * (1.0 + 1e-5);
  config.maximum_edge_length = maximum_edge;
  config.vertex_merge_tolerance = 0.0;
  config.minimum_component_triangles = 1U;
  config.maximum_small_component_area = 0.0;
  config.maximum_small_component_extent = 0.0;
  return config;
}

MeshCleanupResult cleanupMeshTopology(const IndexedMesh& input,
                                      const MeshCleanupConfig& config) {
  validateConfig(config);
  MeshCleanupResult result;
  result.mesh.triangle_limit_reached = input.triangle_limit_reached;
  result.stats.input_vertices = input.vertices.size();
  result.stats.input_triangles = input.triangles.size();

  // Marching tetrahedra may reach an iso-surface exactly at a lattice vertex.
  // That one position is then cached under several different lattice edges,
  // producing duplicate indices which fragment an otherwise continuous plane.
  // Weld only at a lattice-relative epsilon, choosing the lowest input index
  // deterministically as the representative.
  std::vector<int> canonical_vertex(input.vertices.size(), -1);
  std::unordered_map<QuantizedVertexKey, int, QuantizedVertexKeyHash>
      welded_vertices;
  if (config.vertex_merge_tolerance > 0.0) {
    welded_vertices.reserve(input.vertices.size());
  }
  for (std::size_t index = 0; index < input.vertices.size(); ++index) {
    canonical_vertex[index] = static_cast<int>(index);
    QuantizedVertexKey key;
    if (!quantizedVertexKey(input.vertices[index],
                            config.vertex_merge_tolerance, key)) {
      continue;
    }
    const auto inserted =
        welded_vertices.emplace(key, static_cast<int>(index));
    if (!inserted.second) {
      canonical_vertex[index] = inserted.first->second;
      ++result.stats.merged_vertices;
    }
  }

  std::vector<CandidateTriangle> candidates;
  candidates.reserve(input.triangles.size());
  std::unordered_map<TriangleKey, std::size_t, TriangleKeyHash>
      candidate_by_key;
  candidate_by_key.reserve(input.triangles.size());
  for (const Eigen::Vector3i& triangle : input.triangles) {
    const int first = triangle.x();
    const int second = triangle.y();
    const int third = triangle.z();
    if (first < 0 || second < 0 || third < 0 ||
        static_cast<std::size_t>(first) >= input.vertices.size() ||
        static_cast<std::size_t>(second) >= input.vertices.size() ||
        static_cast<std::size_t>(third) >= input.vertices.size()) {
      ++result.stats.rejected_invalid_index;
      continue;
    }
    const Eigen::Vector3f& first_point = input.vertices[first];
    const Eigen::Vector3f& second_point = input.vertices[second];
    const Eigen::Vector3f& third_point = input.vertices[third];
    if (!finitePoint(first_point) || !finitePoint(second_point) ||
        !finitePoint(third_point)) {
      ++result.stats.rejected_nonfinite;
      continue;
    }

    const Eigen::Vector3i welded_triangle(
        canonical_vertex[static_cast<std::size_t>(first)],
        canonical_vertex[static_cast<std::size_t>(second)],
        canonical_vertex[static_cast<std::size_t>(third)]);
    if (welded_triangle.x() == welded_triangle.y() ||
        welded_triangle.y() == welded_triangle.z() ||
        welded_triangle.x() == welded_triangle.z()) {
      ++result.stats.rejected_degenerate;
      continue;
    }
    const Eigen::Vector3f& welded_first_point =
        input.vertices[static_cast<std::size_t>(welded_triangle.x())];
    const Eigen::Vector3f& welded_second_point =
        input.vertices[static_cast<std::size_t>(welded_triangle.y())];
    const Eigen::Vector3f& welded_third_point =
        input.vertices[static_cast<std::size_t>(welded_triangle.z())];

    const Eigen::Vector3d ab =
        welded_second_point.cast<double>() - welded_first_point.cast<double>();
    const Eigen::Vector3d ac =
        welded_third_point.cast<double>() - welded_first_point.cast<double>();
    const Eigen::Vector3d bc =
        welded_third_point.cast<double>() - welded_second_point.cast<double>();
    const double maximum_edge =
        std::sqrt(std::max(ab.squaredNorm(),
                           std::max(ac.squaredNorm(), bc.squaredNorm())));
    const double area = 0.5 * ab.cross(ac).norm();
    if (!std::isfinite(area) || !std::isfinite(maximum_edge) ||
        area <= config.minimum_triangle_area) {
      ++result.stats.rejected_degenerate;
      continue;
    }
    if (area > config.maximum_triangle_area ||
        maximum_edge > config.maximum_edge_length) {
      ++result.stats.rejected_large_face;
      continue;
    }
    CandidateTriangle candidate;
    candidate.indices = welded_triangle;
    candidate.canonical = canonicalTriangle(welded_triangle);
    candidate.area = area;
    const auto inserted =
        candidate_by_key.emplace(candidate.canonical, candidates.size());
    if (!inserted.second) {
      ++result.stats.rejected_duplicate;
      CandidateTriangle& retained = candidates[inserted.first->second];
      if (orientedTriangle(candidate.indices) <
          orientedTriangle(retained.indices)) {
        retained = candidate;
      }
      continue;
    }
    candidates.push_back(candidate);
  }

  // The extractor walks cells deterministically and normally emits a manifold
  // mesh. Build one edge graph which supplies both the pre-cleanup edge counts
  // and connected components. Only the exceptional non-manifold case pays for
  // canonical sorting and a second edge graph.
  std::unordered_map<EdgeKey, EdgeTopology> topology;
  topology.reserve(candidates.size() * 2U + 1U);
  for (std::size_t index = 0; index < candidates.size(); ++index) {
    for (const EdgeKey edge : triangleEdges(candidates[index].indices)) {
      const auto inserted = topology.emplace(edge, EdgeTopology());
      EdgeTopology& entry = inserted.first->second;
      if (inserted.second) {
        entry.first_triangle = index;
      }
      ++entry.count;
    }
  }
  for (const auto& item : topology) {
    if (item.second.count == 1U) {
      ++result.stats.boundary_edges_before_cleanup;
    } else if (item.second.count > 2U) {
      ++result.stats.nonmanifold_edges_before_cleanup;
    }
  }

  std::vector<std::size_t> accepted_indices;
  accepted_indices.reserve(candidates.size());
  std::unordered_map<EdgeKey, EdgeTopology> rebuilt_topology;
  if (result.stats.nonmanifold_edges_before_cleanup == 0U) {
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      accepted_indices.push_back(index);
    }
  } else {
    std::vector<std::size_t> canonical_order;
    canonical_order.reserve(candidates.size());
    for (std::size_t index = 0; index < candidates.size(); ++index) {
      canonical_order.push_back(index);
    }
    std::sort(canonical_order.begin(), canonical_order.end(),
              [&candidates](std::size_t left, std::size_t right) {
                if (candidates[left].canonical !=
                    candidates[right].canonical) {
                  return candidates[left].canonical <
                         candidates[right].canonical;
                }
                return orientedTriangle(candidates[left].indices) <
                       orientedTriangle(candidates[right].indices);
              });
    rebuilt_topology.reserve(candidates.size() * 2U + 1U);
    for (const std::size_t index : canonical_order) {
      const auto edges = triangleEdges(candidates[index].indices);
      bool would_be_nonmanifold = false;
      for (const EdgeKey edge : edges) {
        const auto found = rebuilt_topology.find(edge);
        if (found != rebuilt_topology.end() && found->second.count >= 2U) {
          would_be_nonmanifold = true;
          break;
        }
      }
      if (would_be_nonmanifold) {
        ++result.stats.rejected_nonmanifold;
        continue;
      }
      accepted_indices.push_back(index);
      for (const EdgeKey edge : edges) {
        const auto inserted = rebuilt_topology.emplace(edge, EdgeTopology());
        EdgeTopology& entry = inserted.first->second;
        if (inserted.second) {
          entry.first_triangle = index;
        }
        ++entry.count;
      }
    }
  }

  std::vector<std::size_t> removed_by_vertex_repair;
  const auto cleanup_area = [&candidates](std::size_t index) {
    return candidates[index].area;
  };
  repairNonmanifoldVertexLinks(
      input.vertices.size(), candidates, accepted_indices,
      result.stats.rejected_nonmanifold_vertex,
      result.stats.nonmanifold_vertices_before_cleanup,
      result.stats.nonmanifold_vertices_after_cleanup, cleanup_area, nullptr,
      &removed_by_vertex_repair);

  // Edge filtering already built the exact accepted edge table. Vertex-link
  // repair can only delete faces, so update those three counts in place. This
  // preserves the original edge identities and avoids allocating and hashing
  // a third full edge graph. Component connectivity is still rebuilt from the
  // final face set because deleting a fan can split a component.
  std::unordered_map<EdgeKey, EdgeTopology> accepted_topology =
      result.stats.nonmanifold_edges_before_cleanup == 0U
          ? std::move(topology)
          : std::move(rebuilt_topology);
  for (const std::size_t index : removed_by_vertex_repair) {
    for (const EdgeKey edge : triangleEdges(candidates[index].indices)) {
      const auto found = accepted_topology.find(edge);
      if (found != accepted_topology.end() && found->second.count > 0U) {
        --found->second.count;
      }
    }
  }
  const std::size_t no_triangle = candidates.size();
  for (auto& edge : accepted_topology) {
    edge.second.first_triangle = no_triangle;
  }
  DisjointSet components(candidates.size());
  for (const std::size_t index : accepted_indices) {
    for (const EdgeKey edge : triangleEdges(candidates[index].indices)) {
      const auto found = accepted_topology.find(edge);
      if (found == accepted_topology.end() || found->second.count == 0U) {
        continue;
      }
      EdgeTopology& entry = found->second;
      if (entry.first_triangle == no_triangle) {
        entry.first_triangle = index;
      } else {
        components.unite(index, entry.first_triangle);
      }
    }
  }

  std::vector<ComponentSummary> summaries(candidates.size());
  std::vector<std::uint8_t> component_present(candidates.size(), 0U);
  for (const std::size_t index : accepted_indices) {
    const std::size_t root = components.find(index);
    ComponentSummary& summary = summaries[root];
    if (component_present[root] == 0U) {
      component_present[root] = 1U;
      ++result.stats.components_before_filter;
    }
    ++summary.triangle_count;
    summary.area += candidates[index].area;
    const Eigen::Vector3i& triangle = candidates[index].indices;
    for (int corner = 0; corner < 3; ++corner) {
      const Eigen::Vector3d point =
          input.vertices[triangle[corner]].cast<double>();
      summary.minimum = summary.minimum.cwiseMin(point);
      summary.maximum = summary.maximum.cwiseMax(point);
    }
  }

  std::vector<std::uint8_t> keep_component(candidates.size(), 0U);
  for (std::size_t root = 0; root < summaries.size(); ++root) {
    if (component_present[root] == 0U) {
      continue;
    }
    const ComponentSummary& summary = summaries[root];
    const double extent = (summary.maximum - summary.minimum).norm();
    const bool remove =
        summary.triangle_count < config.minimum_component_triangles &&
        config.maximum_small_component_area > 0.0 &&
        summary.area <= config.maximum_small_component_area &&
        config.maximum_small_component_extent > 0.0 &&
        extent <= config.maximum_small_component_extent;
    keep_component[root] = remove ? 0U : 1U;
    if (!remove) {
      ++result.stats.components_after_filter;
    }
  }

  std::vector<std::uint8_t> keep_candidate(candidates.size(), 0U);
  for (const std::size_t index : accepted_indices) {
    const std::size_t root = components.find(index);
    keep_candidate[index] = keep_component[root];
    if (keep_candidate[index] == 0U) {
      ++result.stats.rejected_small_component;
    }
  }

  // Removing a whole connected component cannot change an edge degree in a
  // retained component, so no post-filter edge map is necessary.
  for (const auto& item : accepted_topology) {
    if (item.second.count == 0U ||
        item.second.first_triangle == no_triangle) {
      continue;
    }
    if (keep_candidate[item.second.first_triangle] == 0U) {
      continue;
    }
    if (item.second.count == 1U) {
      ++result.stats.boundary_edges_after_cleanup;
    } else if (item.second.count > 2U) {
      ++result.stats.nonmanifold_edges_after_cleanup;
    }
  }

  std::sort(accepted_indices.begin(), accepted_indices.end(),
            [&candidates](std::size_t left, std::size_t right) {
              if (candidates[left].canonical != candidates[right].canonical) {
                return candidates[left].canonical < candidates[right].canonical;
              }
              return orientedTriangle(candidates[left].indices) <
                     orientedTriangle(candidates[right].indices);
            });

  std::vector<bool> vertex_used(input.vertices.size(), false);
  for (const std::size_t index : accepted_indices) {
    if (keep_candidate[index] == 0U) {
      continue;
    }
    const CandidateTriangle& candidate = candidates[index];
    for (int corner = 0; corner < 3; ++corner) {
      vertex_used[static_cast<std::size_t>(candidate.indices[corner])] = true;
    }
  }

  std::vector<int> remap(input.vertices.size(), -1);
  result.mesh.vertices.reserve(input.vertices.size());
  for (std::size_t index = 0; index < input.vertices.size(); ++index) {
    if (!vertex_used[index]) {
      continue;
    }
    remap[index] = static_cast<int>(result.mesh.vertices.size());
    result.mesh.vertices.push_back(input.vertices[index]);
  }
  result.mesh.triangles.reserve(accepted_indices.size() -
                                result.stats.rejected_small_component);
  for (const std::size_t index : accepted_indices) {
    if (keep_candidate[index] == 0U) {
      continue;
    }
    const CandidateTriangle& candidate = candidates[index];
    result.mesh.triangles.emplace_back(
        remap[static_cast<std::size_t>(candidate.indices.x())],
        remap[static_cast<std::size_t>(candidate.indices.y())],
        remap[static_cast<std::size_t>(candidate.indices.z())]);
  }

  result.stats.output_vertices = result.mesh.vertices.size();
  result.stats.output_triangles = result.mesh.triangles.size();
  return result;
}

namespace {

MeshTopologyValidationResult validateMeshTopologyImpl(
    const IndexedMesh& mesh, double vertex_equivalence_tolerance,
    SupportGeometryValidation* geometry,
    QuantizedVertexCache* quantized_vertices) {
  if (!std::isfinite(vertex_equivalence_tolerance) ||
      vertex_equivalence_tolerance < 0.0) {
    throw std::invalid_argument(
        "mesh topology equivalence tolerance must be finite and nonnegative");
  }
  MeshTopologyValidationResult result;
  if (quantized_vertices != nullptr) {
    quantized_vertices->keys.resize(mesh.vertices.size());
    quantized_vertices->valid.assign(mesh.vertices.size(), 0U);
  }
  std::vector<int> canonical_vertex(mesh.vertices.size(), -1);
  std::unordered_map<QuantizedVertexKey, int, QuantizedVertexKeyHash>
      equivalent_vertices;
  if (vertex_equivalence_tolerance > 0.0) {
    equivalent_vertices.reserve(mesh.vertices.size());
  }
  for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
    canonical_vertex[index] = static_cast<int>(index);
    if (vertex_equivalence_tolerance <= 0.0) {
      continue;
    }
    QuantizedVertexKey key;
    if (!quantizedVertexKey(mesh.vertices[index],
                            vertex_equivalence_tolerance, key)) {
      continue;
    }
    if (quantized_vertices != nullptr) {
      quantized_vertices->keys[index] = key;
      quantized_vertices->valid[index] = 1U;
    }
    const auto inserted =
        equivalent_vertices.emplace(key, static_cast<int>(index));
    if (!inserted.second) {
      canonical_vertex[index] = inserted.first->second;
    }
  }

  std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>
      triangles;
  triangles.reserve(mesh.triangles.size());
  std::unordered_map<TriangleKey, std::size_t, TriangleKeyHash> unique_faces;
  unique_faces.reserve(mesh.triangles.size());
  for (const Eigen::Vector3i& triangle : mesh.triangles) {
    if (triangle.minCoeff() < 0 ||
        static_cast<std::size_t>(triangle.maxCoeff()) >=
            mesh.vertices.size()) {
      ++result.invalid_indices;
      continue;
    }
    bool finite = true;
    for (int corner = 0; corner < 3; ++corner) {
      finite = finite && finitePoint(
                             mesh.vertices[static_cast<std::size_t>(
                                 triangle[corner])]);
    }
    if (!finite) {
      ++result.nonfinite_triangles;
      continue;
    }
    if (geometry != nullptr) {
      if (triangle.x() == triangle.y() || triangle.y() == triangle.z() ||
          triangle.x() == triangle.z()) {
        ++geometry->degenerate;
      } else {
        const Eigen::Vector3f& first =
            mesh.vertices[static_cast<std::size_t>(triangle.x())];
        const Eigen::Vector3f& second =
            mesh.vertices[static_cast<std::size_t>(triangle.y())];
        const Eigen::Vector3f& third =
            mesh.vertices[static_cast<std::size_t>(triangle.z())];
        const Eigen::Vector3d ab =
            second.cast<double>() - first.cast<double>();
        const Eigen::Vector3d ac =
            third.cast<double>() - first.cast<double>();
        const Eigen::Vector3d bc =
            third.cast<double>() - second.cast<double>();
        const double maximum_edge = std::sqrt(
            std::max(ab.squaredNorm(),
                     std::max(ac.squaredNorm(), bc.squaredNorm())));
        const double area = 0.5 * ab.cross(ac).norm();
        if (!std::isfinite(area) || !std::isfinite(maximum_edge) ||
            area <= geometry->config.minimum_triangle_area) {
          ++geometry->degenerate;
        } else if (area > geometry->config.maximum_triangle_area ||
                   maximum_edge > geometry->config.maximum_edge_length) {
          ++geometry->large_faces;
        } else {
          ++geometry->triangles;
          for (int corner = 0; corner < 3; ++corner) {
            geometry->vertex_used[static_cast<std::size_t>(
                triangle[corner])] = 1U;
          }
        }
      }
    }
    const Eigen::Vector3i canonical(
        canonical_vertex[static_cast<std::size_t>(triangle.x())],
        canonical_vertex[static_cast<std::size_t>(triangle.y())],
        canonical_vertex[static_cast<std::size_t>(triangle.z())]);
    if (canonical.x() == canonical.y() || canonical.y() == canonical.z() ||
        canonical.x() == canonical.z()) {
      ++result.degenerate_triangles;
      continue;
    }
    const Eigen::Vector3f& a =
        mesh.vertices[static_cast<std::size_t>(triangle.x())];
    const Eigen::Vector3f& b =
        mesh.vertices[static_cast<std::size_t>(triangle.y())];
    const Eigen::Vector3f& c =
        mesh.vertices[static_cast<std::size_t>(triangle.z())];
    const Eigen::Vector3f normal = (b - a).cross(c - a);
    if (!normal.allFinite() || normal.squaredNorm() <= 1e-20f) {
      ++result.degenerate_triangles;
      continue;
    }
    const TriangleKey key = canonicalTriangle(canonical);
    if (!unique_faces.emplace(key, triangles.size()).second) {
      ++result.duplicate_triangles;
      continue;
    }
    triangles.push_back(canonical);
  }

  std::vector<std::size_t> link_offsets(mesh.vertices.size() + 1U, 0U);
  for (const Eigen::Vector3i& triangle : triangles) {
    const std::array<int, 3> vertices =
        {{triangle.x(), triangle.y(), triangle.z()}};
    for (std::size_t corner = 0; corner < vertices.size(); ++corner) {
      ++link_offsets[static_cast<std::size_t>(vertices[corner]) + 1U];
    }
  }
  for (std::size_t vertex = 0U; vertex < mesh.vertices.size(); ++vertex) {
    link_offsets[vertex + 1U] += link_offsets[vertex];
  }
  std::vector<std::array<int, 2>> vertex_links(link_offsets.back());
  std::vector<std::size_t> link_cursors(link_offsets.begin(),
                                        link_offsets.end() - 1U);
  for (const Eigen::Vector3i& triangle : triangles) {
    const std::array<int, 3> vertices =
        {{triangle.x(), triangle.y(), triangle.z()}};
    for (std::size_t corner = 0U; corner < vertices.size(); ++corner) {
      const std::size_t vertex =
          static_cast<std::size_t>(vertices[corner]);
      vertex_links[link_cursors[vertex]++] =
          {{vertices[(corner + 1U) % 3U],
            vertices[(corner + 2U) % 3U]}};
    }
  }
  VertexLinkValidationWorkspace workspace(mesh.vertices.size());
  std::size_t boundary_neighbor_count = 0U;
  std::size_t nonmanifold_neighbor_count = 0U;
  for (std::size_t vertex = 0; vertex < mesh.vertices.size(); ++vertex) {
    if (!validVertexLink(vertex_links, link_offsets[vertex],
                         link_offsets[vertex + 1U], workspace,
                         boundary_neighbor_count,
                         nonmanifold_neighbor_count)) {
      ++result.nonmanifold_vertices;
    }
  }
  // A face incident to edge (u,v) contributes one link degree at v in u's
  // link and one at u in v's link. Unique canonical faces therefore count
  // every boundary or non-manifold edge exactly twice without a separate
  // whole-mesh edge hash table.
  result.boundary_edges = boundary_neighbor_count / 2U;
  result.nonmanifold_edges = nonmanifold_neighbor_count / 2U;

  result.valid = result.invalid_indices == 0U &&
                 result.nonfinite_triangles == 0U &&
                 result.degenerate_triangles == 0U &&
                 result.duplicate_triangles == 0U &&
                 result.nonmanifold_edges == 0U &&
                 result.nonmanifold_vertices == 0U;
  if (result.invalid_indices > 0U) {
    result.reason = "invalid_index";
  } else if (result.nonfinite_triangles > 0U) {
    result.reason = "nonfinite";
  } else if (result.degenerate_triangles > 0U) {
    result.reason = "degenerate";
  } else if (result.duplicate_triangles > 0U) {
    result.reason = "duplicate";
  } else if (result.nonmanifold_edges > 0U) {
    result.reason = "nonmanifold_edge";
  } else if (result.nonmanifold_vertices > 0U) {
    result.reason = "nonmanifold_vertex";
  } else {
    result.reason = "accepted";
  }
  return result;
}

}  // namespace

MeshTopologyValidationResult validateMeshTopology(
    const IndexedMesh& mesh, double vertex_equivalence_tolerance) {
  return validateMeshTopologyImpl(mesh, vertex_equivalence_tolerance,
                                  nullptr, nullptr);
}

LayeredMeshPreparationResult prepareLayeredMeshForPublish(
    const IndexedMesh& raw_base, const IndexedMesh& raw_support,
    std::size_t max_total_triangles,
    const MeshCleanupConfig& base_cleanup_config,
    const MeshCleanupConfig& support_validation_config,
    double output_vertex_equivalence_tolerance,
    bool parallel_support_validation) {
  using PreparationClock = std::chrono::steady_clock;
  const auto elapsed_ms = [](const PreparationClock::time_point& start) {
    return std::chrono::duration<double, std::milli>(PreparationClock::now() -
                                                     start)
        .count();
  };
  LayeredMeshPreparationResult result;
  const bool support_validation_needed =
      !raw_support.triangle_limit_reached &&
      !raw_support.triangles.empty();
  std::future<SupportValidationWork> support_future;
  SupportValidationFutureGuard support_future_guard(support_future);
  if (parallel_support_validation && support_validation_needed) {
    const IndexedMesh* const support = &raw_support;
    const MeshCleanupConfig support_config = support_validation_config;
    try {
      support_future = std::async(
          std::launch::async,
          [support, support_config, output_vertex_equivalence_tolerance]() {
            return validateSupportReadOnly(
                *support, support_config,
                output_vertex_equivalence_tolerance);
          });
      result.support_validation_parallel = true;
    } catch (const std::system_error&) {
      result.support_validation_launch_failed = true;
    } catch (const std::bad_alloc&) {
      result.support_validation_launch_failed = true;
    }
  }
  PreparationClock::time_point stage_start = PreparationClock::now();
  MeshCleanupResult cleaned_base =
      cleanupMeshTopology(raw_base, base_cleanup_config);
  result.timings.base_cleanup_ms = elapsed_ms(stage_start);
  result.base_cleanup = cleaned_base.stats;
  stage_start = PreparationClock::now();
  EquivalentTopologyRepairResult repaired_base =
      repairEquivalentBaseTopology(std::move(cleaned_base.mesh),
                                   output_vertex_equivalence_tolerance,
                                   result.base_cleanup);
  result.timings.base_equivalent_repair_ms = elapsed_ms(stage_start);
  result.mesh = std::move(repaired_base.mesh);
  result.base_output_topology = std::move(repaired_base.topology);
  if (!result.base_output_topology.valid) {
    // Equivalent topology repair only deletes complete faces and never moves
    // coordinates. Any residual conflict fails closed so the node emits
    // DELETE instead of publishing an ambiguous mesh.
    result.mesh.vertices.clear();
    result.mesh.triangles.clear();
    result.support_reason = "base_output_topology_invalid";
    if (support_future.valid()) {
      stage_start = PreparationClock::now();
      support_future.wait();
      result.timings.support_validation_wait_ms = elapsed_ms(stage_start);
    }
    return result;
  }

  if (raw_support.triangle_limit_reached) {
    result.support_budget_limited = true;
    result.support_reason = "support_triangle_limit";
    return result;
  }
  if (raw_support.triangles.empty()) {
    result.support_reason = "empty_addition";
    return result;
  }
  SupportValidationWork support_work;
  if (support_future.valid()) {
    stage_start = PreparationClock::now();
    support_work = support_future.get();
    result.timings.support_validation_wait_ms = elapsed_ms(stage_start);
  } else {
    support_work = validateSupportReadOnly(
        raw_support, support_validation_config,
        output_vertex_equivalence_tolerance);
  }
  result.timings.support_validation_ms = support_work.elapsed_ms;
  result.support_validation = std::move(support_work.stats);
  result.support_output_topology = std::move(support_work.topology);
  QuantizedVertexCache support_quantized_vertices =
      std::move(support_work.quantized_vertices);
  if (!support_work.unchanged) {
    result.support_reason = "support_validation_changed_mesh";
    return result;
  }
  // cleanupMeshTopology is used only as a validator for direct strips.  It
  // may canonically reorder triangles even when it rejects nothing, so the
  // original indexed mesh is the only all-or-nothing payload we may append.
  if (!result.support_output_topology.valid) {
    result.support_reason = "support_output_topology_invalid";
    return result;
  }

  IndexedMesh candidate = result.mesh;
  stage_start = PreparationClock::now();
  const MeshAppendResult appended = appendIndexedMeshAtomic(
      raw_support, max_total_triangles, candidate);
  result.timings.append_ms = elapsed_ms(stage_start);
  if (!appended.applied) {
    result.support_budget_limited = appended.budget_limited;
    result.support_reason = appended.reason;
    return result;
  }
  stage_start = PreparationClock::now();
  const bool have_cached_quantization =
      repaired_base.quantized_vertices.keys.size() ==
          result.mesh.vertices.size() &&
      repaired_base.quantized_vertices.valid.size() ==
          result.mesh.vertices.size() &&
      support_quantized_vertices.keys.size() == raw_support.vertices.size() &&
      support_quantized_vertices.valid.size() == raw_support.vertices.size();
  const bool shares_equivalent_vertex =
      have_cached_quantization
          ? quantizedCachesShareVertex(repaired_base.quantized_vertices,
                                       support_quantized_vertices)
          : meshesShareEquivalentVertex(
                result.mesh, raw_support,
                output_vertex_equivalence_tolerance);
  result.timings.cross_equivalence_probe_ms = elapsed_ms(stage_start);
  if (shares_equivalent_vertex) {
    stage_start = PreparationClock::now();
    result.combined_topology = validateMeshTopology(
        candidate, output_vertex_equivalence_tolerance);
    result.timings.combined_validation_ms = elapsed_ms(stage_start);
  } else {
    // Two independently valid indexed surfaces with disjoint output
    // equivalence classes cannot create a cross-source edge or vertex-link
    // conflict.  Avoid rebuilding the full combined link graph in this common
    // case; this is a proof-based fast path, not a relaxed validator.
    result.combined_topology.valid = true;
    result.combined_topology.reason = "accepted";
    result.combined_topology.boundary_edges =
        result.base_output_topology.boundary_edges +
        result.support_output_topology.boundary_edges;
  }
  if (!result.combined_topology.valid) {
    result.support_reason = "cross_source_topology_conflict";
    return result;
  }
  result.mesh = std::move(candidate);
  result.support_applied = true;
  result.support_reason = "applied";
  return result;
}

}  // namespace local_tsdf_mesh
