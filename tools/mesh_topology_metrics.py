#!/usr/bin/env python3
"""Measure mesh boundary, component, and tunnel cross-section closure metrics.

The tool has no geometry-library dependency.  It accepts either a Wavefront
OBJ mesh or a ROS bag containing a visualization_msgs/Marker TRIANGLE_LIST.
It is intended for repeatable offline checks of /local_tsdf_mesh/mesh output.
"""

import argparse
import json
import math
from collections import Counter, defaultdict, deque
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


def cpp_llround(values: np.ndarray) -> np.ndarray:
    """Return the elementwise equivalent of C++ ``std::llround``.

    NumPy's ``rint`` uses round-to-nearest-even, while ``std::llround`` rounds
    halfway cases away from zero regardless of the floating-point rounding
    mode.  Mesh vertex keys must use the latter so the offline topology check
    groups vertices exactly like the C++ publisher.

    The C++ implementation rejects non-finite or out-of-int64-range scaled
    coordinates before calling ``std::llround``.  Mirror that contract here.
    """
    array = np.asarray(values, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        raise ValueError("values to cpp_llround must be finite")

    flat = array.reshape(-1)
    magnitude = np.abs(flat)
    integral = np.floor(magnitude)
    rounded_magnitude = integral + (magnitude - integral >= 0.5)
    int64_limit = float(1 << 63)
    positive_overflow = (flat >= 0.0) & (rounded_magnitude >= int64_limit)
    negative_overflow = (flat < 0.0) & (rounded_magnitude > int64_limit)
    if np.any(positive_overflow | negative_overflow):
        raise OverflowError("cpp_llround result is outside int64 range")

    output = np.empty(flat.shape, dtype=np.int64)
    int64_minimum = (flat < 0.0) & (rounded_magnitude == int64_limit)
    output[int64_minimum] = np.iinfo(np.int64).min
    regular = ~int64_minimum
    regular_magnitude = rounded_magnitude[regular].astype(np.int64)
    output[regular] = np.where(
        flat[regular] < 0.0,
        -regular_magnitude,
        regular_magnitude,
    )
    return output.reshape(array.shape)


def quantize_coordinates(coordinates: np.ndarray, tolerance_m: float) -> np.ndarray:
    """Build coordinate keys exactly as C++ ``llround(value/tolerance)``."""
    if not math.isfinite(tolerance_m) or tolerance_m <= 0.0:
        raise ValueError("quantization tolerance must be finite and positive")
    array = np.asarray(coordinates, dtype=np.float64)
    if not np.all(np.isfinite(array)):
        raise ValueError("coordinates to quantize must be finite")
    return cpp_llround(array / tolerance_m)


def load_obj(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    vertices: List[List[float]] = []
    faces: List[List[int]] = []
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            words = line.split()
            if words[0] == "v" and len(words) >= 4:
                vertices.append([float(words[1]), float(words[2]), float(words[3])])
            elif words[0] == "f" and len(words) >= 4:
                polygon = []
                for token in words[1:]:
                    raw_index = int(token.split("/", 1)[0])
                    index = raw_index - 1 if raw_index > 0 else len(vertices) + raw_index
                    if index < 0 or index >= len(vertices):
                        raise ValueError(f"{path}:{line_number}: OBJ vertex index is out of range")
                    polygon.append(index)
                for corner in range(1, len(polygon) - 1):
                    faces.append([polygon[0], polygon[corner], polygon[corner + 1]])
    return np.asarray(vertices, dtype=np.float64), np.asarray(faces, dtype=np.int64)


def load_marker_bag(path: Path, topic: str, marker_index: int) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
    try:
        import rosbag  # pylint: disable=import-outside-toplevel
    except ImportError as error:
        raise RuntimeError("ROS Python rosbag is required for --bag") from error

    selected = None
    seen = 0
    with rosbag.Bag(str(path), "r") as bag:
        for _, marker, bag_stamp in bag.read_messages(topics=[topic]):
            # visualization_msgs/Marker::TRIANGLE_LIST is type 11.
            if int(marker.type) != 11 or int(marker.action) not in (0, 1):
                continue
            if marker_index >= 0 and seen != marker_index:
                seen += 1
                continue
            selected = (marker, bag_stamp.to_sec(), seen)
            seen += 1
            if marker_index >= 0:
                break
    if selected is None:
        raise RuntimeError(f"no TRIANGLE_LIST marker found on {topic}")
    marker, bag_stamp_s, actual_index = selected
    if len(marker.points) % 3 != 0:
        raise RuntimeError("TRIANGLE_LIST point count is not divisible by three")
    vertices = np.asarray([[point.x, point.y, point.z] for point in marker.points], dtype=np.float64)
    faces = np.arange(len(vertices), dtype=np.int64).reshape((-1, 3))
    metadata = {
        "topic": topic,
        "marker_index": actual_index,
        "bag_stamp_s": bag_stamp_s,
        "header_stamp_s": marker.header.stamp.to_sec(),
        "frame_id": marker.header.frame_id,
    }
    return vertices, faces, metadata


def quantized_vertex_ids(vertices: np.ndarray, tolerance_m: float) -> Tuple[np.ndarray, np.ndarray]:
    """Weld duplicate/near-duplicate vertices without changing their coordinates."""
    if len(vertices) == 0:
        return np.empty((0,), dtype=np.int64), np.empty((0, 3), dtype=np.float64)
    finite = np.all(np.isfinite(vertices), axis=1)
    inverse = np.full((len(vertices),), -1, dtype=np.int64)
    if not np.any(finite):
        return inverse, np.empty((0, 3), dtype=np.float64)
    finite_vertices = vertices[finite]
    quantized = quantize_coordinates(finite_vertices, tolerance_m)
    unique_keys, finite_inverse = np.unique(quantized, axis=0, return_inverse=True)
    inverse[finite] = finite_inverse
    sums = np.zeros((len(unique_keys), 3), dtype=np.float64)
    counts = np.zeros((len(unique_keys),), dtype=np.int64)
    np.add.at(sums, finite_inverse, finite_vertices)
    np.add.at(counts, finite_inverse, 1)
    return inverse, sums / counts[:, None]


def triangle_areas(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    if len(faces) == 0:
        return np.empty((0,), dtype=np.float64)
    first = vertices[faces[:, 1]] - vertices[faces[:, 0]]
    second = vertices[faces[:, 2]] - vertices[faces[:, 0]]
    return 0.5 * np.linalg.norm(np.cross(first, second), axis=1)


def build_edge_incidence(faces: np.ndarray) -> Tuple[Counter, Dict[Tuple[int, int], List[int]]]:
    counts: Counter = Counter()
    owners: Dict[Tuple[int, int], List[int]] = defaultdict(list)
    for face_index, face in enumerate(faces):
        for left, right in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            edge = (int(min(left, right)), int(max(left, right)))
            counts[edge] += 1
            owners[edge].append(face_index)
    return counts, owners


def face_components(face_count: int, edge_owners: Dict[Tuple[int, int], List[int]]) -> List[List[int]]:
    adjacency: List[List[int]] = [[] for _ in range(face_count)]
    for owners in edge_owners.values():
        for index, left in enumerate(owners):
            for right in owners[index + 1:]:
                adjacency[left].append(right)
                adjacency[right].append(left)
    components = []
    unvisited = set(range(face_count))
    while unvisited:
        start = unvisited.pop()
        component = [start]
        queue = deque([start])
        while queue:
            face = queue.popleft()
            for neighbor in adjacency[face]:
                if neighbor in unvisited:
                    unvisited.remove(neighbor)
                    component.append(neighbor)
                    queue.append(neighbor)
        components.append(component)
    return components


def vertex_link_metrics(
    faces: np.ndarray,
    edge_counts: Counter,
) -> Dict[str, int]:
    """Check that every used vertex has a single path/cycle link.

    Edge incidence alone misses bow-tie vertices: two otherwise manifold
    surface fans may touch at one vertex without sharing an edge.  For a
    triangle mesh, the link of an interior manifold vertex is one cycle and
    the link of a boundary manifold vertex is one path.  This routine checks
    that stronger local condition without relying on triangle winding.
    """
    incident_link_edges: Dict[int, List[Tuple[int, int]]] = defaultdict(list)
    incident_neighbors: Dict[int, set] = defaultdict(set)
    for face in faces:
        first, second, third = (int(face[0]), int(face[1]), int(face[2]))
        incident_link_edges[first].append((min(second, third), max(second, third)))
        incident_link_edges[second].append((min(first, third), max(first, third)))
        incident_link_edges[third].append((min(first, second), max(first, second)))
        incident_neighbors[first].update((second, third))
        incident_neighbors[second].update((first, third))
        incident_neighbors[third].update((first, second))

    boundary_vertices = 0
    bow_tie_vertices = 0
    invalid_link_degree_vertices = 0
    invalid_boundary_degree_vertices = 0
    repeated_link_edge_vertices = 0
    nonmanifold_vertices = 0
    max_link_components = 0

    for vertex, link_edge_list in incident_link_edges.items():
        link_edge_counts = Counter(link_edge_list)
        link_adjacency: Dict[int, set] = defaultdict(set)
        for left, right in link_edge_counts:
            link_adjacency[left].add(right)
            link_adjacency[right].add(left)

        remaining = set(link_adjacency)
        link_components = 0
        while remaining:
            link_components += 1
            start = remaining.pop()
            queue = [start]
            while queue:
                node = queue.pop()
                for neighbor in link_adjacency[node]:
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        queue.append(neighbor)
        max_link_components = max(max_link_components, link_components)

        boundary_degree = sum(
            1
            for neighbor in incident_neighbors[vertex]
            if edge_counts.get((min(vertex, neighbor), max(vertex, neighbor)), 0) == 1
        )
        is_boundary = boundary_degree > 0
        if is_boundary:
            boundary_vertices += 1

        link_degrees = [len(neighbors) for neighbors in link_adjacency.values()]
        degree_one_count = sum(1 for degree in link_degrees if degree == 1)
        degree_two_only = all(degree == 2 for degree in link_degrees)
        valid_link_degree = (
            degree_two_only
            if not is_boundary
            else degree_one_count == 2
            and all(degree in (1, 2) for degree in link_degrees)
        )
        valid_boundary_degree = boundary_degree in (0, 2)
        has_repeated_link_edge = any(count > 1 for count in link_edge_counts.values())

        if link_components > 1:
            bow_tie_vertices += 1
        if not valid_link_degree:
            invalid_link_degree_vertices += 1
        if not valid_boundary_degree:
            invalid_boundary_degree_vertices += 1
        if has_repeated_link_edge:
            repeated_link_edge_vertices += 1
        if (
            link_components != 1
            or not valid_link_degree
            or not valid_boundary_degree
            or has_repeated_link_edge
        ):
            nonmanifold_vertices += 1

    return {
        "used_vertices": len(incident_link_edges),
        "boundary_vertices": boundary_vertices,
        "interior_vertices": len(incident_link_edges) - boundary_vertices,
        "nonmanifold_vertices": nonmanifold_vertices,
        "bow_tie_vertices": bow_tie_vertices,
        "invalid_vertex_link_degree_vertices": invalid_link_degree_vertices,
        "invalid_boundary_degree_vertices": invalid_boundary_degree_vertices,
        "repeated_vertex_link_edge_vertices": repeated_link_edge_vertices,
        "max_vertex_link_components": max_link_components,
    }


def principal_axis(vertices: np.ndarray) -> np.ndarray:
    centered = vertices - np.mean(vertices, axis=0)
    covariance = np.dot(centered.T, centered) / max(1, len(vertices))
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    if axis[int(np.argmax(np.abs(axis)))] < 0.0:
        axis = -axis
    return axis


def parse_axis(value: str, vertices: np.ndarray) -> np.ndarray:
    if value == "pca":
        return principal_axis(vertices)
    named = {
        "x": np.asarray([1.0, 0.0, 0.0]),
        "y": np.asarray([0.0, 1.0, 0.0]),
        "z": np.asarray([0.0, 0.0, 1.0]),
    }
    if value in named:
        return named[value]
    components = np.asarray([float(item) for item in value.split(",")], dtype=np.float64)
    if components.shape != (3,) or not np.all(np.isfinite(components)):
        raise ValueError("--axis must be pca, x, y, z, or comma-separated ax,ay,az")
    norm = np.linalg.norm(components)
    if norm <= 0.0:
        raise ValueError("--axis vector must be non-zero")
    return components / norm


def triangle_plane_segment(
    triangle: np.ndarray,
    signed_distance: np.ndarray,
    epsilon: float,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    intersections: List[np.ndarray] = []
    for index in range(3):
        left = triangle[index]
        right = triangle[(index + 1) % 3]
        left_distance = signed_distance[index]
        right_distance = signed_distance[(index + 1) % 3]
        if abs(left_distance) <= epsilon:
            intersections.append(left)
        if left_distance * right_distance < -(epsilon * epsilon):
            alpha = left_distance / (left_distance - right_distance)
            intersections.append(left + alpha * (right - left))
    unique: List[np.ndarray] = []
    for point in intersections:
        if not any(np.linalg.norm(point - existing) <= epsilon for existing in unique):
            unique.append(point)
    if len(unique) < 2:
        return None
    if len(unique) == 2:
        return unique[0], unique[1]
    best = (unique[0], unique[1])
    best_length = np.linalg.norm(best[1] - best[0])
    for left_index, left in enumerate(unique):
        for right in unique[left_index + 1:]:
            length = np.linalg.norm(right - left)
            if length > best_length:
                best = (left, right)
                best_length = length
    return best


def cross_section_metrics(
    vertices: np.ndarray,
    faces: np.ndarray,
    axis: np.ndarray,
    station: float,
    weld_tolerance_m: float,
) -> Dict[str, object]:
    segments = []
    epsilon = weld_tolerance_m * 0.1
    for face in faces:
        triangle = vertices[face]
        signed_distance = np.dot(triangle, axis) - station
        if np.min(signed_distance) > epsilon or np.max(signed_distance) < -epsilon:
            continue
        segment = triangle_plane_segment(triangle, signed_distance, epsilon)
        if segment is not None and np.linalg.norm(segment[1] - segment[0]) > epsilon:
            segments.append(segment)
    if not segments:
        return {
            "station_m": station,
            "segments": 0,
            "components": 0,
            "closed_components": 0,
            "open_endpoints": 0,
            "closed": False,
            "section_length_m": 0.0,
        }
    endpoints = np.asarray([point for segment in segments for point in segment], dtype=np.float64)
    endpoint_ids, _ = quantized_vertex_ids(endpoints, weld_tolerance_m)
    graph: Dict[int, List[int]] = defaultdict(list)
    length = 0.0
    for index, segment in enumerate(segments):
        left = int(endpoint_ids[2 * index])
        right = int(endpoint_ids[2 * index + 1])
        if left == right:
            continue
        graph[left].append(right)
        graph[right].append(left)
        length += float(np.linalg.norm(segment[1] - segment[0]))
    components = []
    remaining = set(graph)
    while remaining:
        start = remaining.pop()
        nodes = [start]
        queue = deque([start])
        while queue:
            node = queue.popleft()
            for neighbor in graph[node]:
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    nodes.append(neighbor)
                    queue.append(neighbor)
        components.append(nodes)
    closed_components = sum(1 for component in components if all(len(graph[node]) == 2 for node in component))
    open_endpoints = sum(1 for neighbors in graph.values() if len(neighbors) == 1)
    branch_nodes = sum(1 for neighbors in graph.values() if len(neighbors) > 2)
    return {
        "station_m": station,
        "segments": len(segments),
        "components": len(components),
        "closed_components": closed_components,
        "open_endpoints": open_endpoints,
        "branch_nodes": branch_nodes,
        "closed": len(components) == 1 and closed_components == 1,
        "section_length_m": length,
    }


def analyze_mesh(
    raw_vertices: np.ndarray,
    raw_faces: np.ndarray,
    weld_tolerance_m: float,
    area_epsilon_m2: float,
    axis_specification: str,
    station_count: int,
    station_margin_fraction: float,
) -> Dict[str, object]:
    if raw_vertices.ndim != 2 or raw_vertices.shape[1:] != (3,):
        raise ValueError("vertices must be an Nx3 array")
    if raw_faces.ndim != 2 or raw_faces.shape[1:] != (3,):
        raise ValueError("faces must be an Mx3 triangle array")
    finite_vertex_mask = np.all(np.isfinite(raw_vertices), axis=1)
    valid_index_mask = np.all((raw_faces >= 0) & (raw_faces < len(raw_vertices)), axis=1)
    finite_face_mask = valid_index_mask.copy()
    if np.any(valid_index_mask):
        finite_face_mask[valid_index_mask] = np.all(finite_vertex_mask[raw_faces[valid_index_mask]], axis=1)
    finite_faces = raw_faces[finite_face_mask]
    vertex_ids, vertices = quantized_vertex_ids(raw_vertices, weld_tolerance_m)
    faces = vertex_ids[finite_faces]
    distinct_mask = (
        (faces[:, 0] != faces[:, 1])
        & (faces[:, 1] != faces[:, 2])
        & (faces[:, 2] != faces[:, 0])
    ) if len(faces) else np.empty((0,), dtype=bool)
    faces = faces[distinct_mask]
    areas = triangle_areas(vertices, faces)
    nondegenerate_mask = areas > area_epsilon_m2
    nondegenerate_faces = faces[nondegenerate_mask]
    nondegenerate_areas = areas[nondegenerate_mask]
    edge_counts, edge_owners = build_edge_incidence(nondegenerate_faces)
    link_metrics = vertex_link_metrics(nondegenerate_faces, edge_counts)
    boundary_edges = [edge for edge, count in edge_counts.items() if count == 1]
    nonmanifold_edges = [edge for edge, count in edge_counts.items() if count > 2]
    edge_lengths = {
        edge: float(np.linalg.norm(vertices[edge[1]] - vertices[edge[0]]))
        for edge in edge_counts
    }
    components = face_components(len(nondegenerate_faces), edge_owners)
    component_areas = [float(np.sum(nondegenerate_areas[component])) for component in components]
    total_area = float(np.sum(nondegenerate_areas))
    axis = parse_axis(axis_specification, vertices)
    projections = np.dot(vertices, axis) if len(vertices) else np.empty((0,))
    section_results = []
    if len(projections) and station_count > 0:
        lower = float(np.min(projections))
        upper = float(np.max(projections))
        margin = (upper - lower) * station_margin_fraction
        stations = np.linspace(lower + margin, upper - margin, station_count + 2)[1:-1]
        section_results = [
            cross_section_metrics(vertices, nondegenerate_faces, axis, float(station), weld_tolerance_m)
            for station in stations
        ]
    closed_sections = sum(1 for result in section_results if result["closed"])
    return {
        "input_vertices": int(len(raw_vertices)),
        "welded_vertices": int(len(vertices)),
        "input_triangles": int(len(raw_faces)),
        "finite_triangles": int(len(finite_faces)),
        "degenerate_triangles": int(len(finite_faces) - len(nondegenerate_faces)),
        "triangles": int(len(nondegenerate_faces)),
        "surface_area_m2": total_area,
        "unique_edges": len(edge_counts),
        "boundary_edges": len(boundary_edges),
        "boundary_edge_ratio": len(boundary_edges) / len(edge_counts) if edge_counts else None,
        "boundary_length_m": sum(edge_lengths[edge] for edge in boundary_edges),
        "nonmanifold_edges": len(nonmanifold_edges),
        **link_metrics,
        "connected_components": len(components),
        "largest_component_area_ratio": max(component_areas) / total_area if total_area > 0.0 else None,
        "watertight": bool(edge_counts)
        and not boundary_edges
        and not nonmanifold_edges
        and link_metrics["nonmanifold_vertices"] == 0,
        "longitudinal_axis": axis.tolist(),
        "cross_sections": section_results,
        "closed_cross_section_ratio": closed_sections / len(section_results) if section_results else None,
        "parameters": {
            "weld_tolerance_m": weld_tolerance_m,
            "area_epsilon_m2": area_epsilon_m2,
            "axis": axis_specification,
            "station_count": station_count,
            "station_margin_fraction": station_margin_fraction,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--obj", type=Path, help="Wavefront OBJ input")
    source.add_argument("--bag", type=Path, help="ROS bag containing Marker TRIANGLE_LIST")
    parser.add_argument("--topic", default="/local_tsdf_mesh/mesh")
    parser.add_argument("--marker-index", type=int, default=-1, help="zero-based marker; -1 selects the last")
    parser.add_argument("--weld", type=float, default=0.005, help="vertex/section endpoint weld tolerance, m")
    parser.add_argument("--area-epsilon", type=float, default=1e-10, help="degenerate triangle area, m^2")
    parser.add_argument("--axis", default="pca", help="pca, x, y, z, or ax,ay,az")
    parser.add_argument("--stations", type=int, default=9, help="number of interior cross-sections")
    parser.add_argument("--station-margin", type=float, default=0.05, help="fraction omitted at each end")
    parser.add_argument("--output", type=Path, help="optional JSON output")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.weld <= 0.0 or args.area_epsilon < 0.0:
        raise SystemExit("--weld must be positive and --area-epsilon must be non-negative")
    if args.stations < 0 or not 0.0 <= args.station_margin < 0.5:
        raise SystemExit("--stations must be non-negative and --station-margin must be in [0, 0.5)")
    metadata: Dict[str, object] = {}
    if args.obj:
        vertices, faces = load_obj(args.obj.resolve())
        metadata["source"] = str(args.obj.resolve())
        metadata["source_type"] = "obj"
    else:
        vertices, faces, bag_metadata = load_marker_bag(
            args.bag.resolve(), args.topic, args.marker_index
        )
        metadata.update(bag_metadata)
        metadata["source"] = str(args.bag.resolve())
        metadata["source_type"] = "ros_marker_bag"
    result = analyze_mesh(
        vertices,
        faces,
        args.weld,
        args.area_epsilon,
        args.axis,
        args.stations,
        args.station_margin,
    )
    result.update(metadata)
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
