#!/usr/bin/env python3
"""Evaluate local TSDF coverage, topology, stability, and callback budget.

The input bag must contain synchronized fused PointCloud2, TSDF diagnostics,
and full TRIANGLE_LIST Marker snapshots.  The checks intentionally mirror the
field acceptance gates instead of treating triangle count as surface quality.
"""

import argparse
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from mesh_topology_metrics import (
    analyze_mesh,
    build_edge_incidence,
    face_components,
    load_marker_bag,
    quantize_coordinates,
    quantized_vertex_ids,
    triangle_areas,
)
from tsdf_bag_metrics import analyze as analyze_runtime


def parse_origins(specification: str) -> List[np.ndarray]:
    origins: List[np.ndarray] = []
    for item in specification.split(";"):
        values = np.asarray([float(value) for value in item.split(",")], dtype=np.float64)
        if values.shape != (3,) or not np.all(np.isfinite(values)):
            raise ValueError("--sensor-origins must be x,y,z;x,y,z;...")
        origins.append(values)
    if not origins:
        raise ValueError("at least one sensor origin is required")
    return origins


def message_stamp(message, bag_stamp) -> float:
    stamp = message.header.stamp.to_sec()
    return stamp if stamp > 0.0 else bag_stamp.to_sec()


def nearest_cloud(
    bag_path: Path,
    topic: str,
    target_stamp_s: float,
    origins: Sequence[np.ndarray],
) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
    try:
        import rosbag  # pylint: disable=import-outside-toplevel
        from sensor_msgs import point_cloud2  # pylint: disable=import-outside-toplevel
    except ImportError as error:
        raise RuntimeError("ROS Python is required to read the acceptance bag") from error

    best_message = None
    best_stamp = 0.0
    best_delta = math.inf
    with rosbag.Bag(str(bag_path), "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        if topic not in topic_info:
            raise RuntimeError(f"cloud topic is absent: {topic}")
        for _, message, bag_stamp in bag.read_messages(topics=[topic]):
            stamp = message_stamp(message, bag_stamp)
            delta = abs(stamp - target_stamp_s)
            if delta < best_delta:
                best_message = message
                best_stamp = stamp
                best_delta = delta
    if best_message is None:
        raise RuntimeError(f"no PointCloud2 found on {topic}")

    available_fields = {field.name for field in best_message.fields}
    missing = {"x", "y", "z"} - available_fields
    if missing:
        raise RuntimeError(f"PointCloud2 is missing fields: {sorted(missing)}")
    has_lidar_id = "lidar_id" in available_fields
    field_names = ("x", "y", "z", "lidar_id") if has_lidar_id else ("x", "y", "z")
    positions: List[Tuple[float, float, float]] = []
    ranges: List[float] = []
    invalid_points = 0
    unknown_origins = 0
    for values in point_cloud2.read_points(
        best_message, field_names=field_names, skip_nans=False
    ):
        x, y, z = (float(values[0]), float(values[1]), float(values[2]))
        if not all(math.isfinite(value) for value in (x, y, z)):
            invalid_points += 1
            continue
        lidar_id = int(values[3]) if has_lidar_id else 0
        if lidar_id < 0 or lidar_id >= len(origins):
            unknown_origins += 1
            continue
        point = np.asarray((x, y, z), dtype=np.float64)
        positions.append((x, y, z))
        ranges.append(float(np.linalg.norm(point - origins[lidar_id])))
    return (
        np.asarray(positions, dtype=np.float64),
        np.asarray(ranges, dtype=np.float64),
        {
            "topic": topic,
            "header_stamp_s": best_stamp,
            "target_stamp_s": target_stamp_s,
            "stamp_delta_ms": best_delta * 1000.0,
            "frame_id": best_message.header.frame_id,
            "input_points": int(best_message.width) * int(best_message.height),
            "finite_configured_points": len(positions),
            "invalid_points": invalid_points,
            "unknown_sensor_origins": unknown_origins,
            "has_lidar_id": has_lidar_id,
        },
    )


def surface_samples(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """Sample each triangle densely enough for a 15 cm coverage query.

    Marching-tetra triangles are bounded by the voxel diagonal.  Vertices,
    edge midpoints, and centroids therefore cover every valid, non-large face
    without the cost of a general triangle-distance acceleration structure.
    Long faces are checked independently and cannot silently pass acceptance.
    """
    if not len(faces):
        return np.empty((0, 3), dtype=np.float64)
    triangles = vertices[faces]
    samples = np.concatenate(
        (
            triangles[:, 0],
            triangles[:, 1],
            triangles[:, 2],
            0.5 * (triangles[:, 0] + triangles[:, 1]),
            0.5 * (triangles[:, 1] + triangles[:, 2]),
            0.5 * (triangles[:, 2] + triangles[:, 0]),
            np.mean(triangles, axis=1),
        ),
        axis=0,
    )
    return samples[np.all(np.isfinite(samples), axis=1)]


def sampled_surface_distance(
    points: np.ndarray,
    samples: np.ndarray,
    threshold_m: float,
) -> np.ndarray:
    if not len(points):
        return np.empty((0,), dtype=np.float64)
    if not len(samples):
        return np.full((len(points),), math.inf, dtype=np.float64)

    cell_size = threshold_m
    surface_cells: Dict[Tuple[int, int, int], List[np.ndarray]] = defaultdict(list)
    # Remove repeat samples before building Python buckets.  A quarter-threshold
    # key preserves enough detail while making duplicated TRIANGLE_LIST cheap.
    sample_quantum = threshold_m * 0.25
    quantized_samples = quantize_coordinates(samples, sample_quantum)
    _, unique_indices = np.unique(quantized_samples, axis=0, return_index=True)
    samples = samples[np.sort(unique_indices)]
    sample_keys = quantize_coordinates(samples, cell_size)
    for sample, raw_key in zip(samples, sample_keys):
        key = tuple(int(value) for value in raw_key)
        surface_cells[key].append(sample)
    packed_surface_cells = {
        key: np.asarray(values, dtype=np.float64)
        for key, values in surface_cells.items()
    }

    point_cells: Dict[Tuple[int, int, int], List[int]] = defaultdict(list)
    point_keys = quantize_coordinates(points, cell_size)
    for index, raw_key in enumerate(point_keys):
        key = tuple(int(value) for value in raw_key)
        point_cells[key].append(index)

    squared_limit = threshold_m * threshold_m
    distances = np.full((len(points),), math.inf, dtype=np.float64)
    neighbor_offsets = [
        (x, y, z)
        for x in (-1, 0, 1)
        for y in (-1, 0, 1)
        for z in (-1, 0, 1)
    ]
    for key, point_indices in point_cells.items():
        candidate_parts = []
        for offset in neighbor_offsets:
            neighbor = (key[0] + offset[0], key[1] + offset[1], key[2] + offset[2])
            candidate = packed_surface_cells.get(neighbor)
            if candidate is not None:
                candidate_parts.append(candidate)
        if not candidate_parts:
            continue
        candidates = np.concatenate(candidate_parts, axis=0)
        selected_points = points[np.asarray(point_indices, dtype=np.int64)]
        # Bound temporary arrays even when a dense wall falls in one cell.
        for start in range(0, len(selected_points), 512):
            chunk = selected_points[start : start + 512]
            squared = np.sum(
                (chunk[:, None, :] - candidates[None, :, :]) ** 2,
                axis=2,
            )
            minimum = np.min(squared, axis=1)
            output_indices = point_indices[start : start + len(chunk)]
            distances[np.asarray(output_indices, dtype=np.int64)] = np.sqrt(minimum)
    # Values above the requested radius remain useful for debugging only if a
    # neighboring bucket was present; acceptance still compares to the limit.
    distances[distances * distances > squared_limit * 16.0] = math.inf
    return distances


def estimate_tunnel_axis(
    points: np.ndarray,
    quantization_m: float,
    min_span_m: float,
    min_eigenvalue_ratio: float,
) -> Dict[str, object]:
    """Estimate the tunnel axis and report whether the PCA result is usable."""
    points = np.asarray(points, dtype=np.float64)
    finite = points[np.all(np.isfinite(points), axis=1)]
    if len(finite) < 3:
        return {
            "axis": None,
            "confident": False,
            "reason": "fewer_than_three_finite_points",
            "span_m": None,
            "eigenvalues": None,
            "largest_to_second_ratio": None,
        }
    keys = quantize_coordinates(finite, quantization_m)
    _, unique_indices = np.unique(keys, axis=0, return_index=True)
    unique = finite[np.sort(unique_indices)]
    if len(unique) < 3:
        return {
            "axis": None,
            "confident": False,
            "reason": "fewer_than_three_unique_points",
            "span_m": None,
            "eigenvalues": None,
            "largest_to_second_ratio": None,
        }
    covariance = np.cov(unique, rowvar=False, bias=True)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)
    eigenvalues = eigenvalues[order]
    axis = eigenvectors[:, order[-1]]
    # Eigenvector signs are arbitrary.  Canonicalize the result so reports and
    # synthetic regressions are deterministic.
    dominant = int(np.argmax(np.abs(axis)))
    if axis[dominant] < 0.0:
        axis = -axis
    projections = unique @ axis
    span = float(np.max(projections) - np.min(projections))
    second = float(eigenvalues[-2])
    largest = float(eigenvalues[-1])
    ratio = largest / second if second > 0.0 else math.inf
    confident = bool(span >= min_span_m and ratio >= min_eigenvalue_ratio)
    reasons = []
    if span < min_span_m:
        reasons.append("span_below_minimum")
    if ratio < min_eigenvalue_ratio:
        reasons.append("eigenvalue_gap_below_minimum")
    return {
        "axis": axis,
        "confident": confident,
        "reason": "accepted" if confident else "+".join(reasons),
        "span_m": span,
        "eigenvalues": eigenvalues.tolist(),
        "largest_to_second_ratio": ratio,
    }


def infer_tunnel_axis(points: np.ndarray, quantization_m: float) -> np.ndarray:
    """Compatibility helper returning the unconstrained largest PCA axis."""
    estimate = estimate_tunnel_axis(points, quantization_m, 0.0, 0.0)
    axis = estimate["axis"]
    if axis is None:
        raise ValueError(str(estimate["reason"]))
    return np.asarray(axis, dtype=np.float64)


def face_adjacency_patches(
    faces: np.ndarray,
    selected_faces: Sequence[int],
    normals: Optional[np.ndarray] = None,
    minimum_normal_cosine: float = -1.0,
) -> List[List[int]]:
    """Partition selected faces through shared edges and an optional normal gate."""
    selected = sorted(int(index) for index in selected_faces)
    selected_set = set(selected)
    edge_owners: Dict[Tuple[int, int], List[int]] = defaultdict(list)
    for face_index in selected:
        face = faces[face_index]
        for first, second in (
            (face[0], face[1]),
            (face[1], face[2]),
            (face[2], face[0]),
        ):
            edge = (int(min(first, second)), int(max(first, second)))
            edge_owners[edge].append(face_index)
    adjacency: Dict[int, List[int]] = {index: [] for index in selected}
    for owners in edge_owners.values():
        for owner_index, left in enumerate(owners):
            for right in owners[owner_index + 1 :]:
                if left not in selected_set or right not in selected_set:
                    continue
                if normals is not None:
                    alignment = float(abs(np.dot(normals[left], normals[right])))
                    if alignment < minimum_normal_cosine:
                        continue
                adjacency[left].append(right)
                adjacency[right].append(left)
    patches: List[List[int]] = []
    unvisited = set(selected)
    while unvisited:
        start = min(unvisited)
        unvisited.remove(start)
        patch = [start]
        pending = [start]
        while pending:
            current = pending.pop()
            for neighbor in adjacency[current]:
                if neighbor in unvisited:
                    unvisited.remove(neighbor)
                    patch.append(neighbor)
                    pending.append(neighbor)
        patches.append(sorted(patch))
    return patches


def normalized_face_normals(vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    triangles = vertices[faces]
    raw = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(raw, axis=1)
    normals = np.zeros_like(raw)
    valid = lengths > 0.0
    normals[valid] = raw[valid] / lengths[valid, None]
    return normals


def component_surface_samples(
    vertices: np.ndarray,
    faces: np.ndarray,
    areas: np.ndarray,
    component: Sequence[int],
) -> Tuple[np.ndarray, np.ndarray]:
    """Return surface samples and per-sample area weights for one component."""
    selected = np.asarray(component, dtype=np.int64)
    triangles = vertices[faces[selected]]
    samples = np.stack(
        (
            triangles[:, 0],
            triangles[:, 1],
            triangles[:, 2],
            0.5 * (triangles[:, 0] + triangles[:, 1]),
            0.5 * (triangles[:, 1] + triangles[:, 2]),
            0.5 * (triangles[:, 2] + triangles[:, 0]),
            np.mean(triangles, axis=1),
        ),
        axis=1,
    )
    weights = np.repeat(
        (areas[selected] / samples.shape[1])[:, None],
        samples.shape[1],
        axis=1,
    )
    return samples.reshape((-1, 3)), weights.reshape((-1,))


def weighted_component_pca(
    samples: np.ndarray,
    weights: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    total_weight = float(np.sum(weights))
    if total_weight <= 0.0:
        raise ValueError("component PCA needs positive surface area")
    centroid = np.sum(samples * weights[:, None], axis=0) / total_weight
    centered = samples - centroid
    covariance = (centered * weights[:, None]).T @ centered / total_weight
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)
    eigenvalues = eigenvalues[order]
    eigenvectors = eigenvectors[:, order]
    projections = centered @ eigenvectors
    spans = np.max(projections, axis=0) - np.min(projections, axis=0)
    normal = eigenvectors[:, 0]
    dominant = int(np.argmax(np.abs(normal)))
    if normal[dominant] < 0.0:
        normal = -normal
    return centroid, eigenvalues, spans, normal


def analyze_component_safety(
    vertices: np.ndarray,
    faces: np.ndarray,
    areas: np.ndarray,
    components: Sequence[Sequence[int]],
    observed_points: np.ndarray,
    support_distance_m: float = 0.15,
    min_component_area_m2: float = 1.0,
    min_reverse_precision: float = 0.90,
    tunnel_axis: Optional[np.ndarray] = None,
    axis_normal_cosine: float = 0.85,
    max_planar_thickness_m: float = 0.15,
    max_planar_thickness_ratio: float = 0.10,
    min_curtain_span_m: float = 1.0,
    min_face_reverse_precision: float = 0.50,
    max_unsupported_patch_area_m2: float = 0.25,
    max_unsupported_patch_span_m: float = 1.0,
    max_total_unsupported_area_m2: float = 1.0,
    planar_patch_normal_cosine: float = 0.95,
    min_tunnel_axis_span_m: float = 6.0,
    min_tunnel_axis_eigenvalue_ratio: float = 1.25,
) -> Dict[str, object]:
    """Validate large mesh components against observations and tunnel geometry.

    Disconnected walls are valid when their area is supported by the observed
    cloud.  A thin plane whose normal follows the tunnel axis and which spans
    both transverse directions is an explicit cross-tunnel curtain failure,
    even if points near its perimeter make its raw reverse precision look good.
    """
    vertices = np.asarray(vertices, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    areas = np.asarray(areas, dtype=np.float64)
    observed_points = np.asarray(observed_points, dtype=np.float64)
    if support_distance_m <= 0.0 or min_component_area_m2 < 0.0:
        raise ValueError("component safety distances and areas must be positive")
    if not 0.0 <= min_reverse_precision <= 1.0 or not 0.0 <= min_face_reverse_precision <= 1.0:
        raise ValueError("reverse precision thresholds must be in [0, 1]")
    if not 0.0 <= axis_normal_cosine <= 1.0 or not 0.0 <= planar_patch_normal_cosine <= 1.0:
        raise ValueError("normal cosine thresholds must be in [0, 1]")
    if (
        max_planar_thickness_m < 0.0
        or max_planar_thickness_ratio < 0.0
        or min_curtain_span_m < 0.0
        or max_unsupported_patch_area_m2 < 0.0
        or max_unsupported_patch_span_m < 0.0
        or max_total_unsupported_area_m2 < 0.0
        or min_tunnel_axis_span_m < 0.0
        or min_tunnel_axis_eigenvalue_ratio < 1.0
    ):
        raise ValueError("component safety geometry thresholds are invalid")
    if tunnel_axis is None:
        estimate = estimate_tunnel_axis(
            observed_points,
            support_distance_m * 0.25,
            min_tunnel_axis_span_m,
            min_tunnel_axis_eigenvalue_ratio,
        )
        candidate_axis = estimate.pop("axis")
        tunnel_axis = (
            np.asarray(candidate_axis, dtype=np.float64)
            if bool(estimate["confident"]) and candidate_axis is not None
            else None
        )
        axis_status = "auto_pca" if tunnel_axis is not None else "axis_unknown"
        axis_confidence = {
            **estimate,
            "candidate_axis": (
                np.asarray(candidate_axis, dtype=np.float64).tolist()
                if candidate_axis is not None
                else None
            ),
            "minimum_span_m": min_tunnel_axis_span_m,
            "minimum_eigenvalue_ratio": min_tunnel_axis_eigenvalue_ratio,
        }
    else:
        tunnel_axis = np.asarray(tunnel_axis, dtype=np.float64)
        axis_norm = float(np.linalg.norm(tunnel_axis))
        if tunnel_axis.shape != (3,) or not math.isfinite(axis_norm) or axis_norm <= 0.0:
            raise ValueError("tunnel axis must be a finite non-zero xyz vector")
        tunnel_axis = tunnel_axis / axis_norm
        axis_status = "explicit"
        axis_confidence = {
            "confident": True,
            "reason": "explicit_axis",
            "candidate_axis": tunnel_axis.tolist(),
            "span_m": None,
            "eigenvalues": None,
            "largest_to_second_ratio": None,
            "minimum_span_m": min_tunnel_axis_span_m,
            "minimum_eigenvalue_ratio": min_tunnel_axis_eigenvalue_ratio,
        }

    reports: List[Dict[str, object]] = []
    unsupported_large: List[int] = []
    curtains: List[int] = []
    unsafe_patch_components: List[int] = []
    curtain_patches: List[Dict[str, object]] = []
    total_unsupported_area = 0.0
    face_normals = normalized_face_normals(vertices, faces)
    for component_index, component in enumerate(components):
        samples, weights = component_surface_samples(vertices, faces, areas, component)
        component_area = float(np.sum(weights))
        distances = sampled_surface_distance(samples, observed_points, support_distance_m)
        supported = distances <= support_distance_m
        supported_area = float(np.sum(weights[supported]))
        unsupported_area = float(np.sum(weights[~supported]))
        total_unsupported_area += unsupported_area
        precision = supported_area / component_area if component_area > 0.0 else 0.0
        centroid, eigenvalues, spans, normal = weighted_component_pca(samples, weights)
        # PCA axes are ordered from least to greatest variance.  Thickness is
        # measured specifically along the least-variance/main-normal axis.
        thickness = float(spans[0])
        middle_span = float(np.min(spans[1:]))
        thickness_ratio = thickness / middle_span if middle_span > 0.0 else math.inf
        planar_thin = (
            thickness <= max_planar_thickness_m
            and thickness_ratio <= max_planar_thickness_ratio
        )
        normal_alignment = (
            float(abs(np.dot(normal, tunnel_axis))) if tunnel_axis is not None else None
        )
        is_large = component_area >= min_component_area_m2
        if is_large and precision < min_reverse_precision:
            unsupported_large.append(component_index)

        sample_count = 7
        component_faces = [int(index) for index in component]
        supported_by_face = supported.reshape((-1, sample_count))
        weights_by_face = weights.reshape((-1, sample_count))
        face_precision = np.mean(supported_by_face, axis=1)
        face_unsupported_area = np.sum(
            weights_by_face * (~supported_by_face), axis=1
        )
        unsupported_faces = [
            component_faces[index]
            for index, value in enumerate(face_precision)
            if float(value) < min_face_reverse_precision
        ]
        unsupported_patches = []
        if is_large and unsupported_faces:
            local_index = {
                face_index: index for index, face_index in enumerate(component_faces)
            }
            for patch_index, patch in enumerate(
                face_adjacency_patches(faces, unsupported_faces)
            ):
                positions = [local_index[face_index] for face_index in patch]
                patch_vertices = vertices[np.unique(faces[np.asarray(patch, dtype=np.int64)])]
                patch_span = float(
                    np.linalg.norm(
                        np.max(patch_vertices, axis=0) - np.min(patch_vertices, axis=0)
                    )
                )
                patch_unsupported_area = float(np.sum(face_unsupported_area[positions]))
                patch_face_area = float(np.sum(areas[np.asarray(patch, dtype=np.int64)]))
                exceeds_area = patch_unsupported_area > max_unsupported_patch_area_m2
                exceeds_span = patch_span > max_unsupported_patch_span_m
                unsafe = bool(exceeds_area or exceeds_span)
                unsupported_patches.append(
                    {
                        "index": patch_index,
                        "faces": len(patch),
                        "face_area_m2": patch_face_area,
                        "unsupported_area_m2": patch_unsupported_area,
                        "span_m": patch_span,
                        "unsafe": unsafe,
                        "exceeds_area": bool(exceeds_area),
                        "exceeds_span": bool(exceeds_span),
                    }
                )
                if unsafe and component_index not in unsafe_patch_components:
                    unsafe_patch_components.append(component_index)

        component_curtain_patches = []
        for planar_patch in face_adjacency_patches(
            faces,
            component_faces,
            normals=face_normals,
            minimum_normal_cosine=planar_patch_normal_cosine,
        ):
            patch_area = float(np.sum(areas[np.asarray(planar_patch, dtype=np.int64)]))
            if patch_area < min_component_area_m2:
                continue
            patch_samples, patch_weights = component_surface_samples(
                vertices, faces, areas, planar_patch
            )
            patch_centroid, patch_eigenvalues, patch_spans, patch_normal = (
                weighted_component_pca(patch_samples, patch_weights)
            )
            patch_thickness = float(patch_spans[0])
            patch_middle_span = float(np.min(patch_spans[1:]))
            patch_thickness_ratio = (
                patch_thickness / patch_middle_span
                if patch_middle_span > 0.0
                else math.inf
            )
            patch_planar_thin = bool(
                patch_thickness <= max_planar_thickness_m
                and patch_thickness_ratio <= max_planar_thickness_ratio
            )
            patch_alignment = (
                float(abs(np.dot(patch_normal, tunnel_axis)))
                if tunnel_axis is not None
                else None
            )
            patch_is_curtain = bool(
                tunnel_axis is not None
                and patch_planar_thin
                and patch_alignment is not None
                and patch_alignment >= axis_normal_cosine
                and bool(np.all(np.sort(patch_spans[1:]) >= min_curtain_span_m))
            )
            patch_report = {
                "component": component_index,
                "faces": len(planar_patch),
                "area_m2": patch_area,
                "centroid": patch_centroid.tolist(),
                "eigenvalues": patch_eigenvalues.tolist(),
                "spans_m": patch_spans.tolist(),
                "main_normal": patch_normal.tolist(),
                "normal_tunnel_axis_abs_dot": patch_alignment,
                "thickness_m": patch_thickness,
                "thickness_ratio": patch_thickness_ratio,
                "planar_thin": patch_planar_thin,
                "cross_tunnel_curtain": patch_is_curtain,
            }
            component_curtain_patches.append(patch_report)
            if patch_is_curtain:
                curtain_patches.append(patch_report)
                if component_index not in curtains:
                    curtains.append(component_index)

        is_curtain = component_index in curtains

        finite_distances = distances[np.isfinite(distances)]
        longitudinal = samples @ tunnel_axis if tunnel_axis is not None else None
        radial = np.linalg.norm(samples, axis=1)
        reports.append(
            {
                "index": component_index,
                "faces": len(component),
                "area_m2": component_area,
                "is_large": is_large,
                "reverse_precision": precision,
                "observed_supported_area_m2": supported_area,
                "observed_unsupported_area_m2": unsupported_area,
                "observed_support_distance_m": support_distance_m,
                "unsupported_patches": unsupported_patches,
                "planar_patches": component_curtain_patches,
                "observed_distance_m": {
                    "finite_min": float(np.min(finite_distances)) if len(finite_distances) else None,
                    "finite_p50": float(np.percentile(finite_distances, 50)) if len(finite_distances) else None,
                    "finite_p95": float(np.percentile(finite_distances, 95)) if len(finite_distances) else None,
                    "unsupported_samples": int(np.count_nonzero(~supported)),
                },
                "bounds_m": {
                    "min_xyz": np.min(samples, axis=0).tolist(),
                    "max_xyz": np.max(samples, axis=0).tolist(),
                    "longitudinal": (
                        [float(np.min(longitudinal)), float(np.max(longitudinal))]
                        if longitudinal is not None
                        else None
                    ),
                    "origin_range": [float(np.min(radial)), float(np.max(radial))],
                },
                "pca": {
                    "centroid": centroid.tolist(),
                    "eigenvalues": eigenvalues.tolist(),
                    "spans_m": spans.tolist(),
                    "main_normal": normal.tolist(),
                    "normal_tunnel_axis_abs_dot": normal_alignment,
                    "thickness_m": thickness,
                    "thickness_ratio": thickness_ratio,
                    "planar_thin": bool(planar_thin),
                },
                "cross_tunnel_curtain": is_curtain,
            }
        )
    return {
        "tunnel_axis": tunnel_axis.tolist() if tunnel_axis is not None else None,
        "tunnel_axis_status": axis_status,
        "tunnel_axis_confidence": axis_confidence,
        "component_count": len(reports),
        "large_component_count": sum(bool(report["is_large"]) for report in reports),
        "small_component_count": sum(not bool(report["is_large"]) for report in reports),
        "unsupported_large_components": unsupported_large,
        "unsupported_patch_components": unsafe_patch_components,
        "total_unsupported_area_m2": total_unsupported_area,
        "max_total_unsupported_area_m2": max_total_unsupported_area_m2,
        "total_unsupported_area_exceeded": bool(
            total_unsupported_area > max_total_unsupported_area_m2
        ),
        "cross_tunnel_curtain_components": curtains,
        "cross_tunnel_curtain_patches": curtain_patches,
        "components": reports,
        "passed": bool(
            not unsupported_large
            and not unsafe_patch_components
            and total_unsupported_area <= max_total_unsupported_area_m2
            and not curtains
        ),
    }


def prepare_topology(
    raw_vertices: np.ndarray,
    raw_faces: np.ndarray,
    weld_m: float,
    area_epsilon_m2: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, Counter, List[List[int]]]:
    finite_vertices = np.all(np.isfinite(raw_vertices), axis=1)
    valid_indices = np.all((raw_faces >= 0) & (raw_faces < len(raw_vertices)), axis=1)
    valid_faces = raw_faces[valid_indices]
    if len(valid_faces):
        valid_faces = valid_faces[np.all(finite_vertices[valid_faces], axis=1)]
    inverse, vertices = quantized_vertex_ids(raw_vertices, weld_m)
    faces = inverse[valid_faces]
    if len(faces):
        distinct = (
            (faces[:, 0] != faces[:, 1])
            & (faces[:, 1] != faces[:, 2])
            & (faces[:, 2] != faces[:, 0])
        )
        faces = faces[distinct]
    areas = triangle_areas(vertices, faces)
    faces = faces[areas > area_epsilon_m2]
    areas = areas[areas > area_epsilon_m2]
    edge_counts, edge_owners = build_edge_incidence(faces)
    components = face_components(len(faces), edge_owners)
    return vertices, faces, areas, edge_counts, components


def bucket_coverage(
    ranges: np.ndarray,
    distances: np.ndarray,
    lower_m: float,
    upper_m: float,
    threshold_m: float,
) -> Dict[str, object]:
    mask = (
        np.isfinite(ranges)
        & (ranges >= lower_m)
        & (ranges < upper_m)
    )
    count = int(np.count_nonzero(mask))
    covered = int(np.count_nonzero(mask & (distances <= threshold_m)))
    return {
        "range_m": [lower_m, upper_m],
        "points": count,
        "covered_points": covered,
        "coverage": covered / count if count else None,
        "threshold_m": threshold_m,
    }


def add_check(
    checks: List[Dict[str, object]],
    name: str,
    passed: bool,
    observed,
    expectation: str,
) -> None:
    checks.append(
        {
            "name": name,
            "passed": bool(passed),
            "observed": observed,
            "expectation": expectation,
        }
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--cloud-topic", default="/points_raw")
    parser.add_argument("--status-topic", default="/local_tsdf_mesh/status")
    parser.add_argument("--mesh-topic", default="/local_tsdf_mesh/mesh")
    parser.add_argument("--marker-index", type=int, default=-1)
    parser.add_argument("--messages", type=int, default=30)
    parser.add_argument("--sensor-origins", default="0,0,0;0.323744,-0.00124153,-0.200876")
    parser.add_argument("--coverage-distance", type=float, default=0.15)
    parser.add_argument(
        "--reverse-support-distance",
        type=float,
        default=0.15,
        help="mesh-to-observed-cloud support radius; acceptance forbids values above 0.15 m",
    )
    parser.add_argument("--near-range", default="0.5,10")
    parser.add_argument("--far-range", default="10,20")
    parser.add_argument("--near-min-coverage", type=float, default=0.90)
    parser.add_argument("--far-min-coverage", type=float, default=0.70)
    parser.add_argument("--max-boundary-ratio", type=float, default=0.10)
    parser.add_argument("--max-nonmanifold", type=int, default=0)
    parser.add_argument("--max-callback-p95-ms", type=float, default=100.0)
    parser.add_argument("--max-edge-m", type=float, default=0.45)
    parser.add_argument("--max-sync-ms", type=float, default=20.0)
    parser.add_argument(
        "--min-safety-component-area-m2",
        "--max-isolated-component-area-m2",
        dest="min_safety_component_area_m2",
        type=float,
        default=1.0,
        help="minimum component area subject to reverse-support safety checks",
    )
    parser.add_argument("--min-reverse-precision", type=float, default=0.90)
    parser.add_argument("--min-face-reverse-precision", type=float, default=0.50)
    parser.add_argument("--max-unsupported-patch-area-m2", type=float, default=0.25)
    parser.add_argument("--max-unsupported-patch-span-m", type=float, default=1.0)
    parser.add_argument("--max-total-unsupported-area-m2", type=float, default=1.0)
    parser.add_argument(
        "--tunnel-axis",
        help="explicit tunnel longitudinal axis x,y,z in the cloud/Marker frame",
    )
    parser.add_argument("--min-tunnel-axis-span-m", type=float, default=6.0)
    parser.add_argument(
        "--min-tunnel-axis-eigenvalue-ratio", type=float, default=1.25
    )
    parser.add_argument("--curtain-axis-normal-cosine", type=float, default=0.85)
    parser.add_argument("--planar-patch-normal-cosine", type=float, default=0.95)
    parser.add_argument("--max-planar-thickness-m", type=float, default=0.15)
    parser.add_argument("--max-planar-thickness-ratio", type=float, default=0.10)
    parser.add_argument("--min-curtain-span-m", type=float, default=1.0)
    parser.add_argument("--weld", type=float, default=0.0001)
    parser.add_argument("--area-epsilon", type=float, default=1e-10)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def parse_range(value: str, name: str) -> Tuple[float, float]:
    items = [float(item) for item in value.split(",")]
    if len(items) != 2 or not all(math.isfinite(item) for item in items) or items[0] >= items[1]:
        raise ValueError(f"{name} must be lower,upper")
    return items[0], items[1]


def parse_axis(value: Optional[str]) -> Optional[np.ndarray]:
    if value is None:
        return None
    items = np.asarray([float(item) for item in value.split(",")], dtype=np.float64)
    norm = float(np.linalg.norm(items)) if items.shape == (3,) else math.nan
    if items.shape != (3,) or not np.all(np.isfinite(items)) or not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("--tunnel-axis must be a finite non-zero x,y,z vector")
    return items / norm


def frame_ids_match(marker_frame: object, cloud_frame: object) -> bool:
    """Require a non-empty exact frame match before comparing coordinates."""
    marker = str(marker_frame)
    cloud = str(cloud_frame)
    return bool(marker and cloud and marker == cloud)


def main() -> int:
    args = parse_args()
    if args.messages <= 1:
        raise SystemExit("--messages must be greater than one")
    if args.coverage_distance <= 0.0 or args.weld <= 0.0:
        raise SystemExit("coverage distance and weld tolerance must be positive")
    if not 0.0 < args.reverse_support_distance <= 0.15:
        raise SystemExit("reverse support distance must be in (0, 0.15] m")
    if args.min_safety_component_area_m2 < 0.0:
        raise SystemExit("component safety area must be non-negative")
    if not 0.0 <= args.min_reverse_precision <= 1.0 or not 0.0 <= args.min_face_reverse_precision <= 1.0:
        raise SystemExit("reverse precision thresholds must be in [0, 1]")
    if (
        not 0.0 <= args.curtain_axis_normal_cosine <= 1.0
        or not 0.0 <= args.planar_patch_normal_cosine <= 1.0
    ):
        raise SystemExit("normal cosine thresholds must be in [0, 1]")
    if (
        args.max_unsupported_patch_area_m2 < 0.0
        or args.max_unsupported_patch_span_m < 0.0
        or args.max_total_unsupported_area_m2 < 0.0
        or args.min_tunnel_axis_span_m < 0.0
        or args.min_tunnel_axis_eigenvalue_ratio < 1.0
        or args.max_planar_thickness_m < 0.0
        or args.max_planar_thickness_ratio < 0.0
        or args.min_curtain_span_m < 0.0
    ):
        raise SystemExit("component safety thresholds are invalid")
    bag_path = args.bag.resolve()
    origins = parse_origins(args.sensor_origins)
    try:
        explicit_tunnel_axis = parse_axis(args.tunnel_axis)
    except ValueError as error:
        raise SystemExit(str(error)) from error
    near_range = parse_range(args.near_range, "--near-range")
    far_range = parse_range(args.far_range, "--far-range")

    raw_vertices, raw_faces, marker_metadata = load_marker_bag(
        bag_path, args.mesh_topic, args.marker_index
    )
    cloud_points, cloud_ranges, cloud_metadata = nearest_cloud(
        bag_path,
        args.cloud_topic,
        float(marker_metadata["header_stamp_s"]),
        origins,
    )
    vertices, faces, areas, edge_counts, components = prepare_topology(
        raw_vertices, raw_faces, args.weld, args.area_epsilon
    )
    samples = surface_samples(vertices, faces)
    distances = sampled_surface_distance(
        cloud_points, samples, args.coverage_distance
    )
    near_coverage = bucket_coverage(
        cloud_ranges, distances, near_range[0], near_range[1], args.coverage_distance
    )
    far_coverage = bucket_coverage(
        cloud_ranges, distances, far_range[0], far_range[1], args.coverage_distance
    )

    topology = analyze_mesh(
        raw_vertices,
        raw_faces,
        args.weld,
        args.area_epsilon,
        "pca",
        0,
        0.05,
    )
    edge_lengths = np.asarray(
        [np.linalg.norm(vertices[right] - vertices[left]) for left, right in edge_counts],
        dtype=np.float64,
    )
    component_areas = [float(np.sum(areas[component])) for component in components]
    largest_component = int(np.argmax(component_areas)) if component_areas else -1
    isolated_large_components = sum(
        1
        for index, area in enumerate(component_areas)
        if index != largest_component and area >= args.min_safety_component_area_m2
    )
    component_safety = analyze_component_safety(
        vertices,
        faces,
        areas,
        components,
        cloud_points,
        support_distance_m=args.reverse_support_distance,
        min_component_area_m2=args.min_safety_component_area_m2,
        min_reverse_precision=args.min_reverse_precision,
        min_face_reverse_precision=args.min_face_reverse_precision,
        max_unsupported_patch_area_m2=args.max_unsupported_patch_area_m2,
        max_unsupported_patch_span_m=args.max_unsupported_patch_span_m,
        max_total_unsupported_area_m2=args.max_total_unsupported_area_m2,
        tunnel_axis=explicit_tunnel_axis,
        min_tunnel_axis_span_m=args.min_tunnel_axis_span_m,
        min_tunnel_axis_eigenvalue_ratio=args.min_tunnel_axis_eigenvalue_ratio,
        axis_normal_cosine=args.curtain_axis_normal_cosine,
        planar_patch_normal_cosine=args.planar_patch_normal_cosine,
        max_planar_thickness_m=args.max_planar_thickness_m,
        max_planar_thickness_ratio=args.max_planar_thickness_ratio,
        min_curtain_span_m=args.min_curtain_span_m,
    )
    canonical_faces = [tuple(sorted(int(value) for value in face)) for face in faces]
    duplicate_faces = sum(count - 1 for count in Counter(canonical_faces).values() if count > 1)

    runtime = analyze_runtime(
        bag_path,
        args.status_topic,
        args.mesh_topic,
        args.messages,
        None,
        200000,
        args.max_callback_p95_ms,
    )
    callback_p95 = runtime.get("numeric", {}).get("total_ms", {}).get("p95")
    boundary_ratio = topology.get("boundary_edge_ratio")
    nonmanifold_edges = int(topology.get("nonmanifold_edges", 0))
    nonmanifold_vertices = int(topology.get("nonmanifold_vertices", 0))
    long_edges = int(np.count_nonzero(edge_lengths > args.max_edge_m))
    delete_messages = int(runtime.get("mesh_clears_after_first_add", 0))

    checks: List[Dict[str, object]] = []
    add_check(
        checks,
        "cloud_mesh_stamp_sync",
        float(cloud_metadata["stamp_delta_ms"]) <= args.max_sync_ms,
        cloud_metadata["stamp_delta_ms"],
        f"<= {args.max_sync_ms} ms",
    )
    add_check(
        checks,
        "cloud_mesh_frame_match",
        frame_ids_match(marker_metadata.get("frame_id"), cloud_metadata.get("frame_id")),
        {
            "marker": marker_metadata.get("frame_id"),
            "cloud": cloud_metadata.get("frame_id"),
        },
        "identical non-empty Marker and cloud frame_id",
    )
    add_check(
        checks,
        "coverage_0_5_to_10_m",
        near_coverage["coverage"] is not None
        and float(near_coverage["coverage"]) >= args.near_min_coverage,
        near_coverage["coverage"],
        f">= {args.near_min_coverage}",
    )
    add_check(
        checks,
        "coverage_10_to_20_m",
        far_coverage["coverage"] is not None
        and float(far_coverage["coverage"]) >= args.far_min_coverage,
        far_coverage["coverage"],
        f">= {args.far_min_coverage}",
    )
    add_check(
        checks,
        "boundary_edge_ratio",
        boundary_ratio is not None and float(boundary_ratio) < args.max_boundary_ratio,
        boundary_ratio,
        f"< {args.max_boundary_ratio}",
    )
    add_check(
        checks,
        "nonmanifold_edges",
        nonmanifold_edges <= args.max_nonmanifold,
        nonmanifold_edges,
        f"<= {args.max_nonmanifold}",
    )
    add_check(
        checks,
        "nonmanifold_vertices",
        nonmanifold_vertices <= args.max_nonmanifold,
        nonmanifold_vertices,
        f"<= {args.max_nonmanifold}",
    )
    add_check(
        checks,
        "callback_p95_budget",
        callback_p95 is not None and float(callback_p95) < args.max_callback_p95_ms,
        callback_p95,
        f"< {args.max_callback_p95_ms} ms",
    )
    add_check(checks, "no_large_edges", long_edges == 0, long_edges, f"edges <= {args.max_edge_m} m")
    add_check(
        checks,
        "no_duplicate_faces",
        duplicate_faces == 0,
        duplicate_faces,
        "zero duplicate triangles",
    )
    add_check(
        checks,
        "large_component_reverse_precision",
        not component_safety["unsupported_large_components"],
        component_safety["unsupported_large_components"],
        (
            f"every component >= {args.min_safety_component_area_m2} m^2 has "
            f">= {args.min_reverse_precision} area-weighted mesh-to-cloud support "
            f"within {args.reverse_support_distance} m"
        ),
    )
    add_check(
        checks,
        "no_large_unsupported_patches",
        not component_safety["unsupported_patch_components"],
        component_safety["unsupported_patch_components"],
        (
            f"no low-support patch exceeds {args.max_unsupported_patch_area_m2} m^2 "
            f"unsupported area or {args.max_unsupported_patch_span_m} m span"
        ),
    )
    add_check(
        checks,
        "total_unsupported_area_budget",
        not component_safety["total_unsupported_area_exceeded"],
        component_safety["total_unsupported_area_m2"],
        f"<= {args.max_total_unsupported_area_m2} m^2 across all components",
    )
    add_check(
        checks,
        "no_cross_tunnel_curtains",
        not component_safety["cross_tunnel_curtain_components"],
        component_safety["cross_tunnel_curtain_components"],
        "zero large local planar patches spanning the tunnel transverse section",
    )
    add_check(
        checks,
        "no_periodic_clear",
        delete_messages == 0,
        delete_messages,
        "zero DELETE/DELETEALL messages",
    )

    result = {
        "bag": str(bag_path),
        "marker": marker_metadata,
        "cloud": cloud_metadata,
        "coverage": {
            "near": near_coverage,
            "far": far_coverage,
            "surface_sample_count": len(samples),
        },
        "topology": topology,
        "safety": {
            "max_edge_m": float(np.max(edge_lengths)) if len(edge_lengths) else None,
            "p99_edge_m": float(np.percentile(edge_lengths, 99)) if len(edge_lengths) else None,
            "long_edges": long_edges,
            "duplicate_faces": duplicate_faces,
            "isolated_large_components": isolated_large_components,
            "component_area_m2": sorted(component_areas, reverse=True)[:20],
            "component_validation": component_safety,
        },
        "runtime": runtime,
        "checks": checks,
        "all_checks_passed": all(check["passed"] for check in checks),
        "limitations": [
            "Coverage is point-to-densely-sampled-surface distance, not ground-truth completeness.",
            "Reverse precision includes per-component, local unsupported-patch, and global unsupported-area gates.",
            "Thin local planar patches normal to a confident or explicit tunnel axis are rejected as curtains.",
            "Low-confidence automatic tunnel-axis estimates are reported as axis_unknown and do not run the curtain veto.",
            "The callback budget is architecture-specific; ARM64 must run this same check on-device.",
        ],
    }
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["all_checks_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
