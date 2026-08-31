#!/usr/bin/env python3
"""Summarize local TSDF diagnostics and Marker snapshots from a ROS bag.

This is an integration/performance check, not a unit test.  Record at least
100 consecutive status messages and full mesh snapshots, then use the optional
limits to turn expected rolling-window and 10 Hz budgets into explicit checks.
"""

import argparse
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np


NUMERIC_KEYS = {
    "raw_points",
    "finite_points",
    "invalid_points",
    "registration_points",
    "tsdf_range_points",
    "surface_range_points",
    "registration_range_filtered",
    "tsdf_range_filtered",
    "surface_range_filtered",
    "fitness_score",
    "inlier_ratio",
    "relative_translation_m",
    "relative_rotation_deg",
    "prediction_translation_error_m",
    "prediction_rotation_error_deg",
    "integrated_rays",
    "source_topology_points",
    "support_input_points",
    "support_topology_points",
    "support_ring_pairs",
    "support_rejected_ring_pairs",
    "support_rejected_ring_order_pairs",
    "support_candidate_quads",
    "support_locally_valid_quads",
    "support_strong_quads",
    "support_accepted_quads",
    "support_verified_long_quads",
    "support_rejected_long_quads",
    "support_rejected_isolated_quads",
    "support_candidate_samples",
    "support_surface_rays",
    "support_integrated_rays",
    "support_contributed_voxels",
    "support_mesh_vertices",
    "support_mesh_triangles",
    "support_mesh_curve_intervals",
    "support_mesh_skipped_curve_intervals",
    "support_mesh_skipped_degenerate_intervals",
    "support_mesh_skipped_sensor_columns",
    "support_mesh_output_equivalence_input_triangles",
    "support_mesh_output_equivalence_removed_triangles",
    "support_mesh_output_equivalence_masked_sensor_columns",
    "support_mesh_first_curve_gap_sensor_id",
    "support_mesh_first_curve_gap_lower_ring",
    "support_mesh_first_curve_gap_start_deg",
    "support_mesh_first_curve_gap_end_deg",
    "support_validation_input_triangles",
    "support_validation_output_triangles",
    "support_validation_rejected_total",
    "support_validation_rejected_invalid_index",
    "support_validation_rejected_nonfinite",
    "support_validation_rejected_degenerate",
    "support_validation_rejected_duplicate",
    "support_validation_rejected_large_face",
    "support_validation_rejected_nonmanifold_vertex",
    "support_validation_rejected_nonmanifold_edge",
    "contributed_voxels",
    "evicted_frames",
    "active_frames",
    "active_voxels",
    "mesh_triangles",
    "consecutive_registration_failures",
    "registration_failures_total",
    "integrated_frames_total",
    "surface_frames_total",
    "surface_rejected_frames_total",
    "direct_applied_frames_total",
    "rejected_frames_total",
    "reset_count",
    "decode_ms",
    "registration_ms",
    "integration_ms",
    "support_build_ms",
    "support_integration_ms",
    "extraction_ms",
    "mesh_cleanup_ms",
    "mesh_base_cleanup_ms",
    "mesh_base_equivalent_repair_ms",
    "mesh_support_validation_ms",
    "mesh_append_ms",
    "mesh_cross_equivalence_probe_ms",
    "mesh_combined_validation_ms",
    "total_ms",
}

BOOLEAN_KEYS = {
    "integration_accepted",
    "volume_updated",
    "volume_committed",
    "surface_prepared",
    "surface_output_accepted",
    "mesh_published_this_frame",
    "measured_capacity_limited",
    "measured_tsdf_enabled",
    "registration_accepted",
    "registration_converged",
    "motion_prediction_valid",
    "mesh_triangle_limit_reached",
    "support_enabled",
    "support_build_surface_rays",
    "support_build_indexed_mesh",
    "support_candidate_budget_limited",
    "support_surface_budget_limited",
    "support_mesh_budget_limited",
    "support_mesh_accepted",
    "support_mesh_applied",
    "support_mesh_has_first_curve_gap",
    "support_output_topology_valid",
}


def finite_float(value: str) -> Optional[float]:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def summary(values: Sequence[float]) -> Dict[str, Optional[float]]:
    finite = np.asarray([value for value in values if math.isfinite(value)], dtype=np.float64)
    if not len(finite):
        return {"min": None, "p10": None, "median": None, "p90": None, "p95": None, "max": None, "mean": None}
    return {
        "min": float(np.min(finite)),
        "p10": float(np.percentile(finite, 10)),
        "median": float(np.percentile(finite, 50)),
        "p90": float(np.percentile(finite, 90)),
        "p95": float(np.percentile(finite, 95)),
        "max": float(np.max(finite)),
        "mean": float(np.mean(finite)),
    }


def parse_boolean(value: str) -> Optional[bool]:
    lowered = value.strip().lower()
    if lowered in ("1", "true", "yes"):
        return True
    if lowered in ("0", "false", "no"):
        return False
    return None


def counter_dict(counter: Counter) -> Dict[str, int]:
    return {str(key): int(value) for key, value in sorted(counter.items(), key=lambda item: str(item[0]))}


def header_stamp(message, bag_stamp) -> float:
    stamp = message.header.stamp.to_sec()
    return stamp if stamp > 0.0 else bag_stamp.to_sec()


def analyze(
    bag_path: Path,
    status_topic: str,
    mesh_topic: str,
    limit: int,
    max_active_frames: Optional[int],
    max_triangles: Optional[int],
    budget_ms: Optional[float],
) -> Dict[str, object]:
    try:
        import rosbag  # pylint: disable=import-outside-toplevel
    except ImportError as error:
        raise RuntimeError("ROS Python rosbag is required for bag metrics") from error

    status_records: List[Dict[str, object]] = []
    marker_records: List[Dict[str, object]] = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        topic_info = bag.get_type_and_topic_info().topics
        missing_topics = [topic for topic in (status_topic, mesh_topic) if topic not in topic_info]
        if status_topic not in topic_info:
            raise RuntimeError(f"status topic is absent: {status_topic}")
        for topic, message, bag_stamp in bag.read_messages(topics=[status_topic, mesh_topic]):
            if topic == status_topic and len(status_records) < limit:
                if not message.status:
                    status_records.append({"stamp": header_stamp(message, bag_stamp), "level": None, "message": "", "values": {}})
                else:
                    status = message.status[0]
                    status_records.append(
                        {
                            "stamp": header_stamp(message, bag_stamp),
                            "level": int(status.level),
                            "message": status.message,
                            "values": {item.key: item.value for item in status.values},
                        }
                    )
            elif topic == mesh_topic and len(marker_records) < limit:
                marker_records.append(
                    {
                        "stamp": header_stamp(message, bag_stamp),
                        "frame_id": message.header.frame_id,
                        "type": int(message.type),
                        "action": int(message.action),
                        "points": len(message.points),
                        "triangles": len(message.points) // 3,
                        "point_count_valid": len(message.points) % 3 == 0,
                    }
                )
            if len(status_records) >= limit and (mesh_topic in missing_topics or len(marker_records) >= limit):
                break

    numeric: Dict[str, List[float]] = defaultdict(list)
    booleans: Dict[str, List[bool]] = defaultdict(list)
    categorical: Dict[str, Counter] = defaultdict(Counter)
    schema_counts = Counter()
    for record in status_records:
        values = record["values"]
        schema_counts[tuple(sorted(values))] += 1
        for key, value in values.items():
            if key in NUMERIC_KEYS:
                parsed = finite_float(value)
                if parsed is not None:
                    numeric[key].append(parsed)
            elif key in BOOLEAN_KEYS:
                parsed = parse_boolean(value)
                if parsed is not None:
                    booleans[key].append(parsed)
            else:
                categorical[key][value] += 1

    status_stamps = [float(record["stamp"]) for record in status_records]
    # Startup/reset DELETE markers may use wall time while subsequent ADD
    # snapshots use the replayed cloud stamp.  Mixing those two clock domains
    # creates a meaningless negative interval.  Mesh cadence is defined only
    # between consecutive surface snapshots; DELETE frequency is reported
    # separately below.
    marker_stamps = [
        float(record["stamp"])
        for record in marker_records
        if record["action"] in (0, 1)
    ]
    status_intervals = [right - left for left, right in zip(status_stamps, status_stamps[1:])]
    marker_intervals = [right - left for left, right in zip(marker_stamps, marker_stamps[1:])]
    seen_mesh_add = False
    mesh_clears_after_first_add = 0
    for record in marker_records:
        if record["action"] in (0, 1):
            seen_mesh_add = True
        elif record["action"] in (2, 3) and seen_mesh_add:
            mesh_clears_after_first_add += 1
    checks = []

    def add_check(name: str, passed: bool, observed, expectation: str) -> None:
        checks.append({"name": name, "passed": bool(passed), "observed": observed, "expectation": expectation})

    add_check(
        "status_sample_count",
        len(status_records) >= limit,
        len(status_records),
        f">= {limit} consecutive messages",
    )
    add_check(
        "status_stamp_monotonic",
        bool(status_intervals) and all(interval > 0.0 for interval in status_intervals),
        sum(1 for interval in status_intervals if interval <= 0.0),
        "zero non-positive intervals",
    )
    if marker_records:
        invalid_marker_messages = sum(
            1
            for record in marker_records
            if not (
                (record["action"] in (0, 1) and record["type"] == 11 and record["point_count_valid"])
                or (record["action"] in (2, 3) and record["points"] == 0)
            )
        )
        add_check(
            "triangle_list_encoding",
            invalid_marker_messages == 0,
            invalid_marker_messages,
            "ADD markers are TRIANGLE_LIST with points % 3 == 0; DELETE markers have no points",
        )
    if max_active_frames is not None:
        observed = max(numeric.get("active_frames", [math.inf]))
        add_check("rolling_frame_cap", observed <= max_active_frames, observed, f"<= {max_active_frames}")
        integrated_span = (
            max(numeric.get("integrated_frames_total", [0.0]))
            - min(numeric.get("integrated_frames_total", [0.0]))
        )
        if integrated_span >= max_active_frames:
            add_check(
                "rolling_eviction_observed",
                sum(numeric.get("evicted_frames", [])) > 0,
                sum(numeric.get("evicted_frames", [])),
                "at least one evicted frame after the window fills",
            )
    if max_triangles is not None:
        observed = max(numeric.get("mesh_triangles", [math.inf]))
        add_check("mesh_triangle_cap", observed <= max_triangles, observed, f"<= {max_triangles}")
    if budget_ms is not None:
        observed = summary(numeric.get("total_ms", []))["p95"]
        add_check(
            "callback_p95_budget",
            observed is not None and observed <= budget_ms,
            observed,
            f"p95 <= {budget_ms} ms",
        )
    if "raw_points" in numeric and "finite_points" in numeric and "invalid_points" in numeric:
        triples = zip(numeric["raw_points"], numeric["finite_points"], numeric["invalid_points"])
        mismatches = sum(1 for raw, finite, invalid in triples if abs(raw - finite - invalid) > 0.5)
        add_check("decode_point_accounting", mismatches == 0, mismatches, "raw = finite + invalid for every record")
    cumulative_totals = {
        key: numeric.get(key, [])
        for key in (
            "integrated_frames_total",
            "surface_frames_total",
            "surface_rejected_frames_total",
            "direct_applied_frames_total",
            "rejected_frames_total",
        )
    }
    decreases = {
        key: sum(1 for left, right in zip(values, values[1:]) if right < left)
        for key, values in cumulative_totals.items()
    }
    add_check(
        "cumulative_counters_monotonic",
        all(count == 0 for count in decreases.values()),
        decreases,
        "all cumulative frame counters never decrease",
    )

    schema_variants = [
        {"count": count, "keys": list(keys)}
        for keys, count in schema_counts.most_common()
    ]
    return {
        "bag": str(bag_path),
        "status_topic": status_topic,
        "mesh_topic": mesh_topic,
        "missing_topics": missing_topics,
        "status_messages": len(status_records),
        "mesh_messages": len(marker_records),
        "status_interval_s": summary(status_intervals),
        "mesh_interval_s": summary(marker_intervals),
        "status_levels": counter_dict(Counter(record["level"] for record in status_records)),
        "status_messages_by_text": counter_dict(Counter(record["message"] for record in status_records)),
        "numeric": {key: summary(values) for key, values in sorted(numeric.items())},
        "boolean_true_ratio": {
            key: sum(values) / len(values) for key, values in sorted(booleans.items()) if values
        },
        "categorical": {key: counter_dict(values) for key, values in sorted(categorical.items())},
        "mesh_triangles": summary(
            [record["triangles"] for record in marker_records if record["action"] in (0, 1)]
        ),
        "mesh_actions": counter_dict(Counter(record["action"] for record in marker_records)),
        # A latched DELETE at producer startup is an authoritative reset, not
        # the periodic clear/flicker failure this metric is intended to catch.
        "mesh_clears_after_first_add": mesh_clears_after_first_add,
        "mesh_frame_ids": counter_dict(Counter(record["frame_id"] for record in marker_records)),
        "schema_variants": schema_variants,
        "checks": checks,
        "all_checks_passed": all(check["passed"] for check in checks),
        "limitations": [
            "Diagnostic timing is callback wall time, not browser rendering or end-to-end latency.",
            "Topology and cross-section closure require mesh_topology_metrics.py on selected snapshots.",
            "A rolling cap check proves bounded frame count, not geometric registration accuracy.",
        ],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--status-topic", default="/local_tsdf_mesh/status")
    parser.add_argument("--mesh-topic", default="/local_tsdf_mesh/mesh")
    parser.add_argument("--messages", type=int, default=100)
    parser.add_argument("--max-active-frames", type=int)
    parser.add_argument("--max-triangles", type=int)
    parser.add_argument("--budget-ms", type=float)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.messages <= 1:
        raise SystemExit("--messages must be greater than one")
    result = analyze(
        args.bag.resolve(),
        args.status_topic,
        args.mesh_topic,
        args.messages,
        args.max_active_frames,
        args.max_triangles,
        args.budget_ms,
    )
    rendered = json.dumps(result, indent=2, sort_keys=True)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0 if result["all_checks_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
