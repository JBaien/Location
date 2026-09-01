#!/usr/bin/env python3
"""Summarize PointCloud2 timing, density, fields, and cross-topic skew.

This tool intentionally depends only on ROS1's Python packages and NumPy.  It
does not launch ROS nodes and it never mutates the bag.
"""

import argparse
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import rosbag
import rospy


DEFAULT_TOPICS = (
    "/velodyne_points",
    "/right/velodyne_points",
    "/left/lslidar_point_cloud",
)
INSPVAX_TOPIC = "/novatel_data/inspvax"

POINT_FIELD_DTYPES = {
    1: "i1",  # INT8
    2: "u1",  # UINT8
    3: "i2",  # INT16
    4: "u2",  # UINT16
    5: "i4",  # INT32
    6: "u4",  # UINT32
    7: "f4",  # FLOAT32
    8: "f8",  # FLOAT64
}


def percentile(values: Sequence[float], q: float) -> Optional[float]:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), q))


def finite_number(value: float) -> Optional[float]:
    return float(value) if math.isfinite(value) else None


def summarize(values: Sequence[float]) -> Dict[str, Optional[float]]:
    if not values:
        return {"min": None, "p10": None, "median": None, "p90": None, "p95": None, "max": None, "mean": None}
    array = np.asarray(values, dtype=np.float64)
    return {
        "min": finite_number(float(np.min(array))),
        "p10": percentile(values, 10),
        "median": percentile(values, 50),
        "p90": percentile(values, 90),
        "p95": percentile(values, 95),
        "max": finite_number(float(np.max(array))),
        "mean": finite_number(float(np.mean(array))),
    }


def field_map(message) -> Dict[str, object]:
    return {field.name: field for field in message.fields}


def field_array(message, name: str) -> Optional[np.ndarray]:
    fields = field_map(message)
    field = fields.get(name)
    if field is None or field.count != 1:
        return None
    type_code = POINT_FIELD_DTYPES.get(field.datatype)
    if type_code is None:
        return None
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(endian + type_code)
    width = int(message.width)
    height = int(message.height)
    point_step = int(message.point_step)
    row_step = int(message.row_step) or width * point_step
    if height <= 1 or row_step == width * point_step:
        return np.ndarray(
            shape=(width * height,),
            dtype=dtype,
            buffer=message.data,
            offset=int(field.offset),
            strides=(point_step,),
        )
    rows = []
    for row in range(height):
        rows.append(
            np.ndarray(
                shape=(width,),
                dtype=dtype,
                buffer=message.data,
                offset=row * row_step + int(field.offset),
                strides=(point_step,),
            )
        )
    return np.concatenate(rows)


def nearest_skews(reference: Sequence[float], other: Sequence[float]) -> List[float]:
    if not reference or not other:
        return []
    sorted_other = np.asarray(sorted(other), dtype=np.float64)
    skews = []
    for stamp in reference:
        insertion = int(np.searchsorted(sorted_other, stamp))
        candidates = []
        if insertion < len(sorted_other):
            candidates.append(abs(float(sorted_other[insertion] - stamp)))
        if insertion > 0:
            candidates.append(abs(float(sorted_other[insertion - 1] - stamp)))
        if candidates:
            skews.append(min(candidates))
    return skews


def haversine_m(latitude_a: float, longitude_a: float, latitude_b: float, longitude_b: float) -> float:
    earth_radius_m = 6371000.0
    latitude_a_rad = math.radians(latitude_a)
    latitude_b_rad = math.radians(latitude_b)
    delta_latitude = latitude_b_rad - latitude_a_rad
    delta_longitude = math.radians(longitude_b - longitude_a)
    term = (
        math.sin(delta_latitude * 0.5) ** 2
        + math.cos(latitude_a_rad)
        * math.cos(latitude_b_rad)
        * math.sin(delta_longitude * 0.5) ** 2
    )
    return 2.0 * earth_radius_m * math.asin(min(1.0, math.sqrt(term)))


def analyze_inspvax(bag: rosbag.Bag) -> Optional[Dict[str, object]]:
    """Read the embedded message definition even when novatel_msgs is absent."""
    records = []
    for _, raw_message, bag_stamp in bag.read_messages(topics=[INSPVAX_TOPIC], raw=True):
        _, serialized, _, _, message_class = raw_message
        message = message_class()
        message.deserialize(serialized)
        velocity_mps = math.sqrt(
            float(message.north_velocity) ** 2
            + float(message.east_velocity) ** 2
            + float(message.up_velocity) ** 2
        )
        records.append(
            {
                "stamp": bag_stamp.to_sec(),
                "latitude": float(message.latitude),
                "longitude": float(message.longitude),
                "altitude": float(message.altitude),
                "velocity_mps": velocity_mps,
                "ins_status": int(message.ins_status),
                "position_type": int(message.position_type),
            }
        )
    if not records:
        return None
    intervals = []
    gnss_step_m = []
    gnss_step_speed_mps = []
    for previous, current in zip(records, records[1:]):
        interval = current["stamp"] - previous["stamp"]
        intervals.append(interval)
        horizontal_step = haversine_m(
            previous["latitude"],
            previous["longitude"],
            current["latitude"],
            current["longitude"],
        )
        vertical_step = current["altitude"] - previous["altitude"]
        step = math.hypot(horizontal_step, vertical_step)
        gnss_step_m.append(step)
        if interval > 0.0:
            gnss_step_speed_mps.append(step / interval)
    status_values = sorted({record["ins_status"] for record in records})
    status_names = {
        0: "INS_STATUS_INACTIVE",
        1: "INS_STATUS_ALIGNING",
        2: "INS_STATUS_HIGH_VARIANCE",
        3: "INS_STATUS_SOLUTION_GOOD",
    }
    return {
        "messages": len(records),
        "interval_s": summarize(intervals),
        "reported_speed_mps": summarize([record["velocity_mps"] for record in records]),
        "gnss_step_m": summarize(gnss_step_m),
        "gnss_step_speed_mps": summarize(gnss_step_speed_mps),
        "start_to_end_horizontal_m": haversine_m(
            records[0]["latitude"],
            records[0]["longitude"],
            records[-1]["latitude"],
            records[-1]["longitude"],
        ),
        "ins_status_values": status_values,
        "ins_status_names": [status_names.get(value, f"UNKNOWN_{value}") for value in status_values],
        "position_type_values": sorted({record["position_type"] for record in records}),
        "caveat": (
            "INSPVAX is only a 1 Hz motion reference here. Every sampled ins_status is "
            "INS_STATUS_INACTIVE, so it is evidence that the platform moves but is not trajectory ground truth."
        ),
    }


def topic_summary(state: Dict[str, object]) -> Dict[str, object]:
    stamps = state["stamps"]
    intervals = [right - left for left, right in zip(stamps, stamps[1:])]
    monotonic_failures = sum(1 for interval in intervals if interval <= 0.0)
    duration = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
    rate = (len(stamps) - 1) / duration if duration > 0.0 else None
    return {
        "sampled_frames": len(stamps),
        "header_frame_id": state["frame_id"],
        "fields": state["fields"],
        "point_step_bytes": state["point_step"],
        "point_count": summarize(state["point_counts"]),
        "finite_xyz_ratio": summarize(state["finite_ratios"]),
        "usable_0_5_to_80m_ratio": summarize(state["usable_ratios"]),
        "range_median_m": summarize(state["range_medians"]),
        "ring_count": summarize(state["ring_counts"]),
        "points_per_lidar_id": {
            str(lidar_id): summarize(counts)
            for lidar_id, counts in sorted(state["lidar_point_counts"].items())
        },
        "header_interval_s": summarize(intervals),
        "header_rate_hz": finite_number(rate) if rate is not None else None,
        "bag_minus_header_s": summarize(state["bag_header_lags"]),
        "non_monotonic_header_intervals": monotonic_failures,
    }


def analyze_bag(
    bag_path: Path,
    topics: Sequence[str],
    frame_limit: int,
    start_offset_s: float,
) -> Dict[str, object]:
    states = {
        topic: {
            "stamps": [],
            "point_counts": [],
            "finite_ratios": [],
            "usable_ratios": [],
            "range_medians": [],
            "ring_counts": [],
            "lidar_point_counts": {},
            "bag_header_lags": [],
            "frame_id": "",
            "fields": [],
            "point_step": 0,
        }
        for topic in topics
    }

    with rosbag.Bag(str(bag_path), "r") as bag:
        bag_start = float(bag.get_start_time())
        bag_end = float(bag.get_end_time())
        topic_info = bag.get_type_and_topic_info().topics
        available_topics = [topic for topic in topics if topic in topic_info]
        missing_topics = [topic for topic in topics if topic not in topic_info]
        if not available_topics:
            raise RuntimeError("none of the requested topics are present in the bag")
        start_time = rospy.Time.from_sec(bag_start + max(0.0, start_offset_s))
        for topic, message, bag_stamp in bag.read_messages(
            topics=available_topics,
            start_time=start_time,
        ):
            state = states[topic]
            if len(state["stamps"]) >= frame_limit:
                if all(len(states[name]["stamps"]) >= frame_limit for name in available_topics):
                    break
                continue

            point_count = int(message.width) * int(message.height)
            stamp = message.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = bag_stamp.to_sec()
            state["stamps"].append(stamp)
            state["point_counts"].append(point_count)
            state["bag_header_lags"].append(bag_stamp.to_sec() - stamp)
            if not state["frame_id"]:
                state["frame_id"] = message.header.frame_id
                state["fields"] = [
                    {
                        "name": field.name,
                        "offset": int(field.offset),
                        "datatype": int(field.datatype),
                        "count": int(field.count),
                    }
                    for field in message.fields
                ]
                state["point_step"] = int(message.point_step)

            x = field_array(message, "x")
            y = field_array(message, "y")
            z = field_array(message, "z")
            if x is None or y is None or z is None or point_count == 0:
                state["finite_ratios"].append(0.0)
                state["usable_ratios"].append(0.0)
                continue
            finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            state["finite_ratios"].append(float(np.count_nonzero(finite)) / point_count)
            ranges = np.sqrt(
                x[finite].astype(np.float64) ** 2
                + y[finite].astype(np.float64) ** 2
                + z[finite].astype(np.float64) ** 2
            )
            usable = (ranges >= 0.5) & (ranges <= 80.0)
            state["usable_ratios"].append(float(np.count_nonzero(usable)) / point_count)
            if ranges.size:
                state["range_medians"].append(float(np.median(ranges)))
            ring = field_array(message, "ring")
            if ring is not None:
                state["ring_counts"].append(int(np.unique(ring).size))
            lidar_id = field_array(message, "lidar_id")
            if lidar_id is not None:
                identifiers, counts = np.unique(lidar_id, return_counts=True)
                for identifier, count in zip(identifiers, counts):
                    state["lidar_point_counts"].setdefault(int(identifier), []).append(int(count))

        summaries = {
            topic: topic_summary(states[topic])
            for topic in available_topics
        }
        sync = {}
        reference_topic = "/velodyne_points" if "/velodyne_points" in available_topics else available_topics[0]
        for topic in available_topics:
            if topic == reference_topic:
                continue
            sync[f"{reference_topic} -> {topic}"] = summarize(
                nearest_skews(states[reference_topic]["stamps"], states[topic]["stamps"])
            )
        return {
            "bag": str(bag_path),
            "bag_size_bytes": bag_path.stat().st_size,
            "bag_start_s": bag_start,
            "bag_end_s": bag_end,
            "bag_duration_s": bag_end - bag_start,
            "requested_frames_per_topic": frame_limit,
            "start_offset_s": start_offset_s,
            "missing_topics": missing_topics,
            "topics": summaries,
            "nearest_header_stamp_skew_s": sync,
            "inspvax_motion_reference": analyze_inspvax(bag) if INSPVAX_TOPIC in topic_info else None,
            "limitations": [
                "Statistics cover only the requested sample window, not every bag frame.",
                "This bag contains no high-rate odometry or TF ground truth; ICP convergence is not trajectory accuracy.",
            ],
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--start-offset", type=float, default=0.0, help="seconds from bag start")
    parser.add_argument("--topic", action="append", dest="topics", help="repeat to override default topics")
    parser.add_argument("--output", type=Path, help="optional JSON output path")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.frames <= 0:
        raise SystemExit("--frames must be positive")
    result = analyze_bag(
        args.bag.resolve(),
        tuple(args.topics) if args.topics else DEFAULT_TOPICS,
        args.frames,
        args.start_offset,
    )
    rendered = json.dumps(result, indent=2, sort_keys=True, ensure_ascii=False)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
