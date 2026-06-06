#!/usr/bin/env python3
"""Static checks for Docker runtime LiDAR driver selection.

These tests intentionally avoid ROS runtime dependencies. They validate the
deployment contract that operators select the default driver in one YAML file,
while launch files and package code provide the matching implementation.
"""

import re
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
ALLOWED_FAMILIES = {"tm16", "tmlidar", "timoo_sdk"}
DEFAULT_DRIVER_FAMILY = "timoo_sdk"


def read_text(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def parse_simple_yaml_value(relative_path, key):
    text = read_text(relative_path)
    pattern = re.compile(r"^\s*" + re.escape(key) + r"\s*:\s*([A-Za-z0-9_/-]+)\s*$")
    for line in text.splitlines():
        match = pattern.match(line)
        if match:
            return match.group(1)
    raise AssertionError(f"{relative_path} does not define {key}")


def launch_group_arg_values(relative_path, arg_name):
    path = ROOT / relative_path
    root = ET.parse(path).getroot()
    values = []
    for group in root.findall("group"):
        namespace = group.attrib.get("ns", "")
        for include in group.findall("include"):
            for arg in include.findall("arg"):
                if arg.attrib.get("name") == arg_name:
                    values.append((namespace, arg.attrib.get("value", "")))
    return values


def assert_unique_launch_arg_values(relative_path, arg_name):
    values = launch_group_arg_values(relative_path, arg_name)
    require(values, f"{relative_path} must define {arg_name} per LiDAR")
    seen = {}
    duplicates = []
    for namespace, value in values:
        previous = seen.setdefault(value, namespace)
        if previous != namespace:
            duplicates.append(f"{value} used by {previous} and {namespace}")
    require(not duplicates,
            f"{relative_path} has duplicate {arg_name}: {', '.join(duplicates)}")


def assert_launch_args_do_not_share_values(relative_path, arg_names):
    values = []
    for arg_name in arg_names:
        values.extend(
            (namespace, arg_name, value)
            for namespace, value in launch_group_arg_values(relative_path, arg_name)
        )
    require(values, f"{relative_path} must define {', '.join(arg_names)}")
    seen = {}
    duplicates = []
    for namespace, arg_name, value in values:
        previous = seen.setdefault(value, (namespace, arg_name))
        if previous != (namespace, arg_name):
            prev_ns, prev_arg = previous
            duplicates.append(
                f"{value} used by {prev_ns}/{prev_arg} and {namespace}/{arg_name}"
            )
    require(not duplicates,
            f"{relative_path} has shared UDP port values: {', '.join(duplicates)}")


def assert_driver_yaml_controls_default_family():
    family = parse_simple_yaml_value("docker/runtime/runtime/driver.yaml",
                                     "driver_family")
    require(family in ALLOWED_FAMILIES,
            f"driver_family must be one of {sorted(ALLOWED_FAMILIES)}")
    require(family == DEFAULT_DRIVER_FAMILY,
            f"runtime default driver_family must be {DEFAULT_DRIVER_FAMILY}")

    entrypoint = read_text("docker/entrypoint.sh")
    require("driver.yaml" in entrypoint,
            "entrypoint.sh must read the runtime driver.yaml file")
    require("MINE_DRIVER_FAMILY" in entrypoint,
            "entrypoint.sh must export MINE_DRIVER_FAMILY from driver.yaml")

    bringup = read_text("docker/runtime/launch/bringup.launch")
    require('default="$(optenv MINE_DRIVER_FAMILY tmlidar)"' in bringup,
            "bringup.launch must use driver.yaml only as the default driver")
    require('driver_family" default="tmlidar"' not in bringup,
            "bringup.launch must not require editing a hard-coded driver default")


def assert_entrypoint_loads_driver_yaml_value():
    entrypoint = read_text("docker/entrypoint.sh")
    match = re.search(r"^load_driver_family\(\) \{.*?^}", entrypoint,
                      re.M | re.S)
    require(match is not None, "entrypoint.sh must define load_driver_family")
    require("[[:space:]]" not in match.group(0),
            "entrypoint driver parser must be compatible with Melodic mawk")

    with tempfile.TemporaryDirectory() as temp_dir:
        runtime_dir = Path(temp_dir)
        (runtime_dir / "runtime").mkdir()
        (runtime_dir / "runtime" / "driver.yaml").write_text(
            "driver_family: 'timoo_sdk'  # runtime default\n",
            encoding="utf-8")
        script = f"""
set -e
RUNTIME_DIR={runtime_dir}
{match.group(0)}
load_driver_family
test "${{MINE_DRIVER_FAMILY}}" = "timoo_sdk"
"""
        result = subprocess.run(["bash", "-lc", script],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE,
                                text=True)
        require(result.returncode == 0,
                "entrypoint.sh must parse driver.yaml into MINE_DRIVER_FAMILY")


def assert_bringup_maps_all_driver_families():
    bringup = read_text("docker/runtime/launch/bringup.launch")
    for family in sorted(ALLOWED_FAMILIES):
        require(f"arg('driver_family') == '{family}'" in bringup,
                f"bringup.launch missing driver family branch: {family}")

    require("driver_timoo_sdk_multi3.launch" in bringup,
            "bringup.launch must include the timoo SDK multi-LiDAR launch")
    require("multi_lidar_fusion_timoo_sdk.yaml" in bringup,
            "bringup.launch must select the timoo SDK fusion config")


def assert_downstream_runtime_configs_are_driver_independent():
    target_config = read_text("docker/runtime/target_localizer/target_localizer.yaml")
    require(re.search(r"^input_topic:\s*/points_raw\s*$", target_config, re.M),
            "target localizer must subscribe to fused /points_raw, not one driver topic")
    require(re.search(r"^target_frame:\s*base_link\s*$", target_config, re.M),
            "target localizer target_frame must match fused output frame base_link")
    require(re.search(r"^required_frame_id:\s*base_link\s*$", target_config, re.M),
            "equipment state required_frame_id must match fused output frame base_link")
    require(re.search(r"^[ \t]*ins:\n[ \t]*enabled:\s*false\s*$", target_config, re.M),
            "runtime Modbus INS must be disabled by default")
    require(re.search(r"^[ \t]*mmwave:\n[ \t]*enabled:\s*false\s*$", target_config, re.M),
            "runtime Modbus mmwave must be disabled by default")

    web_config = read_text("docker/runtime/web/web_viewer.yaml")
    require(re.search(r"^\s*current_cloud:\s*/points_raw\s*$", web_config, re.M),
            "web current_cloud must use driver-independent fused /points_raw")
    require(re.search(r"^\s*stable_map:\s*/points_raw\s*$", web_config, re.M),
            "web stable_map must use fused /points_raw")


def assert_timoo_sdk_package_is_in_catkin_workspace():
    require((ROOT / "catkin_ws/src/timoo_ros_driver/package.xml").is_file(),
            "timoo_ros_driver must live under catkin_ws/src")
    require(not (ROOT / "timoo_ros_driver/package.xml").exists(),
            "root-level timoo_ros_driver package should be moved into catkin_ws/src")


def assert_timoo_sdk_runtime_launch_contract():
    launch = read_text("docker/runtime/launch/driver_timoo_sdk_multi3.launch")
    for index in (1, 2, 3):
        ns = f"lidar{index}"
        require(f'<group ns="{ns}">' in launch,
                f"timoo SDK launch must define namespace {ns}")
        require(f'<param name="frame_id" value="{ns}"' in launch,
                f"timoo SDK launch must set frame_id={ns}")
    require('type="timoo_ros_driver_node"' in launch,
            "timoo SDK launch must use the actual executable name")
    require("65536" not in launch,
            "timoo SDK launch must not contain invalid UDP port 65536")

    fusion = read_text("docker/runtime/lidar_fusion/multi_lidar_fusion_timoo_sdk.yaml")
    for index in (1, 2, 3):
        require(f"- /lidar{index}/timoo_points" in fusion,
                f"timoo SDK fusion config missing /lidar{index}/timoo_points")


def assert_multi_lidar_driver_ports_do_not_conflict():
    assert_unique_launch_arg_values("docker/runtime/launch/driver_tm16_multi3.launch",
                                    "port")
    assert_unique_launch_arg_values("docker/runtime/launch/driver_tm16_multi3.launch",
                                    "status_port")
    assert_launch_args_do_not_share_values(
        "docker/runtime/launch/driver_tm16_multi3.launch",
        ("port", "status_port"))
    assert_unique_launch_arg_values("docker/runtime/launch/driver_tmlidar_multi3.launch",
                                    "port")


def assert_timoo_sdk_node_contract():
    main_cpp = read_text("catkin_ws/src/timoo_ros_driver/src/main.cpp")
    require('nh.param("frame_id"' in main_cpp,
            "timoo SDK node must expose frame_id as a runtime parameter")
    require('output.header.frame_id = frame_id' in main_cpp,
            "timoo SDK node must not hard-code PointCloud2 frame_id")
    require('imu_msg.header.frame_id = frame_id' in main_cpp,
            "timoo SDK node must not hard-code IMU frame_id")
    require('nh.advertise<sensor_msgs::PointCloud2>("timoo_points"' in main_cpp,
            "timoo SDK node must publish timoo_points in the LiDAR namespace")
    require('private_nh.advertise<sensor_msgs::PointCloud2>("timoo_points"' not in main_cpp,
            "timoo SDK node must not publish timoo_points under the private node namespace")


def assert_timoo_sdk_targets_do_not_collide():
    sdk_root = ROOT / "catkin_ws/src/timoo_ros_driver/third_party/TimooLidarDriverSDK"
    cmake_text = "\n".join(
        path.read_text(encoding="utf-8")
        for path in sdk_root.rglob("CMakeLists.txt")
    )
    for target in ("timoo_input", "timoo_driver"):
        require(f"add_library({target}" not in cmake_text,
                f"SDK target {target} collides with existing catkin targets")
    for target in ("timoo_sdk_input", "timoo_sdk_driver",
                   "timoo_sdk_data_parser", "timoo_sdk_func"):
        require(f"add_library({target}" in cmake_text,
                f"SDK target {target} must be defined with a unique name")


def main():
    checks = [
        assert_driver_yaml_controls_default_family,
        assert_entrypoint_loads_driver_yaml_value,
        assert_bringup_maps_all_driver_families,
        assert_downstream_runtime_configs_are_driver_independent,
        assert_timoo_sdk_package_is_in_catkin_workspace,
        assert_timoo_sdk_runtime_launch_contract,
        assert_multi_lidar_driver_ports_do_not_conflict,
        assert_timoo_sdk_node_contract,
        assert_timoo_sdk_targets_do_not_collide,
    ]
    failures = []
    for check in checks:
        try:
            check()
        except Exception as exc:
            failures.append(f"{check.__name__}: {exc}")

    if failures:
        print("Runtime driver selection checks failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("Runtime driver selection checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
