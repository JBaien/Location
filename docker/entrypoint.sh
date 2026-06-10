#!/usr/bin/env bash
set -e

source /opt/ros/melodic/setup.bash
source /catkin_ws/devel/setup.bash

RUNTIME_DIR="${MINE_RUNTIME_DIR:-/config}"
mkdir -p "${RUNTIME_DIR}" /data/bags

if [ -d /opt/mine_lidar_runtime ]; then
  cp -rn /opt/mine_lidar_runtime/. "${RUNTIME_DIR}/"
fi

export ROS_HOME="${ROS_HOME:-/tmp/ros}"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
export ROS_IP="${ROS_IP:-127.0.0.1}"

load_driver_family() {
  local driver_config="${MINE_DRIVER_CONFIG:-${RUNTIME_DIR}/runtime/driver.yaml}"
  local driver_family=""

  if [ -z "${MINE_DRIVER_FAMILY:-}" ] && [ -f "${driver_config}" ]; then
    driver_family="$(awk -F: '
      /^[ \t]*driver_family[ \t]*:/ {
        value=$2
        sub(/#.*/, "", value)
        gsub(/^[ \t"\047]+|[ \t"\047]+$/, "", value)
        print value
        exit
      }
    ' "${driver_config}")"
    if [ -n "${driver_family}" ]; then
      export MINE_DRIVER_FAMILY="${driver_family}"
    fi
  fi

  case "${MINE_DRIVER_FAMILY:-}" in
    ""|tm16|timoo|tmlidar|timoo_sdk)
      ;;
    *)
      echo "Invalid driver_family: ${MINE_DRIVER_FAMILY}" >&2
      echo "Allowed values: tm16, tmlidar, timoo_sdk" >&2
      return 1
      ;;
  esac
}

serve_web() {
  if [ -d "${MINE_WEB_DIST}" ]; then
    cd "${MINE_WEB_DIST}"
    python3 -m http.server "${WEB_HTTP_PORT:-18080}" --bind 0.0.0.0
  else
    echo "Web dist directory not found: ${MINE_WEB_DIST}" >&2
    return 1
  fi
}

case "${1:-bringup}" in
  bringup)
    load_driver_family
    serve_web &
    WEB_PID=$!
    trap 'kill ${WEB_PID} 2>/dev/null || true' EXIT
    roslaunch "${RUNTIME_DIR}/launch/bringup.launch" "${@:2}"
    ;;
  web)
    serve_web
    ;;
  bash|/bin/bash)
    exec /bin/bash
    ;;
  *)
    exec "$@"
    ;;
esac
