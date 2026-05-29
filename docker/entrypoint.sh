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

serve_web() {
  if [ -d "${MINE_WEB_DIST}" ]; then
    cd "${MINE_WEB_DIST}"
    python3 -m http.server "${WEB_HTTP_PORT:-8080}" --bind 0.0.0.0
  else
    echo "Web dist directory not found: ${MINE_WEB_DIST}" >&2
    return 1
  fi
}

case "${1:-bringup}" in
  bringup)
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
