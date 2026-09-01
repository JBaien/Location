#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." >/dev/null 2>&1 && pwd)"

IMAGE="${IMAGE:-mine-lidar-runtime:melodic-arm64}"
OUTPUT="${OUTPUT:-mine-lidar-runtime-melodic-arm64.tar.gz}"
PLATFORM="${PLATFORM:-linux/arm64}"
BUILDER="${BUILDER:-default}"
BUILDER_IMAGE="${BUILDER_IMAGE:-docker.1ms.run/moby/buildkit:buildx-stable-1}"
PROXY="${PROXY:-http://127.0.0.1:7890}"
NO_PROXY="${NO_PROXY:-localhost,127.0.0.1,::1}"
NETWORK="${NETWORK:-host}"
BINFMT_IMAGE="${BINFMT_IMAGE:-docker.1ms.run/tonistiigi/binfmt}"
ENSURE_BINFMT="${ENSURE_BINFMT:-1}"
CATKIN_MAKE_ARGS="${CATKIN_MAKE_ARGS:--j4 -l4}"

DOCKER_REGISTRY="${DOCKER_REGISTRY:-docker.1ms.run}"
NODE_IMAGE_TAG="${NODE_IMAGE_TAG:-20-bookworm}"
UBUNTU_APT_MIRROR="${UBUNTU_APT_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports}"
ROS_APT_MIRROR="${ROS_APT_MIRROR:-https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu}"
NPM_REGISTRY="${NPM_REGISTRY:-https://registry.npmmirror.com}"

export HTTP_PROXY="${PROXY}"
export HTTPS_PROXY="${PROXY}"
export http_proxy="${PROXY}"
export https_proxy="${PROXY}"
export NO_PROXY="${NO_PROXY}"
export no_proxy="${no_proxy:-${NO_PROXY}}"

if [ "${ENSURE_BINFMT}" != "0" ]; then
  docker run --privileged --rm "${BINFMT_IMAGE}" --install arm64 >/dev/null
fi

if [ "${BUILDER}" = "default" ]; then
  docker buildx use default >/dev/null
else
  if ! docker buildx inspect "${BUILDER}" >/dev/null 2>&1; then
    docker buildx create \
      --name "${BUILDER}" \
      --driver docker-container \
      --driver-opt "image=${BUILDER_IMAGE}" \
      --driver-opt "env.http_proxy=${http_proxy}" \
      --driver-opt "env.https_proxy=${https_proxy}" \
      --driver-opt "env.no_proxy=${no_proxy}" \
      --use >/dev/null
  else
    docker buildx use "${BUILDER}" >/dev/null
  fi
fi

docker buildx build \
  --platform "${PLATFORM}" \
  --network "${NETWORK}" \
  -f "${SCRIPT_DIR}/Dockerfile.arm64" \
  -t "${IMAGE}" \
  --build-arg DOCKER_REGISTRY="${DOCKER_REGISTRY}" \
  --build-arg NODE_IMAGE_TAG="${NODE_IMAGE_TAG}" \
  --build-arg TARGETPLATFORM="${PLATFORM}" \
  --build-arg UBUNTU_APT_MIRROR="${UBUNTU_APT_MIRROR}" \
  --build-arg ROS_APT_MIRROR="${ROS_APT_MIRROR}" \
  --build-arg NPM_REGISTRY="${NPM_REGISTRY}" \
  --build-arg HTTP_PROXY="${HTTP_PROXY}" \
  --build-arg HTTPS_PROXY="${HTTPS_PROXY}" \
  --build-arg http_proxy="${http_proxy}" \
  --build-arg https_proxy="${https_proxy}" \
  --build-arg NO_PROXY="${NO_PROXY}" \
  --build-arg no_proxy="${no_proxy}" \
  --build-arg CATKIN_MAKE_ARGS="${CATKIN_MAKE_ARGS}" \
  --load \
  "${ROOT_DIR}"

docker save "${IMAGE}" | gzip > "${ROOT_DIR}/${OUTPUT}"

echo "Saved ${IMAGE} to ${ROOT_DIR}/${OUTPUT}"
