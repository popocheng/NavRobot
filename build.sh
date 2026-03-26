#!/usr/bin/env bash
set -e

WS=$(cd "$(dirname "$0")"; pwd)
SRC=$WS/src

echo "Workspace: $WS"

colcon build \
  --symlink-install \
  --parallel-workers $(nproc) \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Build finished"