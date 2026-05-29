#!/usr/bin/env bash
set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/third_party/build"

mkdir -p "${BUILD_DIR}" && cd "${BUILD_DIR}"
cmake "${PROJECT_ROOT}/third_party"
cmake --build . -j"$(sysctl -n hw.ncpu)"
cmake --install .
