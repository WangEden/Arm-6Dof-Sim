#!/usr/bin/env bash
set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/third_party/build"

if [ -n "${BUILD_DIR}" ]; then
    rm -rf "${BUILD_DIR}"
fi

mkdir -p "${BUILD_DIR}" && cd "${BUILD_DIR}"
cmake "${PROJECT_ROOT}/third_party"
cmake --build . -j"$(nproc)"

echo "Creating dummy config files to dynamic-fix libccd install bug..."
touch "${BUILD_DIR}/ccd-config.cmake"
touch "${BUILD_DIR}/ccd-config-version.cmake"
touch "${BUILD_DIR}/ccd.pc"
cmake --install .
echo "Third party dependencies built and installed successfully!"