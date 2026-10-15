#!/usr/bin/env bash
# Сборка: ./build.sh [Release|Debug]   (Git Bash; CMake и Ninja берём из ESP-IDF tools, GCC — Arm GNU Toolchain)
set -e
cd "$(dirname "$0")"
export PATH="/c/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/12.2 mpacbti-rel1/bin:/c/Espressif/tools/cmake/4.0.3/bin:/c/Espressif/tools/ninja/1.12.1:$PATH"
cmake -G Ninja -B build -DCMAKE_BUILD_TYPE="${1:-Release}" >/dev/null
cmake --build build
