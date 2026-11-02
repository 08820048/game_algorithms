#!/usr/bin/env bash
# 桌面端构建：编译全部演示（可直接运行 build/desktop/demo_xxx）并运行算法单元测试
set -euo pipefail
cd "$(dirname "$0")/.."

cmake -S cpp -B build/desktop -DCMAKE_BUILD_TYPE=Debug
cmake --build build/desktop -j
ctest --test-dir build/desktop --output-on-failure
