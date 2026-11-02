#!/usr/bin/env bash
# 用 Emscripten 把 cpp/ 下的所有演示编译成 WebAssembly，输出到 public/demos/<name>/
set -euo pipefail
cd "$(dirname "$0")/.."

if ! command -v emcmake >/dev/null 2>&1; then
  echo "未找到 emcmake。macOS 可执行: brew install emscripten" >&2
  exit 1
fi

emcmake cmake -S cpp -B build/web -DCMAKE_BUILD_TYPE=Release
cmake --build build/web -j
echo "Web 演示已输出到 public/demos/"
