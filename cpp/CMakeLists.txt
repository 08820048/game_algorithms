cmake_minimum_required(VERSION 3.20)
project(game_algorithms LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# raylib：桌面端和 Web 端（Emscripten）共用同一份演示代码
# ---------------------------------------------------------------------------
include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
if(EMSCRIPTEN)
  set(PLATFORM Web CACHE STRING "" FORCE)
endif()
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 5.5
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(raylib)

# 把演示画布用的等宽字体（JetBrains Mono 子集，OFL 协议）嵌入为 C 数组，桌面与 Web 都无需额外文件
set(GA_FONT_FILE "${CMAKE_SOURCE_DIR}/common/assets/JetBrainsMono-Medium-subset.ttf")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${GA_FONT_FILE}")
file(READ "${GA_FONT_FILE}" GA_FONT_HEX HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," GA_FONT_BYTES "${GA_FONT_HEX}")
file(WRITE "${CMAKE_BINARY_DIR}/generated/ga_font_data.h"
  "#pragma once\n// 由 CMake 生成，请勿手动编辑\nstatic const unsigned char kGaFontData[] = {${GA_FONT_BYTES}};\n")

# 演示公共框架
add_library(ga_demo STATIC common/demo.cpp common/ui.cpp)
target_include_directories(ga_demo PUBLIC common PRIVATE "${CMAKE_BINARY_DIR}/generated")
target_link_libraries(ga_demo PUBLIC raylib)

# 网页产物输出目录：<repo>/public/demos/<name>/index.{html,js,wasm}
set(GA_WEB_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/../public/demos" CACHE PATH "Web demo output directory")

# add_demo(<name> <sources...>)
#   桌面端：生成可执行文件 demo_<name>
#   Web 端：生成 public/demos/<name>/index.html，由文章中的 <Demo name="..."/> 嵌入
function(add_demo name)
  set(target demo_${name})
  add_executable(${target} ${ARGN})
  target_link_libraries(${target} PRIVATE ga_demo)
  if(EMSCRIPTEN)
    set_target_properties(${target} PROPERTIES
      OUTPUT_NAME index
      SUFFIX ".html"
      RUNTIME_OUTPUT_DIRECTORY "${GA_WEB_OUTPUT_DIR}/${name}")
    target_link_options(${target} PRIVATE
      -sUSE_GLFW=3
      -sALLOW_MEMORY_GROWTH=1
      -sENVIRONMENT=web
      -sEXPORTED_RUNTIME_METHODS=UTF8ToString
      --shell-file "${CMAKE_SOURCE_DIR}/web/shell.html")
    set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS "${CMAKE_SOURCE_DIR}/web/shell.html")
  endif()
endfunction()

# add_algo_test(<name> <sources...>)：纯算法单元测试，只在桌面端构建
enable_testing()
function(add_algo_test name)
  if(NOT EMSCRIPTEN)
    add_executable(${name} ${ARGN})
    target_include_directories(${name} PRIVATE "${CMAKE_SOURCE_DIR}/common")
    add_test(NAME ${name} COMMAND ${name})
  endif()
endfunction()

# ---------------------------------------------------------------------------
# 序 · 基础工具
# ---------------------------------------------------------------------------
add_demo(game_loop foundations/game_loop/demo.cpp)
add_algo_test(fixed_timestep_test foundations/game_loop/fixed_timestep_test.cpp)

add_demo(vectors foundations/vectors/demo.cpp)
add_algo_test(vectors_test foundations/vectors/vectors_test.cpp)

add_demo(easing foundations/interpolation/easing_demo.cpp)
add_demo(follow foundations/interpolation/follow_demo.cpp)
add_algo_test(interpolation_test foundations/interpolation/interpolation_test.cpp)

add_demo(random foundations/random/demo.cpp)
add_algo_test(random_test foundations/random/random_test.cpp)

# ---------------------------------------------------------------------------
# 模块一 · 寻路
# ---------------------------------------------------------------------------
add_demo(bfs pathfinding/bfs/demo.cpp)
add_algo_test(bfs_test pathfinding/bfs/bfs_test.cpp)

add_demo(dijkstra pathfinding/dijkstra/demo.cpp)
add_algo_test(dijkstra_test pathfinding/dijkstra/dijkstra_test.cpp)

add_demo(astar pathfinding/astar/demo.cpp)
add_algo_test(astar_test pathfinding/astar/astar_test.cpp)

add_demo(astar_opt pathfinding/astar_opt/demo.cpp)
add_algo_test(astar_opt_test pathfinding/astar_opt/astar_opt_test.cpp)
# 性能对比 pathfinding/astar_opt/astar_bench.cpp 不在这里构建，用法见文件开头

add_demo(jps pathfinding/jps/demo.cpp)
add_algo_test(jps_test pathfinding/jps/jps_test.cpp)

add_demo(navmesh pathfinding/navmesh/demo.cpp)
add_algo_test(navmesh_test pathfinding/navmesh/navmesh_test.cpp)

add_demo(flowfield pathfinding/flowfield/demo.cpp)
add_algo_test(flowfield_test pathfinding/flowfield/flowfield_test.cpp)
# 性能对比 pathfinding/flowfield/flowfield_bench.cpp 同样手动编译

add_demo(hpa pathfinding/hpa/demo.cpp)
add_algo_test(hpa_test pathfinding/hpa/hpa_test.cpp)

# ---------------------------------------------------------------------------
# 模块二 · 碰撞检测与物理
# ---------------------------------------------------------------------------
add_demo(intersect collision/intersect/demo.cpp)
add_algo_test(intersect_test collision/intersect/intersect_test.cpp)
add_demo(sat collision/sat/demo.cpp)
add_algo_test(sat_test collision/sat/sat_test.cpp)
add_demo(gjk collision/gjk/demo.cpp)
add_algo_test(gjk_test collision/gjk/gjk_test.cpp)
add_demo(spatial_hash collision/spatial_hash/demo.cpp)
add_algo_test(spatial_hash_test collision/spatial_hash/spatial_hash_test.cpp)
# 性能测试 collision/spatial_hash/spatial_hash_bench.cpp 手动编译，用法见文件开头
add_demo(quadtree collision/quadtree/demo.cpp)
add_algo_test(quadtree_test collision/quadtree/quadtree_test.cpp)
# 性能测试 collision/quadtree/quadtree_bench.cpp 手动编译，用法见文件开头
add_demo(bvh collision/bvh/demo.cpp)
add_algo_test(bvh_test collision/bvh/bvh_test.cpp)
# 性能测试 collision/bvh/bvh_bench.cpp 手动编译，用法见文件开头
add_demo(integration collision/integration/demo.cpp)
add_algo_test(integration_test collision/integration/integration_test.cpp)
# 精度与开销对比 collision/integration/integration_bench.cpp 手动编译，用法见文件开头
