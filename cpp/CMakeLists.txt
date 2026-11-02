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

# 演示公共框架
add_library(ga_demo STATIC common/demo.cpp)
target_include_directories(ga_demo PUBLIC common)
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
# 模块一 · 寻路
# ---------------------------------------------------------------------------
add_demo(bfs pathfinding/bfs/demo.cpp)
add_algo_test(bfs_test pathfinding/bfs/bfs_test.cpp)
