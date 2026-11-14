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

# 不让编译器把 a*b + c 合并成 FMA：wasm 没有 FMA，桌面端（arm64 上 clang 默认合并）不关掉的话，
# 同一份确定性模拟在桌面和网页上结果不同（见 6.4 定点数与确定性计算）
add_compile_options(-ffp-contract=off)

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
add_demo(impulse collision/impulse/demo.cpp)
add_algo_test(impulse_test collision/impulse/impulse_test.cpp)
# 稳定性与开销 collision/impulse/impulse_bench.cpp 手动编译，用法见文件开头
add_demo(ccd collision/ccd/demo.cpp)
add_algo_test(ccd_test collision/ccd/ccd_test.cpp)
# 隧穿率与开销 collision/ccd/ccd_bench.cpp 手动编译，用法见文件开头

# ---------------------------------------------------------------------------
# 模块三 · 游戏 AI
# ---------------------------------------------------------------------------
add_demo(fsm ai/fsm/demo.cpp)
add_algo_test(fsm_test ai/fsm/fsm_test.cpp)
# 每帧开销对比 ai/fsm/fsm_bench.cpp 手动编译，用法见文件开头
add_demo(bt ai/bt/demo.cpp)
add_algo_test(bt_test ai/bt/bt_test.cpp)
# 每帧开销 ai/bt/bt_bench.cpp 手动编译，用法见文件开头
add_demo(goap ai/goap/demo.cpp)
add_algo_test(goap_test ai/goap/goap_test.cpp)
# 规划器对比 ai/goap/goap_bench.cpp 手动编译，用法见文件开头
add_demo(utility ai/utility/demo.cpp)
add_algo_test(utility_test ai/utility/utility_test.cpp)
# 组合方式、剪枝、惯性与消融实验 ai/utility/utility_bench.cpp 手动编译，用法见文件开头
add_demo(steering ai/steering/steering_demo.cpp)
add_demo(boids ai/steering/boids_demo.cpp)
add_algo_test(steering_test ai/steering/steering_test.cpp)
# 邻居查询、规则消融、累加顺序与混沌、抵达与避障 ai/steering/steering_bench.cpp 手动编译，用法见文件开头
add_demo(influence ai/influence/demo.cpp)
add_algo_test(influence_test ai/influence/influence_test.cpp)
# 开销、传播延迟、盖章穿墙与避险路径 ai/influence/influence_bench.cpp 手动编译，用法见文件开头
add_demo(alphabeta ai/minimax/tree_demo.cpp)
add_demo(four ai/minimax/connect4_demo.cpp)
add_algo_test(minimax_test ai/minimax/minimax_test.cpp)
# 剪枝与排序的节点数、奇偶效应、对局、小棋盘解到终局 ai/minimax/minimax_bench.cpp 手动编译，用法见文件开头
add_demo(mcts_phases ai/mcts/phases_demo.cpp)
add_demo(mcts ai/mcts/mcts_demo.cpp)
add_algo_test(mcts_test ai/mcts/mcts_test.cpp)
# 迭代次数与探索常数、速度、与 Alpha-Beta 对局、浅层陷阱 ai/mcts/mcts_bench.cpp 手动编译，用法见文件开头
add_demo(fov ai/fov/demo.cpp)
add_demo(guards ai/fov/guards_demo.cpp)
add_algo_test(fov_test ai/fov/fov_test.cpp)
# 各算法的开销、与精确定义的差异、对称性、单点视线 ai/fov/fov_bench.cpp 手动编译，用法见文件开头

# ---------------------------------------------------------------------------
# 模块四 · 程序化生成
# ---------------------------------------------------------------------------
add_demo(noise_1d procgen/noise/lattice_demo.cpp)
add_demo(noise procgen/noise/noise_demo.cpp)
add_algo_test(noise_test procgen/noise/noise_test.cpp)
# 分布、格点结构、频谱、各向异性、导数跳变、开销、float 精度 procgen/noise/noise_bench.cpp 手动编译，用法见文件开头
add_demo(fbm procgen/fractal/fbm_demo.cpp)
add_demo(cells procgen/fractal/cells_demo.cpp)
add_algo_test(fractal_test procgen/fractal/fractal_test.cpp)
# 分布、方向性、影响半径、fBm 频谱、整数点上的零、3×3 搜索的错误率、开销 procgen/fractal/fractal_bench.cpp 手动编译，用法见文件开头
add_demo(terrain procgen/terrain/terrain_demo.cpp)
add_demo(erosion procgen/terrain/erosion_demo.cpp)
add_algo_test(terrain_test procgen/terrain/terrain_test.cpp)
# 高度分布、细节与坡度的相关、热侵蚀的方向性、水力侵蚀与河网、开销 procgen/terrain/terrain_bench.cpp 手动编译，用法见文件开头
add_demo(bsp procgen/dungeon/bsp_demo.cpp)
add_demo(dungeon procgen/dungeon/dungeon_demo.cpp)
add_algo_test(dungeon_test procgen/dungeon/dungeon_test.cpp)
# 随机游走的效率与形状、BSP 的切割与连接、房间的饱和与环路、两次 BFS 的误差、开销 procgen/dungeon/dungeon_bench.cpp 手动编译，用法见文件开头
add_demo(cave procgen/cave/cave_demo.cpp)
add_demo(caves procgen/cave/caves_demo.cpp)
add_algo_test(cave_test procgen/cave/cave_test.cpp)
# 填充率与相变、收敛与周期、原地更新的方向偏差、柱子规则、连通处理、规模、开销 procgen/cave/cave_bench.cpp 手动编译，用法见文件开头
add_demo(poisson procgen/poisson/poisson_demo.cpp)
add_demo(bluenoise procgen/poisson/bluenoise_demo.cpp)
add_algo_test(poisson_test procgen/poisson/poisson_test.cpp)
# 投飞镖的饱和、Bridson 的密度与 k、环带取样、活跃点的选法、最近邻距离、功率谱、开销 procgen/poisson/poisson_bench.cpp 手动编译，用法见文件开头
add_demo(delaunay procgen/voronoi/delaunay_demo.cpp)
add_demo(voronoi procgen/voronoi/voronoi_demo.cpp)
add_algo_test(delaunay_test procgen/voronoi/delaunay_test.cpp)
# 插入顺序与定位步数、空腔大小、超级三角形的缺失、谓词精度、邻居数分布、Lloyd 收敛、开销 procgen/voronoi/delaunay_bench.cpp 手动编译，用法见文件开头
add_demo(wfc procgen/wfc/wfc_demo.cpp)
add_demo(wfc_overlap procgen/wfc/overlap_demo.cpp)
add_algo_test(wfc_test procgen/wfc/wfc_test.cpp)
# 启发式与矛盾率、尺寸与回溯、输出分布的偏差、图块比例、开销 procgen/wfc/wfc_bench.cpp 手动编译，用法见文件开头
add_demo(lsystem procgen/lsystem/lsystem_demo.cpp)
add_demo(lsystem_trees procgen/lsystem/trees_demo.cpp)
add_algo_test(lsystem_test procgen/lsystem/lsystem_test.cpp)
# 长度增长、整串与流式展开的开销、航向的累积误差、盒维数、随机植物的差异 procgen/lsystem/lsystem_bench.cpp 手动编译，用法见文件开头
add_demo(bezier animation/spline/bezier_demo.cpp)
add_demo(catmull_rom animation/spline/catmull_demo.cpp)
add_algo_test(spline_test animation/spline/spline_test.cpp)
# 求值的精度与开销、弧长求积、匀速移动、折线化的段数、参数化与尖点自交 animation/spline/spline_bench.cpp 手动编译，用法见文件开头
add_demo(ik animation/ik/ik_demo.cpp)
add_demo(ik_compare animation/ik/compare_demo.cpp)
add_algo_test(ik_test animation/ik/ik_test.cpp)
# 收敛速度、可达边界附近的减速、弯曲的分布、跟随移动目标、角度限制 animation/ik/ik_bench.cpp 手动编译，用法见文件开头
add_demo(camera animation/camera/camera_demo.cpp)
add_demo(shake animation/camera/shake_demo.cpp)
add_algo_test(camera_test animation/camera/camera_test.cpp)
# 各种相机组合在平台跳跃轨迹上的表现、震屏与帧率、噪声通道错开、trauma 指数、物理与渲染频率不同时的抖动 animation/camera/camera_bench.cpp 手动编译，用法见文件开头
add_demo(particles animation/particles/particles_demo.cpp)
add_demo(particles_integrate animation/particles/integrate_demo.cpp)
add_algo_test(particles_test animation/particles/particles_test.cpp)
# 发射数与帧率、拖尾的均匀程度、积分与帧率、删除策略、内存布局 animation/particles/particles_bench.cpp 手动编译，用法见文件开头
add_demo(culling_frustum animation/culling/frustum_demo.cpp)
add_demo(culling_occlusion animation/culling/occlusion_demo.cpp)
add_algo_test(culling_test animation/culling/culling_test.cpp)
# 平面测试的假阳性、层次剔除与平面掩码、遮挡剔除的分辨率与融合、Hi-Z、只光栅化最近的遮挡物、森林 animation/culling/culling_bench.cpp 手动编译，用法见文件开头

add_demo(handles architecture/pool/handles_demo.cpp)
add_demo(cache_lines architecture/pool/cache_demo.cpp)
add_algo_test(pool_test architecture/pool/pool_test.cpp)
# 分配与销毁、遍历（指针、链表、池的存活率、SlotMap）、按引用访问、步长与缓存行、伪共享、代数回绕 architecture/pool/pool_bench.cpp 手动编译，用法见文件开头
add_demo(ecs_storage architecture/ecs/storage_demo.cpp)
add_demo(ecs_iterate architecture/ecs/iterate_demo.cpp)
add_algo_test(ecs_test architecture/ecs/ecs_test.cpp)
# 移动系统（面向对象、数组、sparse set、archetype）、多组件查询、结构变化的代价、archetype 碎片化 architecture/ecs/ecs_bench.cpp 手动编译，用法见文件开头
add_demo(replay architecture/command/replay_demo.cpp)
add_demo(undo architecture/command/undo_demo.cpp)
add_algo_test(command_test architecture/command/command_test.cpp)
# 录像的存储、混沌（扰动多久放大）、关键帧跳转、撤销的三种做法、反向操作的误差 architecture/command/command_bench.cpp 手动编译，用法见文件开头
add_demo(lockstep architecture/fixed/lockstep_demo.cpp)
add_demo(fixed_orbit architecture/fixed/orbit_demo.cpp)
add_algo_test(fixed_test architecture/fixed/fixed_test.cpp)
# 各种定点格式的精度、轨道误差、加法顺序与 fma 导致的不同步、速度、不同编译选项下的哈希 architecture/fixed/fixed_bench.cpp 手动编译，用法见文件开头
