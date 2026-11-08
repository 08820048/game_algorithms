# 游戏算法图解

用可交互的演示和 C++ 代码，讲清楚游戏开发中的常见算法。计划部署在 https://algo.xuyi.dev 。

- 网站：[Astro](https://astro.build) + [Starlight](https://starlight.astro.build)
- 演示：C++20 + [raylib](https://www.raylib.com)，桌面端直接运行，Web 端通过 Emscripten 编译为 WebAssembly 嵌入文章

## 目录结构

```
cpp/
├── CMakeLists.txt          # add_demo() / add_algo_test() 注册演示与测试
├── common/
│   ├── vec2.h              # 二维向量（0.2 讲解）
│   ├── grid.h              # 网格地图
│   ├── cost_grid.h         # 带地形代价的网格（1.2 起使用）
│   ├── check.h             # 极简单元测试辅助
│   ├── demo.{h,cpp}        # 演示框架：播放/暂停/单步/调速、亮暗主题、Web 控制接口
│   └── ui.{h,cpp}          # 画布内控件：按钮、开关、滑块
├── web/shell.html          # Web 演示页面模板（工具栏、主题同步、自适应高度、离屏暂停）
├── foundations/            # 序 · 基础工具：game_loop / vectors / interpolation / random
├── pathfinding/terrain_map.h  # 带地形网格演示共用的编辑器与绘制（Dijkstra、A*）
├── pathfinding/astar_opt/astar_bench.cpp  # A* 性能对比（不进 CMake，手动 -O2 编译，见文件开头）
├── pathfinding/jps/jps_bench.cpp        # JPS 与 A* 性能对比（同上）
├── pathfinding/flowfield/flowfield_bench.cpp  # 流场与逐单位 A* 对比（同上）
├── pathfinding/hpa/hpa_bench.cpp        # HPA* 与 A* 对比，可传地图边长（同上）
├── collision/sat/sat_bench.cpp          # SAT 耗时、法线单位化与分离轴缓存对比（同上）
├── collision/gjk/gjk_bench.cpp          # GJK / EPA 与 SAT 对比，曲线形状的容差（同上）
├── collision/spatial_hash/spatial_hash_bench.cpp  # 均匀网格 / 空间哈希与两两测试对比（同上）
├── collision/quadtree/quadtree_bench.cpp  # 四叉树 / 八叉树与网格对比（同上）
├── collision/bvh/bvh_bench.cpp          # SAP、动态 BVH 与网格、四叉树对比（同上）
├── collision/integration/integration_bench.cpp  # 积分方法的精度、开销与 float 精度（同上）
├── collision/impulse/impulse_bench.cpp  # 冲量求解器：堆叠、热启动、Baumgarte、牛顿摆（同上）
├── collision/ccd/ccd_bench.cpp          # 隧穿率、扫掠测试与保守前进的开销（同上）
├── ai/fsm/fsm_bench.cpp                 # 状态机每帧开销：switch、扁平、分层、共享定义（同上）
├── ai/bt/bt_bench.cpp                   # 行为树每帧开销、响应式节点的代价（同上）
├── ai/goap/goap_bench.cpp               # GOAP 正向与反向搜索、启发函数对比（同上）
├── ai/utility/utility_bench.cpp         # 效用 AI：组合方式偏差、剪枝开销、惯性与抖动、消融（同上）
├── ai/steering/steering_bench.cpp       # 转向与 Boids：邻居查询、规则消融、累加顺序与混沌、避障组合（同上）
├── ai/influence/influence_bench.cpp     # 影响力图：盖章与传播的开销、穿墙误差、传播拖尾、避险路径（同上）
├── ai/minimax/minimax_bench.cpp         # 博弈树搜索：剪枝、排序、置换表的节点数，奇偶效应，对局，小棋盘解到终局（同上）
├── ai/mcts/mcts_bench.cpp               # 蒙特卡洛树搜索：迭代次数与探索常数、启发式模拟、与 Alpha-Beta 对局、浅层陷阱（同上）
├── ai/fov/fov_bench.cpp                 # 视野：各算法的开销、与精确定义的差异、对称性、单点视线（同上）
├── procgen/noise/noise_bench.cpp        # 噪声：分布、格点痕迹、频谱、梯度方向、导数跳变、开销、float 精度（同上）
├── procgen/fractal/fractal_bench.cpp    # Simplex/Worley/fBm：方向性、影响半径、频谱斜率、整数点的零、3×3 错误率、开销（同上）
└── pathfinding/bfs/
    ├── bfs.h               # 纯算法，不依赖渲染，可直接拷进你的项目
    ├── bfs_test.cpp        # 单元测试
    └── demo.cpp            # 可视化演示
public/demos/<name>/        # Emscripten 产物（提交到仓库，部署时无需 Emscripten）
src/
├── components/
│   ├── Home.astro / Toc.astro   # 首页与书籍式目录
│   ├── overrides/          # 覆盖 Starlight 的页眉标题、站点标识
│   ├── Demo.astro          # <Demo name="bfs" title="..." label="图 1.1" />：嵌入 wasm 演示
│   └── CppSnippet.astro    # <CppSnippet file="..." region="..." />：引用 cpp/ 中的代码片段
├── data/curriculum.ts      # 全书目录与发布状态
├── styles/custom.css       # 技术手稿主题（配色、字体、排版）
└── content/docs/           # 文章（MDX）
scripts/
├── build-demos.sh          # npm run demos:build
└── test-cpp.sh             # npm run cpp:test
```

## 常用命令

| 命令 | 作用 |
|---|---|
| `npm run dev` | 本地开发 http://localhost:4321 |
| `npm run build` | 构建静态站点到 `dist/` |
| `npm run cpp:test` | 桌面端编译全部演示 + 运行单元测试 |
| `npm run demos:build` | 用 Emscripten 编译 Web 演示到 `public/demos/`（需 `brew install emscripten`） |
| `./build/desktop/demo_<name>` | 桌面端运行某个演示，如 `demo_bfs`、`demo_game_loop` |

## 新增一篇文章的流程

1. 在 `cpp/<模块>/<算法>/` 下写 `xxx.h`（纯算法）、`xxx_test.cpp`、`demo.cpp`（继承 `ga::Demo`，实现 `reset / step / draw`，可选 `update` 处理鼠标）
2. 在 `cpp/CMakeLists.txt` 中加入 `add_demo(<name> ...)` 和 `add_algo_test(...)`
3. 在需要被文章引用的代码两侧加 `// #region <名字>` / `// #endregion`
4. `npm run cpp:test` → `npm run demos:build`
5. 在 `src/content/docs/<模块>/` 下写 MDX，用 `<Demo>` 和 `<CppSnippet>` 嵌入演示与代码
6. 在 `src/data/curriculum.ts` 中把对应章节改为 `status: 'published'` 并填上 `href`（首页目录、路线图、文章页眉都读这份数据）；新的一卷需要在 `astro.config.mjs` 的 `sidebar` 中加一个分组

### 约定

- 演示画布内的文字使用英文（raylib 默认字体不含中文），中文说明写在文章和图注里
- 算法头文件不依赖 raylib，演示代码只负责可视化
- 文章中的代码一律通过 `<CppSnippet>` 引用，不要手动复制，保证与测试过的代码一致

## 部署（Cloudflare Pages）

- Build command：`npm run build`
- Output directory：`dist`
- 自定义域名：`algo.xuyi.dev`

wasm 产物已提交在 `public/demos/` 中，因此构建环境不需要 Emscripten。修改 C++ 代码后记得重新运行 `npm run demos:build` 并提交产物。

## 版权与许可

© 2026 XuYi

- **代码**（`cpp/`、`scripts/`、站点组件与配置，以及 `public/demos/` 中编译好的演示）：[MIT](LICENSE)
- **文章与图示**（`src/content/docs/`、`src/components/figures/`）：[CC BY-NC-SA 4.0](LICENSE-CONTENT)，转载请署名并附原文链接，禁止商业使用
