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
