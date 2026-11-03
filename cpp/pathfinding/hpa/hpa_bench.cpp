// HPA* 与 A* 的对比：256×256 地图，随机起终点
//   noise：20% 墙、15% 森林、7% 沼泽，零碎障碍很多
//   rooms：随机矩形障碍 + 大片空地（更接近真实的关卡）
// 输出预处理耗时、抽象图规模、每次查询耗时、路径代价比最优多出多少
// 用法（在 cpp/ 目录下）：
//   c++ -O2 -std=c++20 -I common pathfinding/hpa/hpa_bench.cpp -o hpa_bench && ./hpa_bench

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "../../foundations/random/random.h"
#include "../astar_opt/pathfinder.h"
#include "hpa.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

template <class F>
double millis(F&& f, int runs = 5) {
    double best = 1e30;
    for (int run = 0; run < runs; ++run) {
        auto t0 = Clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
    }
    return best;
}

static void runMap(const char* name, const CostGrid& grid, Pcg32& rng) {
    const int kSize = grid.width();
    Pathfinder pf(grid);
    std::vector<std::pair<Cell, Cell>> queries;
    std::vector<int> optimal;
    while (queries.size() < 300) {
        Cell a{static_cast<int>(rng.below(kSize)), static_cast<int>(rng.below(kSize))};
        Cell b{static_cast<int>(rng.below(kSize)), static_cast<int>(rng.below(kSize))};
        if (!grid.passable(a) || !grid.passable(b)) continue;
        auto p = pf.find(a, b);
        if (p.empty()) continue;
        queries.push_back({a, b});
        optimal.push_back(pathCost(grid, p));
    }
    long sink = 0;
    long astarExpanded = 0;
    for (auto [a, b] : queries) {
        pf.find(a, b);
        astarExpanded += pf.expandedCount();
    }
    double astar = millis([&] {
        for (auto [a, b] : queries) sink += static_cast<long>(pf.find(a, b).size());
    });
    std::printf("[%s]\n  A* (Pathfinder)  %7.1f us/query  expanded %6.0f\n", name, astar * 1000 / queries.size(),
                static_cast<double>(astarExpanded) / queries.size());

    for (int k : {8, 16, 32}) {
        HpaGraph hpa;
        double build = millis([&] { hpa.build(grid, k); }, 3);
        size_t edges = 0;
        for (const auto& n : hpa.nodes()) edges += n.edges.size();
        double ratio = 0, worst = 0;
        long expanded = 0;
        for (size_t i = 0; i < queries.size(); ++i) {
            HpaStats st;
            auto p = hpa.findPath(queries[i].first, queries[i].second, &st);
            double r = static_cast<double>(pathCost(grid, p)) / optimal[i];
            ratio += r;
            worst = std::max(worst, r);
            expanded += st.abstractExpanded;
        }
        double t = millis([&] {
            for (auto [a, b] : queries) sink += static_cast<long>(hpa.findPath(a, b).size());
        });
        std::printf("  HPA* k=%-2d  build %6.1f ms  nodes %5zu  edges %6zu  %7.1f us/query  expanded %5.0f  cost x%.3f avg, x%.3f worst\n",
                    k, build, hpa.nodes().size(), edges, t * 1000 / queries.size(),
                    static_cast<double>(expanded) / queries.size(), ratio / queries.size(), worst);
    }
    if (sink == 0) std::puts("");
}

int main() {
    constexpr int kSize = 256;
    Pcg32 rng(9);
    CostGrid noise(kSize, kSize);
    for (int i = 0; i < noise.size(); ++i) {
        uint32_t r = rng.below(100);
        if (r < 20) noise.setWall(noise.cellAt(i), true);
        else if (r < 35) noise.setCost(noise.cellAt(i), 3);
        else if (r < 42) noise.setCost(noise.cellAt(i), 6);
    }
    CostGrid rooms(kSize, kSize);
    for (int k = 0; k < 220; ++k) {
        int x = static_cast<int>(rng.below(kSize)), y = static_cast<int>(rng.below(kSize));
        int w = 2 + static_cast<int>(rng.below(14)), h = 2 + static_cast<int>(rng.below(14));
        for (int yy = y; yy < std::min(kSize, y + h); ++yy)
            for (int xx = x; xx < std::min(kSize, x + w); ++xx) rooms.setWall({xx, yy}, true);
    }
    runMap("noise", noise, rng);
    runMap("rooms", rooms, rng);
}
