// 流场与"每个单位各跑一次 A*"的耗时对比：256×256 地图（20% 墙、15% 森林、7% 沼泽），所有单位去同一个目标。
//   A*（1.4 的 Pathfinder，4 方向，复用缓冲区）：每个单位一次
//   Dijkstra 地图（1.2，4 方向）：整张图一次，与单位数量无关
//   积分场 + 流场（本章，8 方向）：整张图一次
// 用法（在 cpp/ 目录下）：
//   c++ -O2 -std=c++20 -I common pathfinding/flowfield/flowfield_bench.cpp -o flowfield_bench && ./flowfield_bench

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

#include "../../foundations/random/random.h"
#include "../astar_opt/pathfinder.h"
#include "flowfield.h"

using namespace ga;
using Clock = std::chrono::steady_clock;

template <class F>
double millis(F&& f) {
    f();
    double best = 1e30;
    for (int run = 0; run < 5; ++run) {
        auto t0 = Clock::now();
        f();
        best = std::min(best, std::chrono::duration<double, std::milli>(Clock::now() - t0).count());
    }
    return best;
}

int main() {
    constexpr int kSize = 256;
    Pcg32 rng(5);
    CostGrid grid(kSize, kSize);
    for (int i = 0; i < grid.size(); ++i) {
        uint32_t r = rng.below(100);
        if (r < 20) grid.setWall(grid.cellAt(i), true);
        else if (r < 35) grid.setCost(grid.cellAt(i), 3);
        else if (r < 42) grid.setCost(grid.cellAt(i), 6);
    }
    Cell goal{kSize / 2, kSize / 2};
    grid.setWall(goal, false);
    auto reach = dijkstraMap(grid, {goal});

    std::vector<Cell> units;
    while (units.size() < 1000) {
        Cell c{static_cast<int>(rng.below(kSize)), static_cast<int>(rng.below(kSize))};
        if (grid.passable(c) && reach[grid.index(c)] != kUnreached) units.push_back(c);
    }

    long sink = 0;
    double dijkstra = millis([&] { sink += dijkstraMap(grid, {goal})[0]; });
    double field = millis([&] { sink += static_cast<long>(flowField(grid, integrationField(grid, {goal})).size()); });
    std::printf("Dijkstra map (4-dir)        %8.2f ms\n", dijkstra);
    std::printf("integration + flow (8-dir)  %8.2f ms\n", field);

    Pathfinder pf(grid);
    for (int n : {1, 10, 100, 1000}) {
        double t = millis([&] {
            for (int i = 0; i < n; ++i) sink += static_cast<long>(pf.find(units[i], goal).size());
        });
        std::printf("A* x %-4d                   %8.2f ms\n", n, t);
    }
    return sink == 0;
}
